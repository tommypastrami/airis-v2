"""Bridge + fake Arduino + mount sim: the regression suite for Goal 1.

Each test states the v4 failure it guards against.
"""
import csv
import os
import socket
import threading
import time

import pytest

from airis import config
from airis.airis_bridge import Bridge, RequestServer
from airis.bridge_client import MountClient
from airis.errors import Code
from airis.transport import SocketTransport
from sim.fake_arduino import Faults, make_pair
from sim.mount_sim import INDEX_AZ_PHYS, INDEX_EL_PHYS, MountSim
from tests.conftest import make_bridge


def _wait(pred, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_calibrate_anchors_frame(bridge_and_fw):
    """v4 bug #2: CAL alone left the encoders in the index frame; MOVE 0 0 went to the index pulse."""
    bridge, fa = bridge_and_fw
    r = bridge.calibrate()
    assert r.ok, r
    assert fa.homed
    # after CAL + offset move + SETZERO, encoder zero IS the physical home
    assert abs(fa.mount.el.enc_offset) < 0.05 and abs(fa.mount.az.enc_offset) < 0.05
    m = bridge.move(0.0, 0.0)
    assert m.ok, m
    el, az = fa.mount.physical()
    assert abs(el) < config.SETTLE_TOL_DEG and abs(az) < config.SETTLE_TOL_DEG


def test_home_is_repeatable_across_calibrations(tmp_path, fast_link):
    """Symptom B: 0,0 must land on the same physical spot after every CAL, even from a different start."""
    spots = []
    for start in [(10.0, -20.0), (-40.0, 90.0), (120.0, -150.0)]:
        bridge, fa = make_bridge(tmp_path, mount=MountSim.at_boot(*start))
        try:
            assert bridge.calibrate().ok
            assert bridge.move(0.0, 0.0).ok
            spots.append(fa.mount.physical())
        finally:
            bridge.close(); fa.shutdown()
    for el, az in spots:
        assert abs(el) < 1.5 and abs(az) < 1.5, spots


def test_move_without_setzero_is_wrong_by_the_offset(tmp_path, fast_link):
    """The v4 behaviour, reproduced: skip the anchor step and 0,0 is 55/35 deg off."""
    bridge, fa = make_bridge(tmp_path)
    try:
        bridge.send("CAL")
        assert _wait(lambda: fa.homed, 5.0)
        bridge.homed = True                      # pretend, as v4 did
        assert bridge._move_once(0.0, 0.0).ok
        el, az = fa.mount.physical()
        assert abs(el - INDEX_EL_PHYS) < 1.5 and abs(az - INDEX_AZ_PHYS) < 1.5
    finally:
        bridge.close(); fa.shutdown()


def test_clean_move_reports_residual_and_logs_csv(bridge_and_fw, tmp_path):
    bridge, fa = bridge_and_fw
    assert bridge.calibrate().ok
    r = bridge.move(60.0, 0.0)
    assert r.ok and r.code == "OK"
    assert abs(r.el_err) <= config.SETTLE_TOL_DEG and abs(r.az_err) <= config.SETTLE_TOL_DEG
    assert fa.moves_executed[-1] == (60.0, 0.0)
    rows = list(csv.DictReader(open(bridge.moves_csv)))
    assert rows[-1]["el_cmd"] == "60.0" and rows[-1]["code"] == "OK"


def test_garbled_first_send_executes_exactly_one_move(tmp_path, fast_link):
    """v4 bug #4: a garbled first attempt produced a false failure AND a ghost move.
    v5: the firmware NAKs the damaged line, the resend executes, the host sees ok."""
    faults = Faults(garble_after_idle_s=2.0, garble_bytes=3)
    bridge, fa = make_bridge(tmp_path, faults=faults)
    try:
        assert bridge.calibrate().ok
        time.sleep(0.2)                          # > 2 sim-seconds idle at 25x
        n0 = len(fa.moves_executed)
        r = bridge.move(45.0, 30.0)
        assert r.ok, r
        assert len(fa.moves_executed) == n0 + 1
        assert fa.naks >= 1
    finally:
        bridge.close(); fa.shutdown()


def test_corrupted_acks_never_cause_double_execution(tmp_path, fast_link):
    """Damaged replies: resends carry the same seq, the firmware deduplicates."""
    bridge, fa = make_bridge(tmp_path, faults=Faults(tx_corrupt_prob=0.2, seed=11))
    try:
        ok, attempts = 0, 0
        while attempts < 8 and ok < 2:
            attempts += 1
            if not bridge.homed:
                bridge.calibrate()
            n0 = len(fa.moves_executed)
            r = bridge.move(20.0, 10.0)
            if r.ok:
                ok += 1
            assert len(fa.moves_executed) - n0 <= 1          # never a ghost move, whatever the link does
        assert ok >= 2, "the link should still get moves through with 20% line damage"
        assert fa.naks + sum(1 for l in fa.log if l.startswith("ACK")) > len(fa.moves_executed)  # resends happened
    finally:
        bridge.close(); fa.shutdown()


def test_busy_is_immediate_not_a_recal(bridge_and_fw):
    """v4 bug #3: a busy answer entered the re-CAL loop. v5: MNT-63 at once, no CAL sent."""
    bridge, fa = bridge_and_fw
    assert bridge.calibrate().ok
    cal_count = sum(1 for l in fa.log if l.startswith("RX: CAL"))
    results = {}

    def mover(name, el):
        results[name] = bridge.move(el, 0.0)

    t1 = threading.Thread(target=mover, args=("a", 90.0)); t1.start()
    time.sleep(0.05)
    t2 = threading.Thread(target=mover, args=("b", -90.0)); t2.start()
    t1.join(5); t2.join(5)
    codes = sorted(r.code for r in results.values())
    assert codes == [Code.MNT_BUSY.value, "OK"], results
    assert sum(1 for l in fa.log if l.startswith("RX: CAL")) == cal_count


def test_not_homed_triggers_one_recalibration(tmp_path, fast_link):
    bridge, fa = make_bridge(tmp_path)
    try:
        bridge.homed = True                      # host believes it is homed; firmware does not
        r = bridge.move(30.0, 30.0)
        assert r.ok and r.recalibrated and r.attempts == 2
        assert sum(1 for l in fa.log if l.startswith("RX: CAL")) == 1
    finally:
        bridge.close(); fa.shutdown()


def test_reset_is_detected_and_recovered(tmp_path, fast_link):
    """A board reset (DTR) un-homes the mount; the next move recalibrates once instead of failing forever."""
    bridge, fa = make_bridge(tmp_path)
    try:
        assert bridge.calibrate().ok
        fa.reset()
        assert _wait(lambda: not bridge.homed, 2.0)
        r = bridge.move(10.0, 10.0)
        assert r.ok and r.recalibrated
    finally:
        bridge.close(); fa.shutdown()


def test_stall_is_reported_as_settle_fail_without_touching_az(tmp_path, fast_link):
    """v4: an EL stall nudged AZ +/-10 deg, unbounded, then the bridge detoured 40 deg.
    v5: bounded settle, ERR 58 with the residual, AZ untouched."""
    mount = MountSim.at_boot()
    bridge, fa = make_bridge(tmp_path, mount=mount)
    try:
        assert bridge.calibrate().ok
        assert bridge.move(0.0, 20.0).ok
        mount.el.stall_at = 30.0
        r = bridge.move(60.0, 20.0)
        assert not r.ok and r.code == Code.MNT_SETTLE.value, r
        assert abs(r.el_err - 30.0) < 2.0
        el, az = mount.physical()
        assert abs(az - 20.0) < 1.5                               # AZ never nudged
        assert fa.settle_passes <= config.SETTLE_MAX_PASSES + 1
    finally:
        bridge.close(); fa.shutdown()


def test_timeout_sends_stop(tmp_path, fast_link, monkeypatch):
    bridge, fa = make_bridge(tmp_path, speed=2.0, cal_duration_s=2.0)   # slow sim: the move cannot finish in time
    try:
        assert bridge.calibrate().ok
        monkeypatch.setattr(config, "MOVE_MAX_S", 0.05)
        monkeypatch.setattr(config, "MOVE_MARGIN_S", 0.0)
        r = bridge.move(150.0, 150.0)
        assert not r.ok and r.code == Code.MNT_STOPPED.value
        assert any(l.startswith("RX: STOP") for l in fa.log)
    finally:
        bridge.close(); fa.shutdown()


def test_sun_keepout_refusal_is_a_named_code(bridge_and_fw):
    bridge, fa = bridge_and_fw
    assert bridge.calibrate().ok
    r = bridge.move(-55.0, -35.0)                 # the hard-coded bench sun center
    assert not r.ok and r.code == Code.MNT_SUN.value


def test_live_sun_center_moves_the_box(bridge_and_fw):
    bridge, fa = bridge_and_fw
    assert bridge.calibrate().ok
    assert bridge.set_sun(40.0, 100.0)
    assert bridge.move(-55.0, -35.0).ok           # old center is free now
    r = bridge.move(40.0, 100.0)
    assert not r.ok and r.code == Code.MNT_SUN.value


def test_keepalive_prevents_hold_release(tmp_path, fast_link):
    """v4 bug #9: 5 min of silence released the drivers and the mount sagged."""
    bridge, fa = make_bridge(tmp_path, hold_silence_s=20.0)   # 20 sim-s = 0.8 real-s at 25x
    try:
        assert bridge.calibrate().ok
        assert bridge.move(30.0, 0.0).ok
        time.sleep(2.5)                                        # ~60 sim-seconds idle
        assert fa.mount.drivers_on, "hold lapsed"
        assert not any("HOLD TIMEOUT" in l for l in fa.log)
    finally:
        bridge.close(); fa.shutdown()


def test_link_silence_reopens_port_and_recovers(tmp_path, fast_link):
    bridge, fa = make_bridge(tmp_path)
    try:
        assert bridge.calibrate().ok
        fa.mute = True                                           # the cable falls out
        assert _wait(lambda: bridge.reopened >= 1, 6.0)
        fa.mute = False                                          # and comes back
        assert _wait(lambda: bridge.query_status() is not None, 3.0)
        assert bridge.move(5.0, 5.0).ok
    finally:
        bridge.close(); fa.shutdown()


def test_request_server_and_client(tmp_path, fast_link):
    """End to end over the Unix socket: accepted/done, and a second caller gets busy at once."""
    bridge, fa = make_bridge(tmp_path)
    path = str(tmp_path / "b.sock")
    srv = RequestServer(bridge, path); srv.start()
    client = MountClient(path, timeout_s=10.0)
    try:
        assert client.calibrate().ok
        out = {}
        t = threading.Thread(target=lambda: out.setdefault("a", client.move(80.0, 10.0))); t.start()
        time.sleep(0.05)
        b = client.move(-80.0, 10.0)
        t.join(5)
        assert out["a"].ok and abs(out["a"].el_err) <= 1.5
        assert not b.ok and b.code == Code.MNT_BUSY.value
        assert client.status().ok
        assert client.set_sun(10.0, 10.0).ok
    finally:
        bridge.close(); fa.shutdown()
