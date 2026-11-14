"""End to end against the simulators: alert -> miss -> map -> hit -> downlink; refusals move on; safe mode; pipelining."""
import time
from datetime import datetime, timedelta, timezone

import healpy as hp
import numpy as np
import pytest

from airis import config
from airis.adapt_link import Envelope, JsonCodec, Sender
from airis.airis_main import Orchestrator
from airis.detect import Pipeline
from airis.downlink import Downlink
from airis.nav import NavState
from airis.pointing import MountPos, Site, SunBox, radec_to_altaz
from airis.scheduler import TileQueue
from sim.sky_sim import FakeCamera, SkyModel
from tests.conftest import make_bridge
from tests.test_downlink import RecordingSender
from tests.test_scheduler import gaussian_map

SITE = Site(31.78, -95.63, 159.0)
SKY = SkyModel(seed=9, ra0=83.8, dec0=-5.4)
TRUE = (SKY.ra0 + 0.6, SKY.dec0 + 0.2)


def good_time() -> datetime:
    t = datetime(2026, 12, 15, 0, 0, tzinfo=timezone.utc)
    for h in range(24):
        alt, _ = radec_to_altaz(SKY.ra0, SKY.dec0, SITE, t + timedelta(hours=h))
        if alt > 45:
            return t + timedelta(hours=h)
    raise RuntimeError("Orion never high enough?")


def build(tmp_path, fast_link, transient=True, mount=None):
    bridge, fa = make_bridge(tmp_path, mount=mount)
    assert bridge.calibrate().ok
    cam = FakeCamera(SKY, transient=(TRUE[0], TRUE[1], 7000.0) if transient else None, rot_deg=9.0, seed=8)
    nav = NavState()
    nav.on_adapt(SITE.lat_deg, SITE.lon_deg, SITE.height_m, 2)
    sender = RecordingSender()
    when = good_time()
    orch = Orchestrator(bridge, cam, SKY.catalog, Pipeline(csv_path=str(tmp_path / "cand.csv")),
                        Downlink(sender, queue_dir=str(tmp_path / "q")), sender, nav,
                        tiles=TileQueue(n_candidates=1500), now_utc=lambda: when)
    orch.update_sun()
    return orch, bridge, fa, cam, sender


def test_alert_miss_then_map_hit(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link)
    try:
        # 1. an alert 8 deg off: slew, image, miss
        orch.dispatch(Envelope("gcn_alert", {"burst_id": 42, "ra": TRUE[0] + 8.0, "dec": TRUE[1], "probability_score": 0.6}))
        out = orch.step()
        assert out is not None and out.result == "miss", out
        assert len(fa.moves_executed) >= 2                      # cal offset move + this slew
        # 2. the map arrives centred near the truth: next tile covers it -> hit -> downlink
        prob = gaussian_map(32, TRUE[0], TRUE[1], 3.0)[0]
        orch.dispatch(Envelope("localization_update", {"burst_id": 42, "iteration": 1, "healpix_grid": prob.astype("<f8").tobytes()}))
        out = orch.step()
        assert out.result == "hit", out
        assert out.best_confidence >= config.CONFIDENCE_THRESHOLD
        assert orch.lock_on is not None
        kinds = [e.kind for e in sender.envelopes]
        heads = [e.fields["detail"].replace(":", " ").split()[0] for e in sender.envelopes if e.kind == "event_notification"]
        assert "grb_confirmed" in heads and "image_manifest" in heads and "image_data" in kinds
        assert (tmp_path / "cand.csv").exists()
        # 3. a stale iteration is ignored; a lock-on keeps re-imaging the same tile
        orch.dispatch(Envelope("localization_update", {"burst_id": 42, "iteration": 0, "healpix_grid": prob.tobytes()}))
        out2 = orch.step()
        assert out2.tile.ra == out.tile.ra and out2.result == "hit"
        # ...but only LOCK_ON_REVISITS times: then the queue resumes
        for _ in range(config.LOCK_ON_REVISITS - 1):
            assert orch.step().tile.ra == out.tile.ra
        assert orch.lock_on is None
        nxt = orch.step()
        assert nxt is None or (nxt.tile.ra, nxt.tile.dec) != (out.tile.ra, out.tile.dec)
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_refused_slew_moves_on_to_next_tile(tmp_path, fast_link):
    """v1: a refused slew abandoned the burst. v2: the tile is marked, the next one is queued."""
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link, transient=False)
    try:
        prob = gaussian_map(32, TRUE[0], TRUE[1], 15.0)[0]      # a wide map: tiles exist beyond the exclusion radius
        orch.dispatch(Envelope("localization_update", {"burst_id": 1, "iteration": 1, "healpix_grid": prob.tobytes()}))
        best = orch.tiles.peek()
        fa.sun_el_phys, fa.sun_az_phys = best.el, best.az        # firmware refuses the best tile (sun-facing)
        out = orch.step()
        assert out.result == "slew_failed" and out.code == "MNT-57"
        assert orch.refused and len(orch.tiles) > 0                 # the refusal became an exclusion, not an abandonment
        nxt = orch.step()
        assert nxt is not None and nxt.result == "miss", nxt
        assert max(abs(nxt.tile.el - best.el), abs(nxt.tile.az - best.az)) > 15.0
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_jetson_side_sun_exclusion_never_queues_sun_tiles(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link, transient=False)
    try:
        prob = gaussian_map(32, TRUE[0], TRUE[1], 2.0)[0]
        orch.dispatch(Envelope("localization_update", {"burst_id": 1, "iteration": 1, "healpix_grid": prob.tobytes()}))
        best = orch.tiles.peek()
        orch.sun_box = SunBox(MountPos(best.el, best.az))
        orch._rescore()
        assert len(orch.tiles) == 0 or all(abs(t.el - best.el) > 5 or abs(t.az - best.az) > 5 for t in orch.tiles._heap)
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_overheat_enters_safe_mode_until_ground_clears(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link)
    try:
        cam.temp_c = 75.0
        orch.dispatch(Envelope("gcn_alert", {"burst_id": 3, "ra": TRUE[0], "dec": TRUE[1], "probability_score": 0.9}))
        out = orch.step()
        assert out.result == "overheat" and orch.safe_mode.is_set()
        assert orch.step() is None
        cam.temp_c = 20.0
        orch.dispatch(Envelope("safe_mode_exit_cmd", {"force": False}))
        assert not orch.safe_mode.is_set()
        orch.dispatch(Envelope("gcn_alert", {"burst_id": 4, "ra": TRUE[0], "dec": TRUE[1], "probability_score": 0.9}))
        assert orch.step().result == "hit"
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_comms_modes_and_pointing_cmd(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link)
    try:
        orch.dispatch(Envelope("comms_directive", {"mode": "MODE_SLOW"}))      # enum name, as ProtoCodec decodes it
        assert orch.downlink.mode == "SLOW"
        n0 = len(fa.moves_executed)
        orch.dispatch(Envelope("pointing_cmd", {"target_elevation": 40.0, "target_azimuth": -30.0}))
        assert len(fa.moves_executed) == n0                # queued: the listener thread never slews
        assert orch.step() is None                         # one slew, done by the acquire side
        assert fa.moves_executed[-1] == (40.0, -30.0) and len(fa.moves_executed) == n0 + 1
        orch.dispatch(Envelope("comms_directive", {"mode": 2}))                # MODE_STOP = 2 in the proto
        assert orch.downlink.mode == "STOP"
        orch.dispatch(Envelope("comms_directive", {"mode": "MODE_FULL_UNSPECIFIED"}))
        assert orch.downlink.mode == "FULL"
        orch.dispatch(Envelope("comms_directive", {"mode": 7}))                # unknown: ignored
        assert orch.downlink.mode == "FULL"
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


class DownMount:
    """A mount whose bridge is unreachable, then comes back."""

    def __init__(self, real):
        self.real, self.down, self.code, self.attempts = real, True, "LNK-03", 0

    def move(self, el, az):
        self.attempts += 1
        if self.down:
            from airis.bridge_client import ClientResult
            return ClientResult(False, self.code, "bridge unreachable")
        return self.real.move(el, az)

    def set_sun(self, el, az):
        return self.real.set_sun(el, az)

    def calibrate(self):
        return self.real.calibrate()


def test_unavailable_mount_does_not_burn_the_queue(tmp_path, fast_link):
    """An unavailable mount is not the tile's fault: nothing is retired, and the tile is imaged once it is back."""
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link, transient=False)
    try:
        orch.mount = DownMount(bridge)
        prob = gaussian_map(32, TRUE[0], TRUE[1], 6.0)[0]
        orch.dispatch(Envelope("localization_update", {"burst_id": 5, "iteration": 1, "healpix_grid": prob.tobytes()}))
        n_queued = len(orch.tiles)
        first = orch.tiles.peek()
        for code in ("LNK-03", "LNK-04", "MNT-63", "MNT-64", "MNT-65", "MNT-59"):
            orch.mount.code = code
            for _ in range(4):
                out = orch.step()
                assert out.result == "slew_retry" and out.code == code
        assert not orch.tiles.imaged and len(orch.tiles) == n_queued
        orch.mount.down = False
        out = orch.step()
        assert out.result == "miss" and (out.tile.ra, out.tile.dec) == (first.ra, first.dec)
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_repeated_settle_failure_retires_the_tile_after_retries(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link, transient=False)
    try:
        orch.mount = DownMount(bridge)
        orch.mount.code = "MNT-58"
        orch.dispatch(Envelope("gcn_alert", {"burst_id": 6, "ra": TRUE[0], "dec": TRUE[1]}))
        results = [orch.step().result for _ in range(config.SLEW_MAX_RETRIES)]
        assert results == ["slew_retry"] * (config.SLEW_MAX_RETRIES - 1) + ["slew_failed"]
        assert orch.step() is None
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_worker_threads_survive_exceptions(tmp_path, fast_link):
    """A camera or detector that raises must not end the science loop."""
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link, transient=False)
    try:
        real_capture, real_run = cam.capture, orch.detector.run
        boom = {"capture": 2, "run": 2}

        def capture(*a, **k):
            if boom["capture"]:
                boom["capture"] -= 1
                raise RuntimeError("camera glitch")
            return real_capture(*a, **k)

        def run(*a, **k):
            if boom["run"]:
                boom["run"] -= 1
                raise OSError("disk full")
            return real_run(*a, **k)

        cam.capture, orch.detector.run = capture, run
        prob = gaussian_map(32, TRUE[0], TRUE[1], 6.0)[0]
        orch.dispatch(Envelope("localization_update", {"burst_id": 9, "iteration": 1, "healpix_grid": prob.tobytes()}))
        orch.start()
        t0 = time.monotonic()
        while sum(o.result == "miss" for o in orch.outcomes) < 2 and time.monotonic() - t0 < 40:
            time.sleep(0.05)
        assert orch.threads_alive()
        assert sum(o.result == "miss" for o in orch.outcomes) >= 2
        assert sum(o.result == "detect_failed" for o in orch.outcomes) == 2
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_bad_map_does_not_clear_the_current_queue(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link, transient=False)
    try:
        prob = gaussian_map(32, TRUE[0], TRUE[1], 6.0)[0]
        orch.dispatch(Envelope("localization_update", {"burst_id": 1, "iteration": 1, "healpix_grid": prob.tobytes()}))
        n = len(orch.tiles)
        orch.dispatch(Envelope("localization_update", {"burst_id": 2, "iteration": 1, "healpix_grid": bytes(256)}))
        orch.dispatch(Envelope("localization_update", {"burst_id": 2, "iteration": 1, "healpix_grid": bytes(13)}))
        assert orch.current_burst == 1 and len(orch.tiles) == n
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_overheat_parks_once_and_tells_the_ground(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link)
    try:
        cam.temp_c = 75.0
        orch.dispatch(Envelope("gcn_alert", {"burst_id": 3, "ra": TRUE[0], "dec": TRUE[1]}))
        assert orch.step().result == "overheat"
        assert any(e.fields.get("event_type") == "EVENT_SAFE_MODE_ENTER" for e in sender.envelopes)
        n0 = len(fa.moves_executed)
        assert orch.step() is None and orch.step() is None
        assert fa.moves_executed[n0:] == [(config.SAFE_PARK_EL_DEG, config.SAFE_PARK_AZ_DEG)]
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()


def test_threaded_pipeline_overlaps_slew_and_analysis(tmp_path, fast_link):
    orch, bridge, fa, cam, sender = build(tmp_path, fast_link, transient=False)
    try:
        prob = gaussian_map(32, TRUE[0], TRUE[1], 6.0)[0]
        orch.dispatch(Envelope("localization_update", {"burst_id": 9, "iteration": 1, "healpix_grid": prob.tobytes()}))
        orch.start()
        t0 = time.monotonic()
        while len(orch.outcomes) < 3 and time.monotonic() - t0 < 30:
            time.sleep(0.05)
        assert len(orch.outcomes) >= 3
        assert len(cam.captures) >= len(orch.outcomes)        # acquire ran ahead of analysis
        assert all(o.result in ("miss", "hit") for o in orch.outcomes)
    finally:
        orch.shutdown(); bridge.close(); fa.shutdown()
