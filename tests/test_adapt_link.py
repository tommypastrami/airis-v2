"""ADAPT link: framing roundtrip, resync after garbage, outbound on the same connection."""
import socket
import time

from airis import config
from airis.adapt_link import Envelope, JsonCodec, Listener, Sender, frame, read_frame
from sim.fake_adapt import FakeAdapt


def test_frame_roundtrip_over_socketpair():
    a, b = socket.socketpair()
    codec = JsonCodec()
    env = Envelope("gcn_alert", {"burst_id": 7, "ra": 1.5, "dec": -2.5})
    a.sendall(frame(codec.encode(env)))
    got = codec.decode(read_frame(b, 1 << 20))
    assert got.kind == "gcn_alert" and got.fields["burst_id"] == 7 and got.fields["ra"] == 1.5


def test_resync_after_garbage():
    a, b = socket.socketpair()
    codec = JsonCodec()
    a.sendall(b"\x00garbage\xff" * 5 + frame(codec.encode(Envelope("ping", {"n": 1}))))
    assert codec.decode(read_frame(b, 1 << 20)).fields["n"] == 1
    # a false sync word with an absurd length is skipped too
    a.sendall(config.ADAPT_SYNC_WORD + b"\xff\xff\xff\xff" + frame(codec.encode(Envelope("ping", {"n": 2}))))
    assert codec.decode(read_frame(b, 1 << 20)).fields["n"] == 2


def test_listener_dispatches_and_sender_replies_on_same_connection():
    codec = JsonCodec()
    sender = Sender(codec)
    got = []
    lst = Listener(0, codec, got.append, timeout_s=1.0, max_bytes=1 << 20, sender=sender, host="127.0.0.1")
    lst.start()
    adapt = FakeAdapt("127.0.0.1", lst.bound_port)
    try:
        adapt.send(Envelope("gcn_alert", {"burst_id": 1, "ra": 10.0, "dec": 20.0}))
        t0 = time.monotonic()
        while not got and time.monotonic() - t0 < 3:
            time.sleep(0.01)
        assert got and got[0].kind == "gcn_alert"
        assert sender.connected()
        assert sender.send(Envelope("internal_message", {"heading": "grb_confirmed", "message": "x"}))
        back = adapt.wait_for("internal_message")
        assert back and back[0].fields["heading"] == "grb_confirmed"
    finally:
        adapt.close(); lst.stop.set()


def test_half_open_link_is_dropped_after_timeouts(monkeypatch):
    monkeypatch.setattr(config, "ADAPT_DEAD_AFTER_TIMEOUTS", 3)
    codec = JsonCodec()
    sender = Sender(codec)
    lst = Listener(0, codec, lambda e: None, timeout_s=0.1, max_bytes=1 << 20, sender=sender, host="127.0.0.1")
    lst.start()
    adapt = FakeAdapt("127.0.0.1", lst.bound_port)
    try:
        t0 = time.monotonic()
        while not sender.connected() and time.monotonic() - t0 < 2:
            time.sleep(0.01)
        assert sender.connected()
        t0 = time.monotonic()
        while sender.connected() and time.monotonic() - t0 < 3:
            time.sleep(0.02)
        assert not sender.connected()          # silence -> link dropped, sender detached (no silent half-open)
    finally:
        adapt.close(); lst.stop.set()
