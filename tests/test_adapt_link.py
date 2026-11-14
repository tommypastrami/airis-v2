"""ADAPT link: framing, resync, senders, reconnects, and the UDP path to the comms server."""
import socket
import threading
import time

from airis import config
from airis.adapt_link import (Envelope, JsonCodec, Listener, Sender, UdpSender, frame, read_frame,
                              run_udp_envelope_listener)
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


def _wait(pred, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_quiet_link_is_kept_and_a_reconnect_replaces_it():
    """Silence between bursts is normal: the link stays up (TCP keepalive finds a dead peer)."""
    codec = JsonCodec()
    sender = Sender(codec)
    got = []
    lst = Listener(0, codec, got.append, timeout_s=0.1, max_bytes=1 << 20, sender=sender, host="127.0.0.1")
    lst.start()
    first = FakeAdapt("127.0.0.1", lst.bound_port)
    second = None
    try:
        assert _wait(sender.connected)
        time.sleep(0.6)                                    # several silent recv timeouts
        assert sender.connected()
        first.send(Envelope("ping", {"n": 1}))             # still heard after the silence
        assert _wait(lambda: len(got) == 1)
        second = FakeAdapt("127.0.0.1", lst.bound_port)    # ADAPT reconnects; the old socket is half-open
        second.send(Envelope("ping", {"n": 2}))
        assert _wait(lambda: len(got) == 2)                # heard at once, not after the old link times out
        assert sender.send(Envelope("event_notification", {"detail": "x"}))
        assert second.wait_for("event_notification")       # replies go to the new connection
    finally:
        first.close(); lst.stop.set()
        if second is not None:
            second.close()


def test_timeout_mid_frame_loses_nothing():
    """A recv timeout in the middle of a frame keeps what was read; the frame completes on the next call."""
    a, b = socket.socketpair()
    b.settimeout(0.05)
    codec = JsonCodec()
    data = frame(codec.encode(Envelope("ping", {"n": 1}))) + frame(codec.encode(Envelope("ping", {"n": 2})))
    a.sendall(data[:15])
    try:
        read_frame(b, 1 << 20)
        assert False, "expected a timeout"
    except socket.timeout:
        pass
    a.sendall(data[15:])
    assert codec.decode(read_frame(b, 1 << 20)).fields["n"] == 1
    assert codec.decode(read_frame(b, 1 << 20)).fields["n"] == 2


def test_json_codec_roundtrips_bytes():
    codec = JsonCodec()
    env = codec.decode(codec.encode(Envelope("localization_update", {"burst_id": 1, "healpix_grid": b"\x00\x01\xff"})))
    assert env.fields["healpix_grid"] == b"\x00\x01\xff"


def test_udp_path_to_and_from_the_comms_server():
    """One envelope per datagram, no framing: commands in, downlink out."""
    codec = JsonCodec()
    got = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port_in = probe.getsockname()[1]
    probe.close()
    stop = threading.Event()
    threading.Thread(target=run_udp_envelope_listener, args=(port_in, codec, got.append, stop, "127.0.0.1"),
                     daemon=True).start()
    comms = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # stands in for the comms server
    comms.bind(("127.0.0.1", 0))
    comms.settimeout(2.0)
    try:
        cmd = codec.encode(Envelope("pointing_cmd", {"target_elevation": 30.0, "target_azimuth": 0.0}))
        assert _wait(lambda: comms.sendto(cmd, ("127.0.0.1", port_in)) and got, 3.0)
        assert got[0].kind == "pointing_cmd" and got[0].fields["target_azimuth"] == 0.0
        tx = UdpSender(codec, "127.0.0.1", comms.getsockname()[1])
        assert tx.send(Envelope("event_notification", {"detail": "hello"}))
        assert codec.decode(comms.recvfrom(65535)[0]).fields["detail"] == "hello"
        assert not tx.send(Envelope("image_data", {"image_data": bytes(70000)}))   # over one datagram
    finally:
        stop.set(); comms.close()
