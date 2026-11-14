"""Downlink: products sized for the link, chunk/manifest integrity, comms modes with a disk queue."""
import json
import os

import numpy as np

from airis.adapt_link import JsonCodec, Sender
from airis.downlink import FULL, SLOW, STOP, Downlink, Product, chunk, products_for, reassemble


class RecordingSender(Sender):
    def __init__(self):
        super().__init__(JsonCodec())
        self.envelopes = []
        self.up = True

    def connected(self):
        return self.up

    def send(self, env):
        if not self.up:
            return False
        self.envelopes.append(env)
        return True


def manifest_of(env) -> dict:
    head, _, body = env.fields["detail"].partition(" ")
    assert head == "image_manifest"
    return json.loads(body)


def frame_and_cands():
    rng = np.random.default_rng(1)
    data = rng.normal(100, 5, (480, 720)) + 0
    cands = [{"x": 100.0, "y": 50.0, "confidence": 0.95}, {"x": 700.0, "y": 470.0, "confidence": 0.5}]
    return data, cands


def test_products_are_link_sized():
    data, cands = frame_and_cands()
    prods = products_for(data, cands, 42, (1.0, 2.0), "2026-12-15T00:00:00Z")
    kinds = [p.kind for p in prods]
    assert kinds == ["table", "thumbnail", "cutout", "cutout"]
    assert len(prods[1].data) < 60_000                      # thumbnail of a 480x720 frame
    assert len(prods[2].data) == 128 * 128 * 2               # 16-bit cutout
    assert json.loads(prods[0].data)["candidates"][0]["confidence"] == 0.95
    assert prods[3].meta["x0"] == 720 - 128                  # clamped to the frame edge


def test_chunks_reassemble_and_detect_corruption():
    p = Product("x.bin", "cutout", os.urandom(200_000))
    manifest, chunks = chunk(p, 65536)
    assert manifest["chunks"] == 4 and manifest["bytes"] == 200_000
    assert reassemble(manifest, chunks) == p.data
    bad = chunks[:]
    bad[1] = bad[1][:-1] + bytes([bad[1][-1] ^ 1])
    try:
        reassemble(manifest, bad)
        assert False, "corruption not detected"
    except ValueError:
        pass


def test_modes_full_slow_stop(tmp_path):
    data, cands = frame_and_cands()
    prods = products_for(data, cands, 7, (0, 0), "t")
    s = RecordingSender()
    dl = Downlink(s, queue_dir=str(tmp_path / "q"))
    dl.send_products(prods, 7)
    kinds = [e.kind for e in s.envelopes]
    assert kinds.count("event_notification") == 4 and "image_data" in kinds
    assert dl.sent == [p.name for p in prods]

    s.envelopes.clear()
    dl.set_mode(SLOW)
    dl.send_products(prods, 7)
    names = {manifest_of(e)["name"] for e in s.envelopes if e.kind == "event_notification"}
    assert names == {prods[0].name, prods[1].name}          # table + thumbnail only
    assert any(n.startswith("b7_") and "_cand" in n for n in dl.queued)

    s.envelopes.clear()
    dl.set_mode(STOP)
    dl.send_products(prods, 7)
    assert not s.envelopes and len(os.listdir(tmp_path / "q")) >= 4

    dl.set_mode(FULL)                                        # flush on return to FULL
    assert not [f for f in os.listdir(tmp_path / "q") if not f.endswith(".meta.json")]
    assert len([e for e in s.envelopes if e.kind == "event_notification"]) >= 4


def test_frames_of_one_burst_do_not_share_names():
    data, cands = frame_and_cands()
    a = {p.name for p in products_for(data, cands, 42, (1.0, 2.0), "2026-12-15T00:00:00+00:00")}
    b = {p.name for p in products_for(data, cands, 42, (1.0, 2.0), "2026-12-15T00:00:08+00:00")}
    assert not a & b


def test_image_chunks_fit_one_datagram_and_carry_only_proto_fields():
    s = RecordingSender()
    dl = Downlink(s)
    dl._send_now(Product("x.bin", "cutout", os.urandom(200_000)), 9)
    chunks = [e for e in s.envelopes if e.kind == "image_data"]
    assert [e.fields["sequence_num"] for e in chunks] == [0, 1, 2, 3]
    assert all(len(e.fields["image_data"]) <= 60000 for e in chunks)
    assert all(set(e.fields) == {"timestamp", "burst_id", "sequence_num", "image_data"} for e in chunks)


def test_failed_send_is_queued_and_a_reconnect_flushes_it(tmp_path):
    data, cands = frame_and_cands()
    prods = products_for(data, cands, 3, (0, 0), "t")

    class FailingSender(RecordingSender):
        def send(self, env):
            return False if env.kind == "image_data" else super().send(env)

    s = FailingSender()
    dl = Downlink(s, queue_dir=str(tmp_path / "q"))
    dl.send_products(prods, 3)                               # manifests go, chunks fail
    assert set(dl.queued) == {p.name for p in prods}
    s.__class__ = RecordingSender                            # the link works again...
    s.envelopes.clear()
    dl.flush()                                               # ...and a reconnect (Sender.attach) calls this
    assert not [f for f in os.listdir(tmp_path / "q") if not f.endswith(".meta.json")]


def test_unknown_mode_is_ignored():
    dl = Downlink(RecordingSender())
    dl.set_mode("BANANA")
    assert dl.mode == FULL


def test_no_connection_queues_instead_of_dropping(tmp_path):
    data, cands = frame_and_cands()
    s = RecordingSender(); s.up = False
    dl = Downlink(s, queue_dir=str(tmp_path / "q"))
    dl.send_products(products_for(data, cands, 1, (0, 0), "t"), 1)
    assert dl.queued and not dl.dropped
    s.up = True
    assert dl.flush() == 4
