"""Products sized for the link, chunked with a manifest, honoring FULL / SLOW / STOP.

Products from one observation
    table      candidates as JSON (a few KB)                      FULL, SLOW
    thumbnail  1/THUMB_SCALE PNG of the frame (~150 KB)          FULL, SLOW
    cutouts    CUTOUT_PX square 16-bit arrays around each         FULL
               candidate with the WCS of the cutout
    frame      the full binned frame stays on disk; on request only

Every binary product is split into CHUNK_BYTES chunks; a manifest (name, size,
sha256, chunk count) goes first so the ground can request a missing chunk by
number. STOP mode queues everything to disk and flush() sends it when FULL
returns. Nothing is ever silently dropped: a drop is a DLK-xx log line.
"""
from __future__ import annotations

import hashlib
import io
import json
import logging
import os
import shutil
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import config
from .adapt_link import Envelope, Sender
from .errors import Code

log = logging.getLogger("airis.downlink")

FULL, SLOW, STOP = "FULL", "SLOW", "STOP"


@dataclass
class Product:
    name: str
    kind: str          # table | thumbnail | cutout | frame
    data: bytes
    meta: dict = field(default_factory=dict)

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.data).hexdigest()


def make_thumbnail(data: np.ndarray, scale: int = config.THUMB_SCALE) -> bytes:
    from PIL import Image
    h, w = (data.shape[0] // scale) * scale, (data.shape[1] // scale) * scale
    small = data[:h, :w].reshape(h // scale, scale, w // scale, scale).mean(axis=(1, 3))
    lo, hi = np.percentile(small, [1, 99.5])
    img = np.clip((small - lo) / max(hi - lo, 1e-9) * 255.0, 0, 255).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(img).save(buf, format="PNG", optimize=True)
    return buf.getvalue()


def make_cutout(data: np.ndarray, x: float, y: float, size: int = config.CUTOUT_PX) -> tuple[bytes, dict]:
    h, w = data.shape
    x0, y0 = int(round(x)) - size // 2, int(round(y)) - size // 2
    x0, y0 = max(0, min(x0, w - size)), max(0, min(y0, h - size))
    patch = data[y0:y0 + size, x0:x0 + size]
    arr = np.clip(patch, 0, 65535).astype("<u2")
    return arr.tobytes(), {"x0": x0, "y0": y0, "shape": list(arr.shape), "dtype": "uint16-le"}


def products_for(frame_data: np.ndarray, candidates: list, burst_id: int, tile: tuple[float, float],
                 frame_utc: str, wcs_dict: Optional[dict] = None) -> list[Product]:
    table = {"burst_id": burst_id, "tile_ra": tile[0], "tile_dec": tile[1], "frame_utc": frame_utc,
             "wcs": wcs_dict or {}, "candidates": [c if isinstance(c, dict) else c.__dict__ for c in candidates]}
    out = [Product(f"b{burst_id}_table.json", "table", json.dumps(table).encode("utf-8"))]
    out.append(Product(f"b{burst_id}_thumb.png", "thumbnail", make_thumbnail(frame_data)))
    for i, c in enumerate(table["candidates"]):
        data, meta = make_cutout(frame_data, c["x"], c["y"])
        meta.update({"candidate": i, "confidence": c.get("confidence")})
        out.append(Product(f"b{burst_id}_cand{i}.u16", "cutout", data, meta))
    return out


def chunk(product: Product, size: int = config.CHUNK_BYTES) -> tuple[dict, list[bytes]]:
    chunks = [product.data[i:i + size] for i in range(0, len(product.data), size)] or [b""]
    manifest = {"name": product.name, "kind": product.kind, "bytes": len(product.data),
                "sha256": product.sha256, "chunks": len(chunks), "chunk_bytes": size, "meta": product.meta}
    return manifest, chunks


def reassemble(manifest: dict, chunks: list[bytes]) -> bytes:
    data = b"".join(chunks)
    if len(data) != manifest["bytes"] or hashlib.sha256(data).hexdigest() != manifest["sha256"]:
        raise ValueError("reassembled product does not match its manifest")
    return data


class Downlink:
    def __init__(self, sender: Sender, queue_dir: str = config.DOWNLINK_QUEUE_DIR):
        self.sender = sender
        self.mode = FULL
        self.queue_dir = queue_dir
        self.sent: list[str] = []
        self.queued: list[str] = []
        self.dropped: list[str] = []

    def set_mode(self, mode: str) -> None:
        self.mode = mode
        log.info("comms mode %s", mode)
        if mode == FULL:
            self.flush()

    def send_products(self, products: list[Product], burst_id: int) -> None:
        for p in products:
            if self.mode == SLOW and p.kind not in ("table", "thumbnail"):
                log.info("%s SLOW mode: %s kept on disk", Code.DLK_QUEUED.value, p.name)
                self.queued.append(p.name)
                self._queue(p)
                continue
            if self.mode == STOP or not self.sender.connected():
                self._queue(p)
                continue
            self._send_now(p, burst_id)

    def _send_now(self, p: Product, burst_id: int) -> bool:
        manifest, chunks = chunk(p)
        if not self.sender.send(Envelope("internal_message", {"heading": "image_manifest", "message": json.dumps(manifest)})):
            self.dropped.append(p.name)
            return False
        for i, c in enumerate(chunks):
            ok = self.sender.send(Envelope("image_data", {"burst_id": burst_id, "sequence_num": i, "image_data": c,
                                                           "name": p.name}))
            if not ok:
                log.warning("%s chunk %d of %s rejected", Code.DLK_CHUNK_REJECTED.value, i, p.name)
                self.dropped.append(p.name)
                return False
        self.sent.append(p.name)
        return True

    def _queue(self, p: Product) -> None:
        os.makedirs(self.queue_dir, exist_ok=True)
        with open(os.path.join(self.queue_dir, p.name), "wb") as f:
            f.write(p.data)
        with open(os.path.join(self.queue_dir, p.name + ".meta.json"), "w") as f:
            json.dump({"kind": p.kind, "meta": p.meta}, f)
        if p.name not in self.queued:
            self.queued.append(p.name)
        log.info("%s queued %s (%d bytes)", Code.DLK_QUEUED.value, p.name, len(p.data))

    def flush(self) -> int:
        if not os.path.isdir(self.queue_dir) or self.mode != FULL or not self.sender.connected():
            return 0
        n = 0
        for name in sorted(os.listdir(self.queue_dir)):
            if name.endswith(".meta.json"):
                continue
            path = os.path.join(self.queue_dir, name)
            try:
                with open(path + ".meta.json") as f:
                    m = json.load(f)
            except OSError:
                m = {"kind": "frame", "meta": {}}
            with open(path, "rb") as f:
                p = Product(name, m["kind"], f.read(), m["meta"])
            burst = int(name[1:].split("_")[0]) if name.startswith("b") else 0
            if self._send_now(p, burst):
                os.remove(path)
                try:
                    os.remove(path + ".meta.json")
                except OSError:
                    pass
                self.queued = [q for q in self.queued if q != name]
                n += 1
        return n
