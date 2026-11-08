"""ADAPT link: framing, listener, sender, dispatch. The only file that knows the wire format.

Framing (ratified 2026-07-16): 8-byte sync word, uint32 big-endian length,
payload. The reader hunts for the sync word after corruption instead of
dropping the link. Two improvements over v1: the 5005 link is dropped after
ADAPT_DEAD_AFTER_TIMEOUTS consecutive silent periods (a half-open TCP
connection was never detected before), SO_KEEPALIVE is set, and the
connection is kept so AIRIS can SEND on it.

Payload codec is pluggable: ``ProtoCodec`` wraps messages.transport.v1
(flight); ``JsonCodec`` is used by the simulators and tests. Both produce an
``Envelope(kind, fields)`` where ``kind`` is the oneof body name.
"""
from __future__ import annotations

import hashlib
import json
import logging
import socket
import struct
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import config
from .errors import Code

log = logging.getLogger("airis.adapt")


@dataclass
class Envelope:
    kind: str
    fields: dict = field(default_factory=dict)
    sender_id: str = "controls"
    dest_id: str = "server"


class JsonCodec:
    def encode(self, env: Envelope) -> bytes:
        d = {"kind": env.kind, "sender_id": env.sender_id, "dest_id": env.dest_id}
        d.update(env.fields)
        return json.dumps(d, default=_json_default).encode("utf-8")

    def decode(self, payload: bytes) -> Envelope:
        d = json.loads(payload.decode("utf-8"))
        kind = d.pop("kind")
        return Envelope(kind, d, d.pop("sender_id", "adapt"), d.pop("dest_id", "controls"))


def _json_default(o):
    if isinstance(o, (bytes, bytearray)):
        return {"__bytes__": o.hex()}
    raise TypeError(str(o))


class ProtoCodec:  # pragma: no cover - needs the flight protobufs
    """messages.transport.v1.MessageEnvelope <-> Envelope. Imported lazily."""

    def __init__(self):
        from messages.transport.v1 import transport_pb2
        self.pb = transport_pb2

    def decode(self, payload: bytes) -> Envelope:
        env = self.pb.MessageEnvelope()
        env.ParseFromString(payload)
        kind = env.WhichOneof("message_body")
        body = getattr(env, kind) if kind else None
        fields = {}
        if body is not None:
            for f, v in body.ListFields():
                fields[f.name] = v
        return Envelope(kind or "unknown", fields, env.sender_id, env.dest_id)

    def encode(self, env: Envelope) -> bytes:
        pb = self.pb.MessageEnvelope()
        pb.sender_id, pb.dest_id = env.sender_id, env.dest_id
        body = getattr(pb, env.kind)
        for k, v in env.fields.items():
            if k == "timestamp":
                getattr(body, k).GetCurrentTime()
            else:
                setattr(body, k, v)
        return pb.SerializeToString()


# ---- Framing ------------------------------------------------------------------

def frame(payload: bytes) -> bytes:
    return config.ADAPT_SYNC_WORD + struct.pack(">I", len(payload)) + payload


def _recv_exact(conn: socket.socket, n: int) -> Optional[bytes]:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def read_frame(conn: socket.socket, max_bytes: int) -> Optional[bytes]:
    """Return the next payload, resynchronizing on the sync word; None on EOF."""
    sync = config.ADAPT_SYNC_WORD
    window = _recv_exact(conn, len(sync))
    if window is None:
        return None
    window = bytearray(window)
    skipped = 0
    while True:
        if bytes(window) == sync:
            header = _recv_exact(conn, 4)
            if header is None:
                return None
            (length,) = struct.unpack(">I", header)
            if 0 < length <= max_bytes:
                if skipped:
                    log.warning("resynchronized after skipping %d bytes", skipped)
                return _recv_exact(conn, length)
            tail = _recv_exact(conn, 4)
            if tail is None:
                return None
            window = bytearray(header) + bytearray(tail)
            skipped += len(sync)
            continue
        b = conn.recv(1)
        if not b:
            return None
        del window[0]
        window += b
        skipped += 1


# ---- Sender ---------------------------------------------------------------------

class Sender:
    """Thread-safe outbound side. Holds whatever connection the listener gave it."""

    def __init__(self, codec):
        self.codec = codec
        self._conn: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self.sent = 0

    def attach(self, conn: Optional[socket.socket]) -> None:
        with self._lock:
            self._conn = conn

    def connected(self) -> bool:
        with self._lock:
            return self._conn is not None

    def send(self, env: Envelope) -> bool:
        with self._lock:
            conn = self._conn
            if conn is None:
                log.warning("%s no connection; dropped %s", Code.DLK_NO_SENDER.value, env.kind)
                return False
            try:
                conn.sendall(frame(self.codec.encode(env)))
                self.sent += 1
                return True
            except OSError as e:
                log.warning("%s send failed: %s", Code.DLK_NO_SENDER.value, e)
                self._conn = None
                return False


# ---- Listener -------------------------------------------------------------------

class Listener(threading.Thread):
    """Accept loop for one ADAPT port; every envelope goes to ``dispatch(env)``."""

    def __init__(self, port: int, codec, dispatch: Callable[[Envelope], None], *,
                 timeout_s: float, max_bytes: int, nav_port: bool = False,
                 sender: Optional[Sender] = None, host: str = config.ADAPT_HOST):
        super().__init__(daemon=True, name=f"adapt-{port}")
        self.port, self.codec, self.dispatch = port, codec, dispatch
        self.timeout_s, self.max_bytes, self.nav_port = timeout_s, max_bytes, nav_port
        self.sender, self.host = sender, host
        self.stop = threading.Event()
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(2)
        self.srv.settimeout(0.5)
        self.bound_port = self.srv.getsockname()[1]

    def run(self) -> None:
        while not self.stop.is_set():
            try:
                conn, addr = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                self._serve(conn, addr)
        self.srv.close()

    def _serve(self, conn: socket.socket, addr) -> None:
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        conn.settimeout(self.timeout_s)
        if self.sender is not None and not self.nav_port:
            self.sender.attach(conn)
        timeouts = 0
        try:
            while not self.stop.is_set():
                try:
                    payload = read_frame(conn, self.max_bytes)
                except socket.timeout:
                    timeouts += 1
                    if self.nav_port:
                        log.warning("%s no NavigationUpdate for %.0fs", Code.NAV_NO_FIX.value, self.timeout_s)
                    if timeouts >= config.ADAPT_DEAD_AFTER_TIMEOUTS:
                        log.warning("ADAPT port %d silent for %d periods; dropping link", self.port, timeouts)
                        return
                    continue
                except OSError as e:
                    log.warning("ADAPT socket error on %d: %s", self.port, e)
                    return
                timeouts = 0
                if payload is None:
                    return
                try:
                    env = self.codec.decode(payload)
                except Exception:
                    log.exception("undecodable envelope on %d; skipping", self.port)
                    continue
                try:
                    self.dispatch(env)
                except Exception:
                    log.exception("handler failed for %s", env.kind)
        finally:
            if self.sender is not None and not self.nav_port:
                self.sender.attach(None)


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()
