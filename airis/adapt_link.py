"""ADAPT link and comms-server link: framing, codecs, listeners, senders, dispatch."""
from __future__ import annotations

import hashlib
import json
import logging
import socket
import struct
import threading
import weakref
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Optional

from . import config
from .errors import Code

log = logging.getLogger("airis.adapt")

MAX_UDP_PAYLOAD = 65507


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
        d = json.loads(payload.decode("utf-8"), object_hook=_json_bytes_hook)
        kind = d.pop("kind")
        return Envelope(kind, d, d.pop("sender_id", "adapt"), d.pop("dest_id", "controls"))


def _json_default(o):
    if isinstance(o, (bytes, bytearray)):
        return {"__bytes__": o.hex()}
    raise TypeError(str(o))


def _json_bytes_hook(d: dict):
    if len(d) == 1 and "__bytes__" in d:
        return bytes.fromhex(d["__bytes__"])
    return d


def _is_repeated(fd) -> bool:
    rep = getattr(fd, "is_repeated", None)        # newer protobuf; .label is deprecated there
    return bool(rep) if rep is not None else fd.label == fd.LABEL_REPEATED


class ProtoCodec:  # pragma: no cover - needs the flight protobufs
    """messages.transport.v1.MessageEnvelope <-> Envelope."""

    def __init__(self, pb=None):
        if pb is None:
            from messages.transport.v1 import transport_pb2 as pb
        self.pb = pb
        self._warned: set = set()

    def decode(self, payload: bytes) -> Envelope:
        env = self.pb.MessageEnvelope()
        env.ParseFromString(payload)
        kind = env.WhichOneof("message_body")
        fields = {}
        if kind:
            body = getattr(env, kind)
            for fd in body.DESCRIPTOR.fields:
                if fd.message_type is not None:
                    if fd.message_type.full_name == "google.protobuf.Timestamp" and not _is_repeated(fd) \
                            and body.HasField(fd.name):
                        ts = getattr(body, fd.name)
                        fields[fd.name] = ts.seconds + ts.nanos * 1e-9
                    continue                              # no other nested message is used by controls
                value = getattr(body, fd.name)
                if _is_repeated(fd):
                    fields[fd.name] = list(value)
                elif fd.enum_type is not None:
                    ev = fd.enum_type.values_by_number.get(value)
                    fields[fd.name] = ev.name if ev is not None else value
                else:
                    fields[fd.name] = value
        return Envelope(kind or "unknown", fields,
                        getattr(env, "sender_id", "") or "adapt", getattr(env, "dest_id", "") or "controls")

    def encode(self, env: Envelope) -> bytes:
        pb = self.pb.MessageEnvelope()
        top = pb.DESCRIPTOR.fields_by_name
        if "sender_id" in top:
            pb.sender_id = env.sender_id
        if "dest_id" in top:
            pb.dest_id = env.dest_id
        if env.kind not in top or top[env.kind].message_type is None:
            raise ValueError(f"MessageEnvelope has no body {env.kind!r}")
        body = getattr(pb, env.kind)
        body.SetInParent()                                # select the oneof even with no fields set
        by_name = body.DESCRIPTOR.fields_by_name
        for k, v in env.fields.items():
            fd = by_name.get(k)
            if fd is None:
                if (env.kind, k) not in self._warned:
                    self._warned.add((env.kind, k))
                    log.warning("%s has no field %r; not sent", env.kind, k)
                continue
            if fd.message_type is not None:
                if fd.message_type.full_name != "google.protobuf.Timestamp":
                    continue
                ts = getattr(body, k)
                if isinstance(v, datetime):
                    ts.FromDatetime(v)
                elif isinstance(v, (int, float)):
                    ts.seconds, ts.nanos = int(v), int((v % 1) * 1e9)
                else:
                    ts.GetCurrentTime()
            elif _is_repeated(fd):
                getattr(body, k).extend(v)
            elif fd.enum_type is not None and isinstance(v, str):
                setattr(body, k, fd.enum_type.values_by_name[v].number)
            else:
                setattr(body, k, v)
        return pb.SerializeToString()


# ---- Framing ------------------------------------------------------------------

def frame(payload: bytes) -> bytes:
    return config.ADAPT_SYNC_WORD + struct.pack(">I", len(payload)) + payload


class FrameReader:
    """Per-connection frame reader."""

    def __init__(self, max_bytes: int):
        self.max_bytes = max_bytes
        self.buf = bytearray()
        self.skipped = 0

    def read(self, conn: socket.socket) -> Optional[bytes]:
        """Next payload, resynchronizing on the sync word; None on EOF. socket.timeout propagates."""
        while True:
            payload = self._take()
            if payload is not None:
                return payload
            chunk = conn.recv(65536)
            if not chunk:
                return None
            self.buf += chunk

    def _take(self) -> Optional[bytes]:
        sync = config.ADAPT_SYNC_WORD
        head = len(sync) + 4
        while True:
            i = self.buf.find(sync)
            if i < 0:                                     # no sync yet: keep a tail that could start one
                drop = max(0, len(self.buf) - (len(sync) - 1))
                self.skipped += drop
                del self.buf[:drop]
                return None
            if i:
                self.skipped += i
                del self.buf[:i]
            if len(self.buf) < head:
                return None
            (length,) = struct.unpack(">I", self.buf[len(sync):head])
            if not (0 < length <= self.max_bytes):        # a false sync word: slide past it
                self.skipped += 1
                del self.buf[:1]
                continue
            if len(self.buf) < head + length:
                return None
            payload = bytes(self.buf[head:head + length])
            del self.buf[:head + length]
            if self.skipped:
                log.warning("resynchronized after skipping %d bytes", self.skipped)
                self.skipped = 0
            return payload


_readers: "weakref.WeakKeyDictionary[socket.socket, FrameReader]" = weakref.WeakKeyDictionary()


def read_frame(conn: socket.socket, max_bytes: int) -> Optional[bytes]:
    """Function form of :class:`FrameReader`; the reader is kept per socket."""
    reader = _readers.get(conn)
    if reader is None:
        reader = _readers[conn] = FrameReader(max_bytes)
    return reader.read(conn)


# ---- Senders ---------------------------------------------------------------------

class Sender:
    """Thread-safe outbound side on a framed TCP connection handed over by a Listener."""

    def __init__(self, codec):
        self.codec = codec
        self._conn: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self.sent = 0
        self.on_attach: Optional[Callable[[], None]] = None   # called when a connection appears

    def attach(self, conn: Optional[socket.socket]) -> None:
        with self._lock:
            self._conn = conn
        if conn is not None and self.on_attach is not None:
            self.on_attach()

    def detach(self, conn: socket.socket) -> None:
        """Forget ``conn`` if it is still the current one (a newer connection is left alone)."""
        with self._lock:
            if self._conn is conn:
                self._conn = None

    def connected(self) -> bool:
        with self._lock:
            return self._conn is not None

    def send(self, env: Envelope) -> bool:
        try:
            data = frame(self.codec.encode(env))
        except Exception as e:                           # a codec error must not kill the caller's thread
            log.error("%s cannot encode %s: %s", Code.DLK_CHUNK_REJECTED.value, env.kind, e)
            return False
        with self._lock:
            conn = self._conn
            if conn is None:
                log.warning("%s no connection; dropped %s", Code.DLK_NO_SENDER.value, env.kind)
                return False
            try:
                conn.sendall(data)
                self.sent += 1
                return True
            except OSError as e:
                log.warning("%s send failed: %s", Code.DLK_NO_SENDER.value, e)
                self._conn = None
                try:                                      # a failed sendall may have left half a frame on the
                    conn.shutdown(socket.SHUT_RDWR)       # wire: end the connection so both sides start clean
                except OSError:
                    pass
                return False


class UdpSender:
    """Downlink through the airis-comms server: one MessageEnvelope per datagram."""

    def __init__(self, codec, host: str = config.COMMS_HOST, port: int = config.COMMS_PORT_DOWNLINK):
        self.codec, self.addr = codec, (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self.sent = 0
        self.on_attach: Optional[Callable[[], None]] = None   # unused: there is no connection event

    def connected(self) -> bool:
        return True

    def send(self, env: Envelope) -> bool:
        try:
            data = self.codec.encode(env)
        except Exception as e:
            log.error("%s cannot encode %s: %s", Code.DLK_CHUNK_REJECTED.value, env.kind, e)
            return False
        if len(data) > MAX_UDP_PAYLOAD:
            log.error("%s %s is %d bytes, over the UDP limit", Code.DLK_CHUNK_REJECTED.value, env.kind, len(data))
            return False
        try:
            with self._lock:
                self.sock.sendto(data, self.addr)
                self.sent += 1
            return True
        except OSError as e:
            log.warning("%s send failed: %s", Code.DLK_NO_SENDER.value, e)
            return False


# ---- Listeners -------------------------------------------------------------------

def _set_keepalive(conn: socket.socket) -> None:
    """Bare SO_KEEPALIVE waits about two hours before the first probe; set real timers where the OS has them."""
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for name, value in (("TCP_KEEPIDLE", config.ADAPT_KEEPALIVE_IDLE_S),
                        ("TCP_KEEPINTVL", config.ADAPT_KEEPALIVE_INTVL_S),
                        ("TCP_KEEPCNT", config.ADAPT_KEEPALIVE_CNT)):
        opt = getattr(socket, name, None)
        if opt is not None:
            try:
                conn.setsockopt(socket.IPPROTO_TCP, opt, value)
            except OSError:
                pass


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
        self._conn: Optional[socket.socket] = None
        self._conn_lock = threading.Lock()
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
            with self._conn_lock:
                old, self._conn = self._conn, conn
            if old is not None:
                log.warning("ADAPT port %d: new connection from %s replaces the previous one", self.port, addr)
                try:
                    old.shutdown(socket.SHUT_RDWR)        # its _serve thread sees EOF and exits
                except OSError:
                    pass
            threading.Thread(target=self._serve, args=(conn, addr), daemon=True,
                             name=f"adapt-{self.port}-conn").start()
        self.srv.close()
        with self._conn_lock:
            conn = self._conn
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _serve(self, conn: socket.socket, addr) -> None:
        reader = FrameReader(self.max_bytes)
        try:
            _set_keepalive(conn)
            conn.settimeout(self.timeout_s)
            if self.sender is not None and not self.nav_port:
                self.sender.attach(conn)
            while not self.stop.is_set():
                try:
                    payload = reader.read(conn)
                except socket.timeout:
                    if self.nav_port:
                        log.warning("%s no NavigationUpdate for %.0fs", Code.NAV_NO_FIX.value, self.timeout_s)
                    continue                              # quiet is not dead; keepalive finds a dead peer
                except OSError as e:
                    log.warning("ADAPT socket error on %d: %s", self.port, e)
                    return
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
                self.sender.detach(conn)
            with self._conn_lock:
                if self._conn is conn:
                    self._conn = None
            try:
                conn.close()
            except OSError:
                pass


def run_udp_envelope_listener(port: int, codec, dispatch: Callable[[Envelope], None],
                              stop: Optional[threading.Event] = None, host: str = config.COMMS_HOST) -> None:
    """Commands relayed by the airis-comms server: one MessageEnvelope per datagram. Re-binds after a socket error."""
    import time
    while stop is None or not stop.is_set():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(1.0)
            sock.bind((host, port))
            while stop is None or not stop.is_set():
                try:
                    data, _ = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                try:
                    env = codec.decode(data)
                except Exception:
                    log.exception("undecodable datagram on %d; skipping", port)
                    continue
                try:
                    dispatch(env)
                except Exception:
                    log.exception("handler failed for %s", env.kind)
        except OSError as e:
            log.warning("comms command socket error on %d: %s; rebinding", port, e)
            time.sleep(2.0)
        finally:
            sock.close()


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()
