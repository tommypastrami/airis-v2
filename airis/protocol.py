"""Serial line protocol v5 between the bridge and the firmware.

Host -> firmware, one ASCII line per command::

    MOVE 60.000 0.000 #17*5A\\n
    ^--- command text ----^ ^seq ^checksum

* ``#<seq>``: 0..255, wraps. The firmware remembers the last executed seq and
  answers a repeat with ``ACK <seq>`` without executing it again, so a resend
  after a lost ACK can never cause a ghost move.
* ``*<cs>``: two hex digits, XOR of every byte from the first character of the
  command up to and including the ``#<seq>`` field (NMEA style). A line whose
  checksum does not match is answered ``NAK <seq> checksum`` and NOT executed.
  Byte loss can no longer turn ``MOVE 60.000`` into a valid ``MOVE 6.000``.

Firmware -> host, one line each, all parseable by :func:`parse_fw_line`::

    ACK 17                      command accepted (not yet finished)
    NAK 17 checksum             rejected; host resends the same seq
    STATUS 15 SLEWING           pushed on every state change and on request
    ERR 63 busy                 any code >= 50; distinct codes (see errors.py)
    DONE 0.13 -0.12             move finished; residual el/az in degrees
    POS 59.87 0.12              position (1 Hz heartbeat while idle)
    ZERO SET                    SETZERO applied
    SUN OK -12.0 140.0          live sun center accepted
    RX: <line>                  human echo, ignored by the parser

No sync bytes: the firmware drops non-printable bytes at ingest, and the
checksum makes a damaged line detectable.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

STATUS_IDLE = 0
STATUS_HOMING = (11, 12, 13)
STATUS_SLEWING = 15
ERROR_MIN = 50

STATUS_NAMES = {0: "IDLE", 11: "HOMING_M1", 12: "HOMING_M2", 13: "HOMING_COMPLETE", 15: "SLEWING"}


def checksum(payload: str) -> int:
    cs = 0
    for b in payload.encode("ascii"):
        cs ^= b
    return cs


def encode(cmd: str, seq: int) -> bytes:
    """``cmd`` is the command text without seq/checksum, e.g. ``MOVE 60.000 0.000``."""
    if not (0 <= seq <= 255):
        raise ValueError("seq out of range")
    body = f"{cmd} #{seq}"
    return f"{body}*{checksum(body):02X}\n".encode("ascii")


@dataclass
class HostLine:
    cmd: str          # e.g. "MOVE"
    args: list[str]   # e.g. ["60.000", "0.000"]
    seq: int
    raw: str


class ProtocolError(ValueError):
    pass


def parse_host_line(line: str) -> HostLine:
    """Firmware-side parser (also used by the fake Arduino). Raises on any damage."""
    line = line.strip()
    if "*" not in line or "#" not in line:
        raise ProtocolError("missing seq or checksum")
    body, cs_text = line.rsplit("*", 1)
    try:
        cs = int(cs_text, 16)
    except ValueError:
        raise ProtocolError("bad checksum field")
    if len(cs_text) != 2 or cs != checksum(body):
        raise ProtocolError("checksum")
    text, seq_text = body.rsplit("#", 1)
    try:
        seq = int(seq_text)
    except ValueError:
        raise ProtocolError("bad seq")
    if not (0 <= seq <= 255):
        raise ProtocolError("seq range")
    parts = text.split()
    if not parts:
        raise ProtocolError("empty command")
    return HostLine(parts[0].upper(), parts[1:], seq, line)


@dataclass
class FwLine:
    kind: str                 # ACK NAK STATUS ERR DONE POS ZERO SUN RX OTHER
    code: Optional[int] = None
    args: tuple = ()
    raw: str = ""


def parse_fw_line(line: str) -> FwLine:
    """Host-side parser. Never raises; unknown lines come back as kind='OTHER'."""
    line = line.strip()
    parts = line.split()
    if not parts:
        return FwLine("OTHER", raw=line)
    head = parts[0]
    try:
        if head == "ACK" and len(parts) >= 2:
            return FwLine("ACK", int(parts[1]), raw=line)
        if head == "NAK" and len(parts) >= 2:
            return FwLine("NAK", int(parts[1]), tuple(parts[2:]), raw=line)
        if head == "STATUS" and len(parts) >= 2:
            return FwLine("STATUS", int(parts[1]), tuple(parts[2:]), raw=line)
        if head == "ERR" and len(parts) >= 2:
            return FwLine("ERR", int(parts[1]), tuple(parts[2:]), raw=line)
        if head == "DONE":
            vals = tuple(float(p) for p in parts[1:3]) if len(parts) >= 3 else ()
            return FwLine("DONE", None, vals, raw=line)
        if head == "POS" and len(parts) >= 3:
            return FwLine("POS", None, (float(parts[1]), float(parts[2])), raw=line)
        if head == "ZERO" and len(parts) >= 2 and parts[1] == "SET":
            return FwLine("ZERO", raw=line)
        if head == "SUN" and len(parts) >= 2 and parts[1] == "OK":
            return FwLine("SUN", None, tuple(float(p) for p in parts[2:4]), raw=line)
        if head == "RX:":
            return FwLine("RX", None, (line[4:],), raw=line)
    except ValueError:
        pass
    return FwLine("OTHER", raw=line)


def fmt_angle(x: float) -> str:
    return f"{x:.3f}"
