"""Serial line protocol v5 between the bridge and the firmware."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

STATUS_IDLE = 0
STATUS_HOMING = (11, 12, 13)
STATUS_SLEWING = 15
ERROR_MIN = 50

STATUS_NAMES = {0: "IDLE", 11: "HOMING_M1", 12: "HOMING_M2", 13: "HOMING_COMPLETE", 15: "SLEWING"}


def checksum(payload: str) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection (check value of '123456789' is 0x29B1)."""
    crc = 0xFFFF
    for b in payload.encode("ascii"):
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(cmd: str, seq: int) -> bytes:
    """``cmd`` is the command text without seq/checksum, e.g. ``MOVE 60.000 0.000``."""
    if not (0 <= seq <= 255):
        raise ValueError("seq out of range")
    body = f"{cmd} #{seq}"
    return f"{body}*{checksum(body):04X}\n".encode("ascii")


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
    if len(cs_text) != 4 or any(c not in "0123456789abcdefABCDEF" for c in cs_text):
        raise ProtocolError("bad checksum field")
    try:
        ok = int(cs_text, 16) == checksum(body)
    except UnicodeEncodeError:
        ok = False
    if not ok:
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
        if head == "DONE" and len(parts) >= 3:      # a DONE without both residuals is damaged
            return FwLine("DONE", None, (float(parts[1]), float(parts[2])), raw=line)
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
