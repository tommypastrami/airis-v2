"""Trimble BX992 NMEA assembly, ADAPT fallback, and the observer position.

Keeps three things and nothing else: the latest trustworthy fix (GGA), the
latest heading (HDT) and the UTC date (RMC/ZDA). Everything is parsed here with
explicit types, so a quality field that arrives as the string "4" is still a 4.

Authority: the Trimble is authoritative for position while its fix is younger
than TRIMBLE_STALE_S; otherwise the latest ADAPT navigation update is used;
before either, the configured defaults. ADAPT always owns flight_phase.
"""
from __future__ import annotations

import logging
import socket
import threading
import time
from dataclasses import dataclass, field
from datetime import date, datetime, time as dtime, timedelta, timezone
from typing import Callable, Optional

from . import config
from .errors import Code
from .pointing import Site

log = logging.getLogger("airis.nav")


# ---- NMEA parsing (only the four sentences we use) ----------------------------

def nmea_checksum_ok(sentence: str) -> bool:
    if not sentence.startswith("$"):
        return False
    body, _, cs = sentence[1:].partition("*")
    if not cs:
        return True  # some receivers omit it; accept
    x = 0
    for b in body.encode("ascii", "ignore"):
        x ^= b
    try:
        return x == int(cs[:2], 16)
    except ValueError:
        return False


def _ddmm_to_deg(val: str, hemi: str) -> Optional[float]:
    if not val:
        return None
    try:
        v = float(val)
    except ValueError:
        return None
    deg = int(v // 100)
    minutes = v - deg * 100
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out


def _tod(val: str) -> Optional[dtime]:
    try:
        hh, mm, ss = int(val[0:2]), int(val[2:4]), float(val[4:])
        return dtime(hh, mm, int(ss), int((ss % 1) * 1e6))
    except (ValueError, IndexError):
        return None


@dataclass
class Sentence:
    kind: str                # GGA HDT RMC ZDA OTHER
    fields: dict


def parse_sentence(line: str) -> Optional[Sentence]:
    line = line.strip()
    if not line.startswith("$") or not nmea_checksum_ok(line):
        return None
    body = line[1:].split("*", 1)[0]
    parts = body.split(",")
    kind = parts[0][-3:]
    try:
        if kind == "GGA":
            return Sentence("GGA", {
                "tod": _tod(parts[1]),
                "lat": _ddmm_to_deg(parts[2], parts[3]),
                "lon": _ddmm_to_deg(parts[4], parts[5]),
                "quality": int(parts[6]) if parts[6] else 0,
                "num_sats": int(parts[7]) if parts[7] else 0,
                "alt_msl_m": float(parts[9]) if parts[9] else None,
                "geoid_sep_m": float(parts[11]) if parts[11] else 0.0,
            })
        if kind == "HDT":
            return Sentence("HDT", {"heading_deg": float(parts[1]) if parts[1] else None})
        if kind == "RMC":
            d = parts[9]
            return Sentence("RMC", {"date": date(2000 + int(d[4:6]), int(d[2:4]), int(d[0:2])) if len(d) == 6 else None})
        if kind == "ZDA":
            return Sentence("ZDA", {"date": date(int(parts[4]), int(parts[3]), int(parts[2])) if parts[4] else None})
    except (ValueError, IndexError):
        return None
    return Sentence("OTHER", {})


# ---- State ----------------------------------------------------------------------

@dataclass
class Fix:
    lat: float
    lon: float
    alt_ellips_m: float
    quality: int
    num_sats: int
    utc: Optional[datetime]
    t_mono: float


@dataclass
class AdaptNav:
    lat: float
    lon: float
    alt_m: float
    flight_phase: int
    t_mono: float


class NavState:
    """Thread-safe holder of the latest fix/heading/date, with a throttled push callback."""

    def __init__(self, push: Optional[Callable[[Site, Optional[int]], None]] = None,
                 clock: Callable[[], float] = time.monotonic):
        self._lock = threading.Lock()
        self._clock = clock
        self._push = push
        self.fix: Optional[Fix] = None
        self.heading: Optional[tuple[float, float]] = None   # (deg, t_mono)
        self.utc_date: Optional[date] = None
        self.adapt: Optional[AdaptNav] = None
        self._last_push = -1e9
        self.rejected = 0

    # -- ingest
    def handle(self, s: Sentence) -> None:
        now = self._clock()
        f = s.fields
        if s.kind == "RMC" or s.kind == "ZDA":
            if f.get("date"):
                with self._lock:
                    self.utc_date = f["date"]
        elif s.kind == "HDT":
            if f.get("heading_deg") is not None:
                with self._lock:
                    self.heading = (f["heading_deg"] % 360.0, now)
        elif s.kind == "GGA":
            if f["quality"] not in config.ACCEPTED_GGA_QUALITIES or f["lat"] is None or f["lon"] is None:
                self.rejected += 1
                return
            alt = f["alt_msl_m"]
            if alt is None:
                alt = 0.0  # 2-D fix: keep lat/lon, altitude unknown -> 0 (logged)
                log.debug("%s GGA without altitude; using 0", Code.NAV_BAD_SENTENCE.value)
            with self._lock:
                utc = None
                if self.utc_date and f["tod"]:
                    utc = datetime.combine(self.utc_date, f["tod"]).replace(tzinfo=timezone.utc)
                self.fix = Fix(f["lat"], f["lon"], alt + (f["geoid_sep_m"] or 0.0),
                               f["quality"], f["num_sats"], utc, now)
            self._maybe_push()

    def on_adapt(self, lat: float, lon: float, alt_m: float, flight_phase: int) -> None:
        with self._lock:
            self.adapt = AdaptNav(lat, lon, alt_m, flight_phase, self._clock())

    # -- queries
    def trimble_fresh(self) -> bool:
        with self._lock:
            return self.fix is not None and self._clock() - self.fix.t_mono <= config.TRIMBLE_STALE_S

    def observer(self) -> tuple[Site, Code]:
        """Best available site and the code saying which source it is."""
        with self._lock:
            fix, adapt, now = self.fix, self.adapt, self._clock()
        if fix is not None and now - fix.t_mono <= config.TRIMBLE_STALE_S:
            return Site(fix.lat, fix.lon, fix.alt_ellips_m), Code.NAV_TRIMBLE
        if adapt is not None and not (adapt.lat == 0.0 and adapt.lon == 0.0):
            return Site(adapt.lat, adapt.lon, adapt.alt_m), Code.NAV_STALE_USING_ADAPT
        return Site(), Code.NAV_USING_DEFAULTS

    def heading_deg(self) -> Optional[float]:
        with self._lock:
            h = self.heading
        if h is None or self._clock() - h[1] > config.HEADING_STALE_S:
            return None
        return h[0]

    def flight_phase(self) -> Optional[int]:
        with self._lock:
            return self.adapt.flight_phase if self.adapt else None

    def _maybe_push(self) -> None:
        if self._push is None:
            return
        now = self._clock()
        if now - self._last_push < config.NAV_PUSH_MIN_PERIOD_S:
            return
        self._last_push = now
        site, _ = self.observer()
        self._push(site, self.flight_phase())


def run_udp_listener(state: NavState, port: int = config.TRIMBLE_UDP_PORT, host: str = "0.0.0.0",
                     stop: Optional[threading.Event] = None) -> None:
    """Bind, receive, parse, assemble; re-binds after any socket error."""
    while stop is None or not stop.is_set():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(1.0)
            sock.bind((host, port))
            while stop is None or not stop.is_set():
                try:
                    data, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                for line in data.decode("ascii", "replace").splitlines():
                    s = parse_sentence(line)
                    if s is not None:
                        state.handle(s)
        except OSError as e:
            log.warning("%s listener socket error: %s; rebinding", Code.NAV_BAD_SENTENCE.value, e)
            time.sleep(2.0)
        finally:
            sock.close()
