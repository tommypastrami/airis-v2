"""The only module that converts between coordinate frames.

Frames (every angle in the system is in exactly one of these, in degrees):

1. Sky, ICRS            ra 0..360 east-positive, dec -90..90
2. Horizon, geographic  alt -90..90, az 0..360 from true north through east
3. Mount, physical home el = alt + EL_OFFSET_DEG
                        az = wrap180(az_geo - heading + MOUNT_TO_GONDOLA_AZ_DEG)
                        both axes +/-180, no wrap-around; +el up, +az clockwise
                        seen from above. The frame is anchored by the bridge:
                        CAL -> MOVE HOME_*_OFFSET -> SETZERO.
4. Firmware index frame firmware-internal; the host never sees it
5. Encoder counts       firmware-internal

The camera pixel frame is NOT assumed anywhere; detect.py gets it from a WCS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Sequence

import numpy as np

from . import config
from .errors import Code, Fault

try:  # astropy is the flight dependency; the pure-numpy fallback keeps tests honest without it
    from astropy.coordinates import AltAz, EarthLocation, SkyCoord, get_sun
    from astropy.time import Time
    import astropy.units as u
    from astropy.utils import iers
    iers.conf.auto_download = False
    _HAVE_ASTROPY = True
except Exception:  # pragma: no cover
    _HAVE_ASTROPY = False


def wrap180(x: float) -> float:
    """Map any angle to (-180, 180]."""
    y = (x + 180.0) % 360.0 - 180.0
    return 180.0 if y == -180.0 else y


def wrap360(x: float) -> float:
    return x % 360.0


@dataclass(frozen=True)
class Site:
    lat_deg: float = config.DEFAULT_LAT_DEG
    lon_deg: float = config.DEFAULT_LON_DEG
    height_m: float = config.DEFAULT_HEIGHT_M


@dataclass(frozen=True)
class MountPos:
    el: float
    az: float

    def flipped(self) -> "MountPos":
        """The other mount attitude that aims the same boresight (camera over the top)."""
        el = 180.0 - self.el if self.el >= 0 else -180.0 - self.el
        return MountPos(el, wrap180(self.az + 180.0))

    def in_range(self) -> bool:
        return (config.ANGLE_MIN_DEG <= self.el <= config.ANGLE_MAX_DEG
                and config.ANGLE_MIN_DEG <= self.az <= config.ANGLE_MAX_DEG)


# ---- 1 -> 2 ------------------------------------------------------------------

def _lst_deg(when: datetime, lon_deg: float) -> float:
    """Local sidereal time in degrees (fallback path, ~0.01 deg accuracy)."""
    jd = when.timestamp() / 86400.0 + 2440587.5
    t = (jd - 2451545.0) / 36525.0
    gmst = 280.46061837 + 360.98564736629 * (jd - 2451545.0) + 0.000387933 * t * t
    return wrap360(gmst + lon_deg)


def radec_to_altaz(ra_deg: float, dec_deg: float, site: Site,
                   when: Optional[datetime] = None) -> tuple[float, float]:
    """ICRS -> geographic (alt, az), az 0..360 from north through east."""
    when = when or datetime.now(timezone.utc)
    if _HAVE_ASTROPY:
        loc = EarthLocation(lat=site.lat_deg * u.deg, lon=site.lon_deg * u.deg, height=site.height_m * u.m)
        c = SkyCoord(ra=ra_deg * u.deg, dec=dec_deg * u.deg, frame="icrs")
        aa = c.transform_to(AltAz(obstime=Time(when), location=loc))
        return float(aa.alt.deg), float(aa.az.deg)
    ha = math.radians(wrap180(_lst_deg(when, site.lon_deg) - ra_deg))
    dec = math.radians(dec_deg)
    lat = math.radians(site.lat_deg)
    sin_alt = math.sin(dec) * math.sin(lat) + math.cos(dec) * math.cos(lat) * math.cos(ha)
    alt = math.asin(max(-1.0, min(1.0, sin_alt)))
    y = -math.sin(ha) * math.cos(dec)
    x = math.sin(dec) * math.cos(lat) - math.cos(dec) * math.sin(lat) * math.cos(ha)
    return math.degrees(alt), wrap360(math.degrees(math.atan2(y, x)))


def sun_altaz(site: Site, when: Optional[datetime] = None) -> tuple[float, float]:
    when = when or datetime.now(timezone.utc)
    if _HAVE_ASTROPY:
        loc = EarthLocation(lat=site.lat_deg * u.deg, lon=site.lon_deg * u.deg, height=site.height_m * u.m)
        t = Time(when)
        s = get_sun(t).transform_to(AltAz(obstime=t, location=loc))
        return float(s.alt.deg), float(s.az.deg)
    # low-precision solar position (Meeus), enough for a keep-out box
    jd = when.timestamp() / 86400.0 + 2440587.5
    n = jd - 2451545.0
    L = wrap360(280.460 + 0.9856474 * n)
    g = math.radians(wrap360(357.528 + 0.9856003 * n))
    lam = math.radians(L + 1.915 * math.sin(g) + 0.020 * math.sin(2 * g))
    eps = math.radians(23.439 - 0.0000004 * n)
    ra = math.degrees(math.atan2(math.cos(eps) * math.sin(lam), math.cos(lam)))
    dec = math.degrees(math.asin(math.sin(eps) * math.sin(lam)))
    return radec_to_altaz(wrap360(ra), dec, site, when)


# ---- 2 -> 3 ------------------------------------------------------------------

def altaz_to_mount(alt_deg: float, az_geo_deg: float, heading_deg: Optional[float] = None) -> MountPos:
    """Geographic horizon -> mount physical-home frame.

    ``heading_deg`` is the gondola heading (HDT, true). It is applied only when
    HEADING_CORRECTION_ENABLED, and then it must be applied to the sun through
    this same function, so the keep-out and the pointing share one frame.
    """
    el = alt_deg + config.EL_OFFSET_DEG
    if config.HEADING_CORRECTION_ENABLED:
        if heading_deg is None:
            raise Fault(Code.PTG_HEADING_STALE, "heading correction enabled but no heading")
        az = wrap180(az_geo_deg - heading_deg + config.MOUNT_TO_GONDOLA_AZ_DEG)
    else:
        az = wrap180(az_geo_deg + config.AZ_OFFSET_DEG)
    return MountPos(el, az)


def mount_to_altaz(p: MountPos, heading_deg: Optional[float] = None) -> tuple[float, float]:
    alt = p.el - config.EL_OFFSET_DEG
    if config.HEADING_CORRECTION_ENABLED:
        if heading_deg is None:
            raise Fault(Code.PTG_HEADING_STALE, "heading correction enabled but no heading")
        az = wrap360(p.az + heading_deg - config.MOUNT_TO_GONDOLA_AZ_DEG)
    else:
        az = wrap360(p.az - config.AZ_OFFSET_DEG)
    return alt, az


def radec_to_mount(ra_deg: float, dec_deg: float, site: Site, when: Optional[datetime] = None,
                   heading_deg: Optional[float] = None) -> MountPos:
    alt, az = radec_to_altaz(ra_deg, dec_deg, site, when)
    if alt < config.MIN_ELEVATION_DEG:
        raise Fault(Code.PTG_BELOW_HORIZON, f"alt={alt:.1f}", {"ra": ra_deg, "dec": dec_deg})
    return altaz_to_mount(alt, az, heading_deg)


def sun_mount(site: Site, when: Optional[datetime] = None, heading_deg: Optional[float] = None) -> MountPos:
    alt, az = sun_altaz(site, when)
    return altaz_to_mount(alt, az, heading_deg)


# ---- Sun keep-out and routing (mount frame) ----------------------------------

@dataclass(frozen=True)
class SunBox:
    center: MountPos
    half_el: float = config.SunBoxSpec().half_el
    half_az: float = config.SunBoxSpec().half_az

    def contains(self, p: MountPos, margin: float = 0.0) -> bool:
        """True if either attitude of ``p`` (direct or flipped) looks into the box."""
        for q in (p, p.flipped()):
            if (abs(q.el - self.center.el) <= self.half_el + margin
                    and abs(wrap180(q.az - self.center.az)) <= self.half_az + margin):
                return True
        return False

    def segment_crosses(self, a: MountPos, b: MountPos, step_deg: float = 1.0) -> bool:
        n = max(2, int(math.hypot(b.el - a.el, b.az - a.az) / step_deg) + 1)
        for i in range(n + 1):
            t = i / n
            if self.contains(MountPos(a.el + t * (b.el - a.el), a.az + t * (b.az - a.az))):
                return True
        return False

    def corners(self, clear: float = config.SUN_CLEAR_DEG) -> list[MountPos]:
        out = []
        for se in (-1, 1):
            for sa in (-1, 1):
                out.append(MountPos(self.center.el + se * (self.half_el + clear),
                                    wrap180(self.center.az + sa * (self.half_az + clear))))
        return [c for c in out if c.in_range()]

    def angular_distance(self, p: MountPos) -> float:
        """Max-norm distance from the box edge (0 inside), for scheduler exclusion."""
        de = max(0.0, abs(p.el - self.center.el) - self.half_el)
        da = max(0.0, abs(wrap180(p.az - self.center.az)) - self.half_az)
        return max(de, da)


def path_length(pts: Sequence[MountPos]) -> float:
    return sum(max(abs(b.el - a.el), abs(b.az - a.az)) for a, b in zip(pts, pts[1:]))


def plan_route(start: MountPos, target: MountPos, box: Optional[SunBox],
               allow_flip: bool = True) -> list[MountPos]:
    """Waypoints from ``start`` to ``target`` (target included) that never enter the box.

    Order of preference: direct leg; one corner waypoint; two corners along one
    edge of the box; then the same three for the flipped attitude of the target.
    Shortest clear route wins. The firmware keeps its own box check as the last
    line of defense.
    """
    if not target.in_range():
        raise Fault(Code.PTG_RANGE, f"el={target.el:.1f} az={target.az:.1f}")
    candidates = [target] + ([target.flipped()] if allow_flip and target.el >= 0 else [])
    if box is None:
        return [target]
    best: Optional[list[MountPos]] = None
    for tgt in candidates:
        if not tgt.in_range() or box.contains(tgt):
            continue
        if not box.segment_crosses(start, tgt):
            route = [tgt]
        else:
            route = None
            corners = box.corners()
            options = [[c, tgt] for c in corners]
            options += [[c1, c2, tgt] for c1 in corners for c2 in corners
                        if c1 != c2 and (c1.el == c2.el or c1.az == c2.az)]   # along one edge of the box
            for cand in options:
                legs = [start] + cand
                if any(box.segment_crosses(a, b) for a, b in zip(legs, legs[1:])):
                    continue
                if route is None or path_length(legs) < path_length([start] + route):
                    route = cand
            if route is None:
                continue
        margin = 0.0 if tgt == target else config.FLIP_MARGIN_DEG   # hysteresis: prefer the direct attitude
        if best is None or path_length([start] + route) + margin < path_length([start] + best):
            best = route
    if best is None:
        raise Fault(Code.PTG_SUN_RADIUS, "no sun-safe route", {"el": target.el, "az": target.az})
    return best


# ---- Mount timing model -------------------------------------------------------

def slew_time_s(theta_deg: float, land: bool = True) -> float:
    """Time for the worst axis to travel ``theta_deg`` with the firmware's profile.

    Quintic ramps: distance per ramp = 0.5 * Vp * Ta, Ta = 1.875 * Vp / A.
    Calibrate against moves.csv; the bench median was 3.7 s.
    """
    v, a = config.SLEW_SPEED_DPS, config.SLEW_ACCEL_DPS2
    ta = 1.875 * v / a
    td = 1.875 * v / (a * config.DECEL_RATIO)
    d_ramps = 0.5 * v * (ta + td)
    land_t = (config.LAND_ZONE_DEG / config.LAND_SPEED_DPS + 0.5) if land else 0.0
    theta = abs(theta_deg)
    if theta <= d_ramps:
        # triangular profile, scale the ramps down
        scale = math.sqrt(theta / d_ramps) if d_ramps > 0 else 0.0
        return (ta + td) * scale + land_t + 0.2
    return ta + td + (theta - d_ramps) / v + land_t + 0.2


def travel_deg(a: MountPos, b: MountPos) -> float:
    return max(abs(b.el - a.el), abs(b.az - a.az))
