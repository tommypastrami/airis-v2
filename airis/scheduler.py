"""Transient priority queue: which pointing is worth the next ten seconds."""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

import numpy as np

from . import config
from .errors import Code, Fault
from .pointing import MountPos, Site, SunBox, radec_to_altaz, altaz_to_mount, slew_time_s, travel_deg

try:
    import healpy as hp
    _HAVE_HEALPY = True
except Exception:  # pragma: no cover
    _HAVE_HEALPY = False

PRIORITY = {"adapt": 0, "ground": 0, "gcn": 1}


@dataclass(order=True)
class Tile:
    sort_key: tuple = field(repr=False, compare=True)
    ra: float = field(compare=False, default=0.0)
    dec: float = field(compare=False, default=0.0)
    el: float = field(compare=False, default=0.0)
    az: float = field(compare=False, default=0.0)
    p_fov: float = field(compare=False, default=0.0)
    t_slew: float = field(compare=False, default=0.0)
    score: float = field(compare=False, default=0.0)
    source: str = field(compare=False, default="adapt")
    pix: int = field(compare=False, default=-1)
    burst_id: int = field(compare=False, default=0)


class TileQueue:
    def __init__(self, fov_w: float = config.CAM_FOV_W_DEG, fov_h: float = config.CAM_FOV_H_DEG,
                 exposure_s: float = config.EXPOSURE_S, proc_s: float = config.PROC_S,
                 n_candidates: int = config.SCHED_CANDIDATE_PIXELS):
        self.fov_w, self.fov_h = fov_w, fov_h
        self.exposure_s, self.proc_s, self.n_candidates = exposure_s, proc_s, n_candidates
        self.prob: Optional[np.ndarray] = None
        self.nside = 0
        self.nested = config.HEALPIX_NESTED
        self.covered: Optional[np.ndarray] = None
        self.burst_id = 0
        self.iteration = -1
        self._heap: list[Tile] = []
        self._points: list[tuple[float, float, str, int]] = []   # single pointings (alerts without a map)
        self.imaged: list[tuple[float, float]] = []

    # ---- inputs
    def set_map(self, prob: np.ndarray, burst_id: int, iteration: int, nested: bool = config.HEALPIX_NESTED) -> None:
        if not _HAVE_HEALPY:
            raise Fault(Code.SCH_BAD_MAP, "healpy not installed")
        # NaN, infinite or negative (UNSEEN) pixels would give NaN scores and corrupt the heap order.
        prob = np.nan_to_num(np.asarray(prob, dtype=float), nan=0.0, posinf=0.0, neginf=0.0)
        prob[prob < 0] = 0.0
        try:
            self.nside = hp.npix2nside(prob.size)
        except Exception as e:
            raise Fault(Code.SCH_BAD_MAP, f"not a HEALPix map: {e}", {"npix": prob.size})
        if burst_id != self.burst_id:
            self.clear()
        self.burst_id, self.iteration, self.nested = burst_id, iteration, nested
        s = prob.sum()
        self.prob = prob / s if s > 0 else prob
        if self.covered is None or self.covered.size != prob.size:
            self.covered = np.zeros(prob.size, dtype=bool)
            for ra, dec in self.imaged:                  # a new resolution: replay what was already imaged
                self.covered[self._footprint(ra, dec)] = True
        self._points = [p for p in self._points if p[2] != "gcn"]

    def add_point(self, ra: float, dec: float, source: str = "adapt", burst_id: int = 0) -> None:
        """A single pointing (an alert without a map yet)."""
        if burst_id and burst_id != self.burst_id:
            self.clear()
            self.burst_id = burst_id
        self._points.append((ra, dec, source, burst_id))

    def mark_imaged(self, ra: float, dec: float) -> None:
        self.imaged.append((ra, dec))
        if self.prob is not None:
            self.covered[self._footprint(ra, dec)] = True
        self._points = [p for p in self._points if not (abs(p[0] - ra) < 1e-6 and abs(p[1] - dec) < 1e-6)]

    def clear(self) -> None:
        self._heap, self._points, self.imaged = [], [], []
        self.prob, self.covered = None, None
        self.iteration = -1

    # ---- geometry
    def _footprint(self, ra: float, dec: float) -> np.ndarray:
        """Indices of the pixels inside the (unrotated) camera rectangle centred on ra, dec."""
        half_diag = math.hypot(self.fov_w, self.fov_h) / 2.0
        vec = hp.ang2vec(math.radians(90.0 - dec), math.radians(ra))
        pix = hp.query_disc(self.nside, vec, math.radians(half_diag), nest=self.nested, inclusive=True)
        theta, phi = hp.pix2ang(self.nside, pix, nest=self.nested)
        pra, pdec = np.degrees(phi), 90.0 - np.degrees(theta)
        dx = ((pra - ra + 180.0) % 360.0 - 180.0) * math.cos(math.radians(dec))
        dy = pdec - dec
        inside = (np.abs(dx) <= self.fov_w / 2.0) & (np.abs(dy) <= self.fov_h / 2.0)
        return pix[inside]

    def _candidate_centres(self) -> list[tuple[float, float, int]]:
        if self.prob is None:
            return []
        live = np.where(self.covered, 0.0, self.prob)
        order = np.argsort(live)[::-1][: self.n_candidates]
        theta, phi = hp.pix2ang(self.nside, order, nest=self.nested)
        return [(float(np.degrees(p)), float(90.0 - np.degrees(t)), int(i)) for t, p, i in zip(theta, phi, order) if live[i] > 0]

    # ---- scoring
    def rescore(self, site: Site, when: Optional[datetime], mount: MountPos, sun: Optional[SunBox] = None,
                heading: Optional[float] = None, exclusions: Optional[list[SunBox]] = None) -> int:
        """Rebuild the heap."""
        self._heap = []
        boxes = ([(sun, True)] if sun is not None else []) + [(b, False) for b in (exclusions or [])]
        for ra, dec, src, bid in self._points:
            self._push(ra, dec, 1.0, src, -1, bid, site, when, mount, boxes, heading)
        if self.prob is not None:
            live = np.where(self.covered, 0.0, self.prob)
            for ra, dec, pix in self._candidate_centres():
                p = float(live[self._footprint(ra, dec)].sum())
                if p <= 0:
                    continue
                self._push(ra, dec, p, "adapt", pix, self.burst_id, site, when, mount, boxes, heading)
        return len(self._heap)

    def _push(self, ra, dec, p, src, pix, bid, site, when, mount, boxes, heading) -> None:
        alt, az_geo = radec_to_altaz(ra, dec, site, when)
        if alt < config.MIN_ELEVATION_DEG:
            return
        pos = altaz_to_mount(alt, az_geo, heading)
        for box, with_margin in boxes:
            if box.contains(pos):
                return
            if with_margin and box.angular_distance(pos) < config.SUN_TILE_MARGIN_DEG:
                return
        t_slew = slew_time_s(travel_deg(mount, pos))
        score = p / (t_slew + self.exposure_s + self.proc_s)
        # heapq is a min-heap: lower priority number first, then higher score
        tile = Tile((PRIORITY.get(src, 1), -score), ra, dec, pos.el, pos.az, p, t_slew, score, src, pix, bid)
        heapq.heappush(self._heap, tile)

    def pop(self) -> Optional[Tile]:
        return heapq.heappop(self._heap) if self._heap else None

    def peek(self) -> Optional[Tile]:
        return self._heap[0] if self._heap else None

    def __len__(self) -> int:
        return len(self._heap)
