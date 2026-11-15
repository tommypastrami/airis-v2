"""Reference stars for a field, from the Gaia DR3 chunk files on the Jetson.

Ported from the query half of the old image_subtraction.generateReferenceImage (table
read, box cut, magnitude to flux). Rendering is render_reference in detect.py.

Chunks are 5-degree declination bands named like
    airis_gaia_dr3_G8_G18_dec_-80_to_-75_ruwe1.4.fits
That layout is inferred from one filename and is owned by the catalog team; only the
``dec_<lo>_to_<hi>`` part of the name is relied on here.
"""
from __future__ import annotations

import logging
import math
import os
import re

import numpy as np

from . import config
from .detect import Star
from .errors import Code, Fault

log = logging.getLogger("airis.catalog")

_BAND_RE = re.compile(r"dec_(-?\d+)_to_(-?\d+)")
_COLUMNS = ("ra", "dec", "phot_g_mean_mag")


class GaiaCatalog:
    """stars(ra, dec) -> the catalog stars within a generous circle around the field center.

    The field roll is unknown on an el/az mount, so the cut is on the frame diagonal (with
    20 % margin) in declination, and that width divided by cos(dec) in right ascension so
    the window stays the same angular size near the pole.
    """

    def __init__(self, chunk_dir: str):
        self.chunk_dir = chunk_dir
        self.radius_deg = 1.2 * math.hypot(config.CAM_FOV_W_DEG, config.CAM_FOV_H_DEG) / 2
        self._bands: list[tuple[float, float, str]] = []       # (dec_lo, dec_hi, path)
        self._cache: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        if not os.path.isdir(chunk_dir):
            log.warning("%s Gaia chunk directory %s not found; every field will be empty",
                        Code.DET_REFERENCE_EMPTY.value, chunk_dir)
            return
        for name in sorted(os.listdir(chunk_dir)):
            m = _BAND_RE.search(name)
            if m:
                lo, hi = sorted((float(m.group(1)), float(m.group(2))))
                self._bands.append((lo, hi, os.path.join(chunk_dir, name)))
        log.info("Gaia catalog: %d declination bands in %s", len(self._bands), chunk_dir)

    def stars(self, ra: float, dec: float) -> list[Star]:
        r = self.radius_deg
        paths = [p for lo, hi, p in self._bands if lo <= dec + r and hi >= dec - r]
        if not paths:
            raise Fault(Code.DET_REFERENCE_EMPTY, f"no Gaia chunk covers dec {dec:.2f} +- {r:.2f}",
                        {"chunk_dir": self.chunk_dir})
        for p in paths:
            if p not in self._cache:
                self._cache[p] = self._load(p)
        for p in list(self._cache):                              # keep only the bands this call used
            if p not in paths:
                del self._cache[p]
        cat_ra = np.concatenate([self._cache[p][0] for p in paths])
        cat_dec = np.concatenate([self._cache[p][1] for p in paths])
        cat_mag = np.concatenate([self._cache[p][2] for p in paths])

        keep = np.abs(cat_dec - dec) <= r
        if abs(dec) + r < 90.0:
            half_ra = r / math.cos(math.radians(abs(dec) + r))
            dra = (cat_ra - ra + 180.0) % 360.0 - 180.0
            keep &= np.abs(dra) <= half_ra
        flux = 10.0 ** (-0.4 * (cat_mag[keep] - config.GAIA_ZERO_POINT))
        return [Star(float(a), float(d), float(f)) for a, d, f in zip(cat_ra[keep], cat_dec[keep], flux)]

    @staticmethod
    def _load(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        from astropy.table import Table
        t = Table.read(path, memmap=True)
        ra, dec, mag = (np.array(t[c], dtype=float) for c in _COLUMNS)
        ok = np.isfinite(mag) & np.isfinite(ra) & np.isfinite(dec)
        log.info("loaded %s: %d stars (%d without a finite G magnitude dropped)",
                 os.path.basename(path), int(ok.sum()), int((~ok).sum()))
        return ra[ok], dec[ok], mag[ok]
