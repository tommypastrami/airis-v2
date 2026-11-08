"""Synthetic sky and a fake camera: a catalog of stars, frames rendered through a WCS, an optional transient.

The camera renders whatever the catalog says is inside the field at the tile
center, with the frame's own rotation/scale (so registration is exercised, not
assumed), Poisson + read noise, hot pixels, and - if the tile covers the true
burst position - a transient of the requested flux.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

import numpy as np
from scipy import ndimage

from airis import config
from airis.detect import Frame, LinearWCS, Star


@dataclass
class SkyModel:
    seed: int = 1
    n_stars: int = 4000
    ra0: float = 83.8
    dec0: float = -5.4
    extent_deg: float = 20.0
    flux_min: float = 300.0
    flux_max: float = 20000.0
    stars: list[Star] = field(default_factory=list)

    def __post_init__(self):
        rng = random.Random(self.seed)
        self.stars = []
        for _ in range(self.n_stars):
            ra = self.ra0 + rng.uniform(-self.extent_deg, self.extent_deg) / math.cos(math.radians(self.dec0))
            dec = self.dec0 + rng.uniform(-self.extent_deg, self.extent_deg)
            flux = self.flux_min * (self.flux_max / self.flux_min) ** rng.random() ** 2
            self.stars.append(Star(ra % 360.0, dec, flux))

    def catalog(self, ra: float, dec: float, radius_deg: float = 8.0) -> list[Star]:
        out = []
        for s in self.stars:
            dra = ((s.ra - ra + 180.0) % 360.0 - 180.0) * math.cos(math.radians(dec))
            if abs(dra) <= radius_deg and abs(s.dec - dec) <= radius_deg:
                out.append(s)
        return out


class FakeCamera:
    def __init__(self, sky: SkyModel, shape: tuple[int, int] = (240, 360), scale_deg: float = config.CAM_FOV_W_DEG / 360,
                 rot_deg: float = 7.0, flip_x: bool = True, psf_sigma: float = config.PSF_SIGMA_PX,
                 read_noise: float = 3.0, sky_level: float = 50.0, hot_pixels: int = 20,
                 transient: Optional[tuple[float, float, float]] = None, seed: int = 5, temperature_c: float = 20.0):
        self.sky, self.shape, self.scale_deg, self.rot_deg, self.flip_x = sky, shape, scale_deg, rot_deg, flip_x
        self.psf_sigma, self.read_noise, self.sky_level, self.hot_pixels = psf_sigma, read_noise, sky_level, hot_pixels
        self.transient = transient          # (ra, dec, flux)
        self.rng = np.random.default_rng(seed)
        self.temp_c = temperature_c
        self.captures: list[tuple[float, float]] = []

    def temperature_c(self) -> Optional[float]:
        return self.temp_c

    def wcs_for(self, ra: float, dec: float) -> LinearWCS:
        h, w = self.shape
        return LinearWCS(ra, dec, w / 2.0, h / 2.0, self.scale_deg, self.rot_deg, self.flip_x)

    def capture(self, tile_ra: float, tile_dec: float, exposure_s: float) -> Frame:
        self.captures.append((tile_ra, tile_dec))
        wcs = self.wcs_for(tile_ra, tile_dec)
        stars = self.sky.catalog(tile_ra, tile_dec)
        img = np.zeros(self.shape, dtype=float)
        if stars:
            x, y = wcs.sky_to_pix(np.array([s.ra for s in stars]), np.array([s.dec for s in stars]))
            fl = np.array([s.flux for s in stars]) * exposure_s / 8.0
            self._stamp(img, x, y, fl)
        if self.transient is not None:
            tx, ty = wcs.sky_to_pix(np.array([self.transient[0]]), np.array([self.transient[1]]))
            self._stamp(img, tx, ty, np.array([self.transient[2] * exposure_s / 8.0]))
        img = ndimage.gaussian_filter(img, self.psf_sigma) + self.sky_level
        img = self.rng.poisson(np.clip(img, 0, None)).astype(float) + self.rng.normal(0, self.read_noise, self.shape)
        for _ in range(self.hot_pixels):
            yy, xx = self.rng.integers(0, self.shape[0]), self.rng.integers(0, self.shape[1])
            img[yy, xx] += self.rng.uniform(300, 3000)
        return Frame(img, datetime.now(timezone.utc), exposure_s, self.psf_sigma, wcs, tile_ra, tile_dec)

    def _stamp(self, img, x, y, fl):
        h, w = img.shape
        xi, yi = np.rint(x).astype(int), np.rint(y).astype(int)
        ok = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
        np.add.at(img, (yi[ok], xi[ok]), fl[ok])
