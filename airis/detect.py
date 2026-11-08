"""Register, clean, subtract, detect, measure, vet. Output: a row per candidate with a confidence.

Pipeline
    register   science frame -> pixel/sky mapping (a WCS). Pluggable: astrometry.net
               in flight; IdentityRegistrar when frames are already on the catalog grid.
    reference  catalog stars projected THROUGH THAT WCS and rendered with the
               frame's PSF (never a flat dRA/dDec grid).
    clean      3x3 median for hot pixels / cosmic rays; sky background removed.
    subtract   pluggable: SFFTSubtractor (cupy) in flight; MatchedGaussianSubtractor
               (PSF-match by Gaussian blur + robust flux scaling) for simulation.
    detect     matched filter (convolve with the PSF), threshold at K x MAD, label peaks.
    measure    x, y, ra, dec, peak, SNR, FWHM, ellipticity, distance to nearest catalog star.
    vet        named vetoes, each a column; confidence = product of their weights.

Every stage either returns data or raises Fault(DET-xx); nothing returns a bare bool.
"""
from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, asdict, field
from datetime import datetime, timezone
from typing import Callable, Optional, Protocol, Sequence

import numpy as np
from scipy import ndimage

from . import config
from .errors import Code, Fault


# ---- WCS: the minimal linear mapping we need (replace with astropy.wcs in flight) ----

@dataclass
class LinearWCS:
    """Tangent-plane approximation: pixel (x, y) <-> (ra, dec) with scale and rotation."""
    ra0: float
    dec0: float
    x0: float
    y0: float
    scale_deg: float          # degrees per pixel
    rot_deg: float = 0.0      # rotation of +x from +RA (east), counter-clockwise
    flip_x: bool = False      # True if +x points west

    def sky_to_pix(self, ra: np.ndarray, dec: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        dra = ((np.asarray(ra) - self.ra0 + 180.0) % 360.0 - 180.0) * math.cos(math.radians(self.dec0))
        ddec = np.asarray(dec) - self.dec0
        if self.flip_x:
            dra = -dra
        c, s = math.cos(math.radians(self.rot_deg)), math.sin(math.radians(self.rot_deg))
        x = (c * dra + s * ddec) / self.scale_deg + self.x0
        y = (-s * dra + c * ddec) / self.scale_deg + self.y0
        return x, y

    def pix_to_sky(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        dx = (np.asarray(x) - self.x0) * self.scale_deg
        dy = (np.asarray(y) - self.y0) * self.scale_deg
        c, s = math.cos(math.radians(self.rot_deg)), math.sin(math.radians(self.rot_deg))
        dra = c * dx - s * dy
        ddec = s * dx + c * dy
        if self.flip_x:
            dra = -dra
        return self.ra0 + dra / math.cos(math.radians(self.dec0)), self.dec0 + ddec


@dataclass
class Frame:
    data: np.ndarray
    utc: datetime
    exposure_s: float
    psf_sigma_px: float = config.PSF_SIGMA_PX
    wcs: Optional[LinearWCS] = None
    tile_ra: float = 0.0
    tile_dec: float = 0.0
    path: str = ""


@dataclass
class Star:
    ra: float
    dec: float
    flux: float


# ---- Registration ------------------------------------------------------------------

class Registrar(Protocol):
    def solve(self, frame: Frame) -> LinearWCS: ...


class IdentityRegistrar:
    """Frames already on a known grid (simulation): the WCS is whatever the frame carries."""

    def solve(self, frame: Frame) -> LinearWCS:
        if frame.wcs is None:
            raise Fault(Code.DET_REGISTRATION, "frame carries no WCS and no solver is configured")
        return frame.wcs


class AstrometryNetRegistrar:  # pragma: no cover - flight
    """Shell out to solve-field with local index files; parse the resulting WCS."""

    def __init__(self, index_dir: str, scale_low: float, scale_high: float):
        self.index_dir, self.scale_low, self.scale_high = index_dir, scale_low, scale_high

    def solve(self, frame: Frame) -> LinearWCS:
        raise Fault(Code.DET_REGISTRATION, "astrometry.net solver not wired in this build")


# ---- Reference rendering ----------------------------------------------------------

def render_reference(stars: Sequence[Star], wcs: LinearWCS, shape: tuple[int, int], psf_sigma_px: float) -> np.ndarray:
    img = np.zeros(shape, dtype=float)
    if not stars:
        raise Fault(Code.DET_REFERENCE_EMPTY, "no catalog stars in the field")
    ra = np.array([s.ra for s in stars]); dec = np.array([s.dec for s in stars]); fl = np.array([s.flux for s in stars])
    x, y = wcs.sky_to_pix(ra, dec)
    h, w = shape
    inside = (x >= -3 * psf_sigma_px) & (x < w + 3 * psf_sigma_px) & (y >= -3 * psf_sigma_px) & (y < h + 3 * psf_sigma_px)
    xi, yi = np.rint(x[inside]).astype(int), np.rint(y[inside]).astype(int)
    ok = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
    np.add.at(img, (yi[ok], xi[ok]), fl[inside][ok])
    return ndimage.gaussian_filter(img, psf_sigma_px)


# ---- Cleaning ---------------------------------------------------------------------

_RING = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=bool)


def clean(data: np.ndarray) -> np.ndarray:
    """Remove single-pixel spikes (hot pixels, cosmic rays) and the sky level.

    A real point source with PSF sigma >= 1 px has at least one neighbour above
    ~60 % of its peak; a spike does not. A pixel that exceeds its brightest
    neighbour by more than max(5 sigma, 40 % of its own height above sky) is a
    spike and is replaced by the 3x3 median.
    """
    data = data.astype(float)
    bg = np.median(data)
    sigma = mad(data)
    neigh_max = ndimage.maximum_filter(data, footprint=_RING, mode="nearest")
    excess = data - neigh_max
    spikes = excess > np.maximum(5.0 * sigma, 0.4 * (data - bg))
    med = ndimage.median_filter(data, size=3)
    out = np.where(spikes, med, data)
    return out - bg


def mad(x: np.ndarray) -> float:
    m = np.median(x)
    return float(1.4826 * np.median(np.abs(x - m)))


# ---- Subtraction ------------------------------------------------------------------

class Subtractor(Protocol):
    def subtract(self, science: np.ndarray, reference: np.ndarray, sci_sigma: float, ref_sigma: float) -> np.ndarray: ...


class MatchedGaussianSubtractor:
    """PSF-match the sharper image to the broader one by Gaussian blur, scale flux robustly, subtract."""

    def subtract(self, science, reference, sci_sigma, ref_sigma):
        sci, ref = science.astype(float), reference.astype(float)
        if sci_sigma > ref_sigma + 1e-6:
            ref = ndimage.gaussian_filter(ref, math.sqrt(sci_sigma ** 2 - ref_sigma ** 2))
        elif ref_sigma > sci_sigma + 1e-6:
            sci = ndimage.gaussian_filter(sci, math.sqrt(ref_sigma ** 2 - sci_sigma ** 2))
        bright = ref > np.percentile(ref, 99.5)
        if bright.sum() >= 10 and ref[bright].sum() > 0:
            scale = float(np.median(sci[bright] / np.maximum(ref[bright], 1e-9)))
            scale = min(max(scale, 1e-3), 1e3)
        else:
            scale = 1.0
        return sci - scale * ref


class SFFTSubtractor:  # pragma: no cover - flight
    def __init__(self, backend: str = "Cupy", device: int = 0):
        self.backend, self.device = backend, device

    def subtract(self, science, reference, sci_sigma, ref_sigma):
        try:
            from sfft.EasyCrowdedPacket import Easy_CrowdedPacket  # noqa: F401
        except ImportError as e:
            raise Fault(Code.DET_SUBTRACTION, f"sfft not available: {e}")
        raise Fault(Code.DET_SUBTRACTION, "SFFT file-based call not wired in this build; use MatchedGaussianSubtractor")


# ---- Candidates -------------------------------------------------------------------

@dataclass
class Candidate:
    x: float
    y: float
    ra: float
    dec: float
    peak: float
    snr: float
    area_px: int
    fwhm_px: float
    ellipticity: float
    nearest_star_px: float
    near_star: bool
    fwhm_ok: bool
    is_round: bool
    not_spike: bool
    persists: Optional[bool]
    confidence: float
    burst_id: int = 0
    tile_ra: float = 0.0
    tile_dec: float = 0.0
    frame_utc: str = ""


@dataclass
class DetectResult:
    candidates: list[Candidate]
    transient_found: bool
    noise_sigma: float
    n_peaks: int
    diff: np.ndarray = field(repr=False, default=None)
    wcs: Optional[LinearWCS] = None


VETO_WEIGHTS = {"near_star": 0.1, "fwhm_ok": 0.3, "is_round": 0.5, "not_spike": 0.1, "persists": 0.4}


def _moments(patch: np.ndarray) -> tuple[float, float, float, float]:
    """Return (cx, cy, fwhm, ellipticity) from intensity-weighted second moments."""
    patch = np.clip(patch, 0, None)
    tot = patch.sum()
    if tot <= 0:
        return 0.0, 0.0, 0.0, 1.0
    ys, xs = np.indices(patch.shape)
    cx, cy = (xs * patch).sum() / tot, (ys * patch).sum() / tot
    vx = ((xs - cx) ** 2 * patch).sum() / tot
    vy = ((ys - cy) ** 2 * patch).sum() / tot
    vxy = ((xs - cx) * (ys - cy) * patch).sum() / tot
    tr, det = vx + vy, vx * vy - vxy * vxy
    disc = max(tr * tr / 4 - det, 0.0)
    l1, l2 = tr / 2 + math.sqrt(disc), tr / 2 - math.sqrt(disc)
    fwhm = 2.3548 * math.sqrt(max((l1 + l2) / 2, 1e-9))
    ell = 1.0 - math.sqrt(max(l2, 1e-9) / max(l1, 1e-9))
    return cx, cy, fwhm, ell


class Pipeline:
    def __init__(self, registrar: Registrar = None, subtractor: Subtractor = None,
                 k_sigma: float = config.DETECT_K_SIGMA, csv_path: Optional[str] = config.CANDIDATES_CSV):
        self.registrar = registrar or IdentityRegistrar()
        self.subtractor = subtractor or MatchedGaussianSubtractor()
        self.k_sigma = k_sigma
        self.csv_path = csv_path

    def run(self, frame: Frame, stars: Sequence[Star], second: Optional[Frame] = None,
            burst_id: int = 0) -> DetectResult:
        wcs = self.registrar.solve(frame)
        sci = clean(frame.data)
        ref = render_reference(stars, wcs, sci.shape, frame.psf_sigma_px)
        diff = self.subtractor.subtract(sci, ref, frame.psf_sigma_px, frame.psf_sigma_px)
        filt = ndimage.gaussian_filter(diff, frame.psf_sigma_px)
        sigma = mad(filt)
        if sigma <= 0:
            raise Fault(Code.DET_SUBTRACTION, "zero noise in difference image")
        mask = filt > self.k_sigma * sigma
        labels, n = ndimage.label(mask)
        diff2 = None
        if second is not None:
            sci2 = clean(second.data)
            diff2 = ndimage.gaussian_filter(self.subtractor.subtract(sci2, ref, second.psf_sigma_px, frame.psf_sigma_px), frame.psf_sigma_px)
        sx, sy = wcs.sky_to_pix(np.array([s.ra for s in stars]), np.array([s.dec for s in stars]))
        cands: list[Candidate] = []
        r = int(math.ceil(3 * frame.psf_sigma_px)) + 1
        for lab in range(1, n + 1):
            ys, xs = np.nonzero(labels == lab)
            area = len(xs)
            i = np.argmax(filt[ys, xs])
            py, px = int(ys[i]), int(xs[i])
            y0, y1, x0, x1 = max(0, py - r), min(diff.shape[0], py + r + 1), max(0, px - r), min(diff.shape[1], px + r + 1)
            cx, cy, fwhm, ell = _moments(diff[y0:y1, x0:x1])
            cx, cy = cx + x0, cy + y0
            d_star = float(np.min(np.hypot(sx - cx, sy - cy))) if len(sx) else 1e9
            ra, dec = wcs.pix_to_sky(np.array([cx]), np.array([cy]))
            peak = float(filt[py, px])
            snr = peak / sigma
            expected_fwhm = 2.3548 * frame.psf_sigma_px * math.sqrt(2)   # matched-filtered width
            flags = {
                "near_star": d_star < 2.5 * frame.psf_sigma_px,
                "fwhm_ok": 0.5 * expected_fwhm <= fwhm <= 2.0 * expected_fwhm,
                "is_round": ell < 0.4,
                "not_spike": area >= config.MIN_PEAK_AREA_PX,
            }
            persists = None
            if diff2 is not None:
                persists = bool(diff2[max(0, py - 1):py + 2, max(0, px - 1):px + 2].max() > 0.5 * self.k_sigma * sigma)
            conf = 1.0
            if flags["near_star"]:
                conf *= VETO_WEIGHTS["near_star"]
            if not flags["fwhm_ok"]:
                conf *= VETO_WEIGHTS["fwhm_ok"]
            if not flags["is_round"]:
                conf *= VETO_WEIGHTS["is_round"]
            if not flags["not_spike"]:
                conf *= VETO_WEIGHTS["not_spike"]
            if persists is False:
                conf *= VETO_WEIGHTS["persists"]
            conf *= min(1.0, snr / (2.0 * self.k_sigma))
            cands.append(Candidate(cx, cy, float(ra[0]), float(dec[0]), peak, snr, area, fwhm, ell, d_star,
                                   flags["near_star"], flags["fwhm_ok"], flags["is_round"], flags["not_spike"],
                                   persists, conf, burst_id, frame.tile_ra, frame.tile_dec, frame.utc.isoformat()))
        cands.sort(key=lambda c: -c.confidence)
        found = any(c.confidence >= config.CONFIDENCE_THRESHOLD for c in cands)
        if self.csv_path:
            self._write_csv(cands)
        return DetectResult(cands, found, sigma, n, diff, wcs)

    def _write_csv(self, cands: list[Candidate]) -> None:
        new = not os.path.exists(self.csv_path)
        with open(self.csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(asdict(Candidate(*([0.0] * 10), False, False, False, False, None, 0.0)).keys()))
            if new:
                w.writeheader()
            for c in cands:
                w.writerow(asdict(c))
