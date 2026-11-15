"""Bench check of the imaging chain on the Jetson: one frame through camera, catalog and SFFT.

Replaces the old file-dialog test in image_subtraction.py. The WCS attached here is NOMINAL
(field center at the frame center, nominal plate scale, no roll), so the subtraction will be
misaligned until registration exists. This tool only shows that the camera, the catalog and
SFFT run at all, and writes the three images for a look.

    export LD_LIBRARY_PATH=...   # see README, Run
    python -m tools.imaging_bench --ra 83.8 --dec -5.4 --exposure-s 4 --out /tmp/bench
"""
import argparse
import os
import sys

import numpy as np
from astropy.io import fits

from airis import config
from airis.camera import XimeaCamera
from airis.catalog import GaiaCatalog
from airis.detect import LinearWCS, SFFTSubtractor, render_reference


def main(argv=None) -> int:  # pragma: no cover - hardware only
    p = argparse.ArgumentParser(description="one frame through camera, catalog and SFFT (Jetson only)")
    p.add_argument("--ra", type=float, required=True, help="field center RA, degrees")
    p.add_argument("--dec", type=float, required=True, help="field center Dec, degrees")
    p.add_argument("--exposure-s", type=float, default=config.EXPOSURE_S / 2, help="one half-exposure by default")
    p.add_argument("--out", required=True, help="directory for science.fits, reference.fits and difference.fits")
    a = p.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)

    cam = XimeaCamera()
    try:
        frame = cam.capture(a.ra, a.dec, a.exposure_s)
        temp = cam.temperature_c()
    finally:
        cam.close()
    h, w = frame.data.shape                                   # whatever the camera delivered (binned 3x3)
    print(f"frame {w} x {h} {frame.data.dtype} min={frame.data.min()} max={frame.data.max()} utc={frame.utc.isoformat()}")
    print(f"sensor board temperature: {temp} C")

    frame.wcs = LinearWCS(a.ra, a.dec, w / 2, h / 2, config.CAM_FOV_W_DEG / w)   # nominal, unregistered
    catalog = GaiaCatalog(config.GAIA_CHUNK_DIR)
    stars = catalog.stars(a.ra, a.dec)
    print(f"catalog: {len(stars)} stars within {catalog.radius_deg:.2f} deg of the field center")
    sci = frame.data.astype(float)
    ref = render_reference(stars, frame.wcs, sci.shape, frame.psf_sigma_px)
    diff = SFFTSubtractor().subtract(sci, ref, frame.psf_sigma_px, frame.psf_sigma_px)

    for name, arr in (("science", sci), ("reference", ref), ("difference", diff)):
        path = os.path.join(a.out, f"{name}.fits")
        fits.PrimaryHDU(np.asarray(arr, dtype=np.float32)).writeto(path, overwrite=True)
        print(f"{name}: min={np.min(arr):.1f} max={np.max(arr):.1f} -> {path}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
