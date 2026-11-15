"""GaiaCatalog: the RA window widens toward the pole, wraps at 0/360, spans bands, and an uncovered field is a named fault."""
import math

import numpy as np
import pytest
from astropy.table import Table

from airis import config
from airis.catalog import GaiaCatalog
from airis.errors import Code, Fault

R = 1.2 * math.hypot(config.CAM_FOV_W_DEG, config.CAM_FOV_H_DEG) / 2      # the selection radius


def write_chunk(d, lo, hi, rows):
    """rows: (ra, dec, g_mag) tuples. The file name follows the Jetson layout."""
    ra, dec, mag = (np.array(c, dtype=float) for c in zip(*rows))
    t = Table({"ra": ra, "dec": dec, "phot_g_mean_mag": mag, "ruwe": np.ones(len(ra))})
    t.write(d / f"airis_gaia_dr3_G8_G18_dec_{lo}_to_{hi}_ruwe1.4.fits", format="fits")


def test_ra_window_widens_near_the_pole(tmp_path):
    write_chunk(tmp_path, -80, -75, [(100.0, -78.0, 18.0),              # center
                                     (130.0, -78.0, 13.0),              # 30 deg of RA away: ~6 deg on the sky
                                     (100.0, -78.0 + R + 0.5, 12.0),    # outside the dec cut
                                     (280.0, -78.0, 12.0)])             # across the pole in RA
    got = GaiaCatalog(str(tmp_path)).stars(100.0, -78.0)
    assert sorted(round(s.ra) for s in got) == [100, 130]
    by_ra = {round(s.ra): s for s in got}
    assert by_ra[100].flux == pytest.approx(1.0)                        # G = zero point
    assert by_ra[130].flux == pytest.approx(100.0)                      # 5 mag brighter


def test_field_straddling_ra_zero(tmp_path):
    write_chunk(tmp_path, -35, -30, [(359.0, -31.0, 15.0), (3.0, -31.0, 15.0), (20.0, -31.0, 15.0), (340.0, -31.0, 15.0)])
    got = GaiaCatalog(str(tmp_path)).stars(1.0, -31.0)
    assert sorted(s.ra for s in got) == [3.0, 359.0]


def test_field_spanning_two_bands_and_cache_follows_the_field(tmp_path):
    write_chunk(tmp_path, -80, -75, [(50.0, -77.0, 15.0)])
    write_chunk(tmp_path, -75, -70, [(50.0, -73.0, 15.0)])
    write_chunk(tmp_path, -40, -35, [(50.0, -37.0, 15.0)])
    cat = GaiaCatalog(str(tmp_path))
    assert sorted(s.dec for s in cat.stars(50.0, -75.0)) == [-77.0, -73.0]
    assert len(cat._cache) == 2
    assert [s.dec for s in cat.stars(50.0, -37.0)] == [-37.0]
    assert len(cat._cache) == 1                                         # the polar bands were dropped


def test_non_finite_magnitudes_are_dropped(tmp_path):
    write_chunk(tmp_path, -35, -30, [(10.0, -32.0, 15.0), (10.1, -32.0, float("nan"))])
    assert len(GaiaCatalog(str(tmp_path)).stars(10.0, -32.0)) == 1


def test_uncovered_field_and_missing_directory_are_named_faults(tmp_path):
    write_chunk(tmp_path, -80, -75, [(50.0, -77.0, 15.0)])
    with pytest.raises(Fault) as e:
        GaiaCatalog(str(tmp_path)).stars(50.0, 20.0)
    assert e.value.code == Code.DET_REFERENCE_EMPTY
    with pytest.raises(Fault) as e:
        GaiaCatalog(str(tmp_path / "nope")).stars(50.0, -77.0)
    assert e.value.code == Code.DET_REFERENCE_EMPTY
