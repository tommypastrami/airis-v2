"""Scheduler: FOV-integrated choice, exclusions, priorities, re-scoring after a miss."""
from datetime import datetime, timezone

import healpy as hp
import numpy as np
import pytest

from airis import config
from airis.pointing import MountPos, Site, SunBox, radec_to_altaz, altaz_to_mount
from airis.scheduler import TileQueue

WHEN = datetime(2026, 12, 15, 3, 0, tzinfo=timezone.utc)
SITE = Site(-77.8, 166.7, 35000.0)       # McMurdo-ish, float altitude


def gaussian_map(nside, ra0, dec0, sigma_deg, nested=False):
    npix = hp.nside2npix(nside)
    theta, phi = hp.pix2ang(nside, np.arange(npix), nest=nested)
    ra, dec = np.degrees(phi), 90 - np.degrees(theta)
    vec0 = hp.ang2vec(np.radians(90 - dec0), np.radians(ra0))
    vec = hp.ang2vec(theta, phi)
    d = np.degrees(np.arccos(np.clip(vec @ vec0, -1, 1)))
    p = np.exp(-0.5 * (d / sigma_deg) ** 2)
    return p / p.sum(), ra, dec


def visible_point():
    """A sky point that is well above the horizon at SITE/WHEN."""
    for ra in range(0, 360, 15):
        for dec in (-80, -60, -40):
            alt, _ = radec_to_altaz(ra, dec, SITE, WHEN)
            if alt > 40:
                return float(ra), float(dec)
    raise RuntimeError("no visible point")


def test_fov_integration_prefers_a_pointing_that_covers_more_probability():
    ra0, dec0 = visible_point()
    nside = 64
    # two blobs: a sharp bright pixel-sized peak, and a broad patch holding more total probability
    p1, ra, dec = gaussian_map(nside, ra0, dec0, 0.3)
    p2, _, _ = gaussian_map(nside, ra0 + 25.0, dec0, 3.0)
    prob = 0.3 * p1 + 0.7 * p2
    q = TileQueue()
    q.set_map(prob, burst_id=1, iteration=1)
    n = q.rescore(SITE, WHEN, MountPos(0, 0))
    assert n > 0
    best = q.pop()
    assert abs(((best.ra - (ra0 + 25.0) + 180) % 360) - 180) < 4.0, (best.ra, best.dec)
    assert best.p_fov > 0.4
    # the single most probable PIXEL is in the sharp blob - v1 would have gone there
    i = np.argmax(prob)
    assert abs((((ra[i] - ra0 + 180) % 360) - 180) * np.cos(np.radians(dec0))) < 1.5 and abs(dec[i] - dec0) < 1.5


def test_miss_marks_footprint_and_next_tile_moves_on():
    ra0, dec0 = visible_point()
    prob, ra, dec = gaussian_map(32, ra0, dec0, 6.0)
    q = TileQueue()
    q.set_map(prob, 1, 1)
    q.rescore(SITE, WHEN, MountPos(0, 0))
    first = q.pop()
    q.mark_imaged(first.ra, first.dec)
    q.rescore(SITE, WHEN, MountPos(first.el, first.az))
    second = q.pop()
    assert max(abs(second.ra - first.ra), abs(second.dec - first.dec)) > 2.0
    assert second.p_fov < first.p_fov
    assert q.covered.sum() > 0


def test_sun_and_horizon_exclusions():
    ra0, dec0 = visible_point()
    prob, ra, dec = gaussian_map(32, ra0, dec0, 2.0)
    q = TileQueue()
    q.set_map(prob, 1, 1)
    alt, az = radec_to_altaz(ra0, dec0, SITE, WHEN)
    sun_at_target = SunBox(altaz_to_mount(alt, az))
    assert q.rescore(SITE, WHEN, MountPos(0, 0), sun=sun_at_target) == 0 or q.pop().p_fov < 0.5
    # below the horizon: a point on the far side of the sky
    q2 = TileQueue()
    far = gaussian_map(32, (ra0 + 180) % 360, -dec0, 2.0)[0]
    q2.set_map(far, 2, 1)
    assert q2.rescore(SITE, WHEN, MountPos(0, 0)) == 0


def test_adapt_outranks_gcn_and_new_map_drops_gcn_points():
    ra0, dec0 = visible_point()
    q = TileQueue()
    q.add_point(ra0, dec0, "gcn", 0)
    q.add_point(ra0 + 2, dec0, "adapt", 5)
    q.rescore(SITE, WHEN, MountPos(0, 0))
    assert q.pop().source == "adapt"
    prob = gaussian_map(16, ra0, dec0, 5.0)[0]
    q.set_map(prob, 5, 1)
    q.rescore(SITE, WHEN, MountPos(0, 0))
    assert all(t.source != "gcn" for t in q._heap)


def test_bad_map_is_a_named_fault():
    from airis.errors import Code, Fault
    q = TileQueue()
    with pytest.raises(Fault) as e:
        q.set_map(np.ones(1000), 1, 1)
    assert e.value.code == Code.SCH_BAD_MAP
