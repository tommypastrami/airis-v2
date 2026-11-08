"""Detection: an injected transient is found with a confidence, a clean field is not, and every candidate is a CSV row."""
import csv

import numpy as np
import pytest

from airis import config
from airis.detect import Candidate, Pipeline
from airis.errors import Code, Fault
from sim.sky_sim import FakeCamera, SkyModel

SKY = SkyModel(seed=2)


def test_injected_transient_is_found_and_logged(tmp_path):
    ra, dec = SKY.ra0 + 0.4, SKY.dec0 - 0.3
    cam = FakeCamera(SKY, transient=(ra, dec, 6000.0), rot_deg=11.0, flip_x=True, seed=3)
    frame = cam.capture(SKY.ra0, SKY.dec0, 8.0)
    pipe = Pipeline(csv_path=str(tmp_path / "cand.csv"))
    res = pipe.run(frame, SKY.catalog(SKY.ra0, SKY.dec0), second=cam.capture(SKY.ra0, SKY.dec0, 8.0), burst_id=42)
    assert res.transient_found, [c.confidence for c in res.candidates[:5]]
    best = res.candidates[0]
    assert abs(best.ra - ra) < 0.05 and abs(best.dec - dec) < 0.05
    assert best.confidence >= config.CONFIDENCE_THRESHOLD and best.snr > config.DETECT_K_SIGMA
    rows = list(csv.DictReader(open(tmp_path / "cand.csv")))
    assert rows and rows[0]["burst_id"] == "42" and float(rows[0]["confidence"]) == pytest.approx(best.confidence)
    assert {"ra", "dec", "snr", "fwhm_px", "near_star", "persists", "confidence"} <= set(rows[0].keys())


def test_clean_field_has_no_confident_candidate(tmp_path):
    cam = FakeCamera(SKY, transient=None, rot_deg=11.0, flip_x=True, seed=4)
    frame = cam.capture(SKY.ra0, SKY.dec0, 8.0)
    res = Pipeline(csv_path=None).run(frame, SKY.catalog(SKY.ra0, SKY.dec0), second=cam.capture(SKY.ra0, SKY.dec0, 8.0))
    assert not res.transient_found
    assert all(c.confidence < config.CONFIDENCE_THRESHOLD for c in res.candidates)


def test_unregistered_reference_produces_false_positives(tmp_path):
    """The v1 failure, reproduced: draw the reference on a flat, unrotated grid and the field lights up."""
    cam = FakeCamera(SKY, transient=None, rot_deg=11.0, flip_x=True, seed=4)
    frame = cam.capture(SKY.ra0, SKY.dec0, 8.0)
    wrong = cam.wcs_for(SKY.ra0, SKY.dec0)
    wrong.rot_deg, wrong.flip_x = 0.0, False            # what image_subtraction.py assumed
    frame.wcs = wrong
    res = Pipeline(csv_path=None).run(frame, SKY.catalog(SKY.ra0, SKY.dec0))
    assert res.n_peaks > 20                             # residual pairs everywhere


def test_empty_catalog_is_a_named_fault():
    cam = FakeCamera(SKY, seed=4)
    frame = cam.capture(SKY.ra0, SKY.dec0, 8.0)
    with pytest.raises(Fault) as e:
        Pipeline(csv_path=None).run(frame, [])
    assert e.value.code == Code.DET_REFERENCE_EMPTY


def test_false_positive_rate_over_many_clean_fields():
    """Ten clean fields, two frames each, at different rotations and seeds: zero confident candidates."""
    hits = 0
    for seed in range(10):
        cam = FakeCamera(SKY, transient=None, rot_deg=3.0 * seed, flip_x=bool(seed % 2), seed=100 + seed)
        a = cam.capture(SKY.ra0 + seed, SKY.dec0 + 0.5 * seed, 4.0)
        b = cam.capture(SKY.ra0 + seed, SKY.dec0 + 0.5 * seed, 4.0)
        res = Pipeline(csv_path=None).run(a, SKY.catalog(SKY.ra0 + seed, SKY.dec0 + 0.5 * seed), second=b)
        hits += int(res.transient_found)
    assert hits == 0


def test_faint_transient_detection_curve():
    """Detection probability rises with flux; the threshold sits where SNR ~ 5-8 (traceable for the paper)."""
    found = {}
    for flux in (800.0, 2500.0, 8000.0):
        n = 0
        for seed in range(4):
            cam = FakeCamera(SKY, transient=(SKY.ra0 + 0.3, SKY.dec0 + 0.1, flux), rot_deg=5.0, seed=200 + seed)
            a = cam.capture(SKY.ra0, SKY.dec0, 4.0)
            b = cam.capture(SKY.ra0, SKY.dec0, 4.0)
            n += int(Pipeline(csv_path=None).run(a, SKY.catalog(SKY.ra0, SKY.dec0), second=b).transient_found)
        found[flux] = n / 4
    assert found[8000.0] == 1.0 and found[8000.0] >= found[2500.0] >= found[800.0]
