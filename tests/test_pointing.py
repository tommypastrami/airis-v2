"""One frame standard: wraps, heading, flip equivalence, sun box, routing, timing model."""
import math
from datetime import datetime, timezone

import pytest

from airis import config, pointing
from airis.errors import Code, Fault
from airis.pointing import MountPos, Site, SunBox, altaz_to_mount, plan_route, radec_to_altaz, slew_time_s, wrap180


def test_wrap180():
    assert wrap180(190) == -170 and wrap180(-190) == 170 and wrap180(180) == 180 and wrap180(0) == 0


def test_polaris_altitude_equals_latitude():
    site = Site(31.78, -95.63, 159.0)
    alt, az = radec_to_altaz(37.95, 89.26, site, datetime(2026, 12, 15, 6, 0, tzinfo=timezone.utc))
    assert abs(alt - site.lat_deg) < 1.0
    assert az < 2.0 or az > 358.0


def test_flip_is_same_boresight():
    p = MountPos(30.0, 170.0)
    f = p.flipped()
    assert f == MountPos(150.0, -10.0)
    assert f.flipped().el == pytest.approx(30.0) and f.flipped().az == pytest.approx(170.0)


def test_heading_rotates_azimuth_only_when_enabled(monkeypatch):
    monkeypatch.setattr(config, "HEADING_CORRECTION_ENABLED", False)
    assert altaz_to_mount(10.0, 90.0, heading_deg=45.0).az == 90.0
    monkeypatch.setattr(config, "HEADING_CORRECTION_ENABLED", True)
    assert altaz_to_mount(10.0, 90.0, heading_deg=45.0).az == 45.0
    with pytest.raises(Fault) as e:
        altaz_to_mount(10.0, 90.0, heading_deg=None)
    assert e.value.code == Code.PTG_HEADING_STALE


def test_sun_box_contains_both_attitudes():
    box = SunBox(MountPos(20.0, 100.0))
    assert box.contains(MountPos(20.0, 100.0))
    assert box.contains(MountPos(160.0, -80.0))          # the flipped attitude stares at the sun too
    assert not box.contains(MountPos(20.0, -100.0))


def test_route_goes_around_the_box():
    box = SunBox(MountPos(20.0, 0.0))
    start, target = MountPos(20.0, -60.0), MountPos(20.0, 60.0)
    route = plan_route(start, target, box, allow_flip=False)
    assert route[-1] == target and 2 <= len(route) <= 3
    pts = [start] + route
    for a, b in zip(pts, pts[1:]):
        assert not box.segment_crosses(a, b)


def test_route_is_direct_when_clear():
    box = SunBox(MountPos(-55.0, -35.0))
    assert plan_route(MountPos(0, 0), MountPos(60.0, 10.0), box) == [MountPos(60.0, 10.0)]


def test_target_inside_box_is_refused_with_named_code():
    box = SunBox(MountPos(20.0, 0.0))
    with pytest.raises(Fault) as e:
        plan_route(MountPos(0, -90), MountPos(20.0, 0.0), box)
    assert e.value.code == Code.PTG_SUN_RADIUS


def test_slew_time_model_matches_firmware_numbers():
    assert 3.0 <= slew_time_s(90.0) <= 4.5          # firmware: ~3.6 s
    assert 4.2 <= slew_time_s(180.0) <= 6.0         # firmware: ~4.9 s
    assert slew_time_s(0.0) < slew_time_s(10.0) < slew_time_s(90.0)
