"""Nav: typed parsing, quality gate, staleness fallback, heading."""
from airis import config
from airis.errors import Code
from airis.nav import NavState, nmea_checksum_ok, parse_sentence


def nmea(body: str) -> str:
    cs = 0
    for b in body.encode():
        cs ^= b
    return f"${body}*{cs:02X}"


GGA_OK = nmea("GNGGA,172815.00,3146.75800,N,09537.86640,W,4,12,0.7,159.0,M,-26.3,M,1.0,0000")
GGA_NOFIX = nmea("GNGGA,172816.00,0000.00000,N,00000.00000,W,0,00,99.9,0.0,M,0.0,M,,")
HDT = nmea("GPHDT,274.07,T")
RMC = nmea("GNRMC,172814.00,A,3146.75800,N,09537.86640,W,0.02,0.0,160726,0.0,E,D")


def test_checksum_rejects_damage():
    assert nmea_checksum_ok(GGA_OK)
    assert not nmea_checksum_ok(GGA_OK.replace("3146", "3156"))


def test_gga_fields_are_typed():
    s = parse_sentence(GGA_OK)
    assert s.kind == "GGA" and s.fields["quality"] == 4 and isinstance(s.fields["quality"], int)
    assert abs(s.fields["lat"] - 31.7793) < 1e-3 and abs(s.fields["lon"] + 95.6311) < 1e-3   # signed, west-negative
    assert s.fields["alt_msl_m"] == 159.0 and s.fields["geoid_sep_m"] == -26.3


def test_quality_gate_and_sources():
    t = [1000.0]
    pushes = []
    nav = NavState(push=lambda site, phase: pushes.append((site, phase)), clock=lambda: t[0])
    nav.handle(parse_sentence(RMC))
    nav.handle(parse_sentence(GGA_NOFIX))
    assert nav.fix is None and nav.rejected == 1
    assert nav.observer()[1] == Code.NAV_USING_DEFAULTS
    nav.handle(parse_sentence(GGA_OK))
    site, src = nav.observer()
    assert src == Code.NAV_TRIMBLE and abs(site.lat_deg - 31.7793) < 1e-3
    assert site.height_m == 159.0 - 26.3                     # ellipsoidal
    assert pushes and pushes[0][0].lat_deg == site.lat_deg
    assert nav.fix.utc is not None and nav.fix.utc.year == 2026
    nav.on_adapt(31.0, -95.0, 25000.0, 2)
    assert nav.observer()[1] == Code.NAV_TRIMBLE              # Trimble still fresh
    t[0] += config.TRIMBLE_STALE_S + 1
    site, src = nav.observer()
    assert src == Code.NAV_STALE_USING_ADAPT and site.height_m == 25000.0
    assert nav.flight_phase() == 2


def test_heading_staleness():
    t = [0.0]
    nav = NavState(clock=lambda: t[0])
    nav.handle(parse_sentence(HDT))
    assert nav.heading_deg() == 274.07
    t[0] += config.HEADING_STALE_S + 1
    assert nav.heading_deg() is None
