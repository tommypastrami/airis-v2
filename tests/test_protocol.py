"""Protocol v5: a damaged line is rejected, never misparsed; replies parse; seq wraps."""
import pytest

from airis import protocol
from airis.protocol import ProtocolError, encode, parse_fw_line, parse_host_line


def test_roundtrip():
    line = encode("MOVE 60.000 0.000", 17).decode()
    hl = parse_host_line(line)
    assert hl.cmd == "MOVE" and hl.args == ["60.000", "0.000"] and hl.seq == 17


def test_lost_character_is_detected_not_executed():
    """The v4 failure: MOVE 60.000 minus one byte is a legal MOVE 6.000. Not any more."""
    good = encode("MOVE 60.000 0.000", 5).decode()
    damaged = good.replace("60.000", "6.000", 1)
    with pytest.raises(ProtocolError):
        parse_host_line(damaged)


def test_leading_garble_is_detected():
    good = encode("MOVE 60.000 0.000", 5).decode()
    with pytest.raises(ProtocolError):
        parse_host_line(good[2:])           # "VE 60.000 0.000 #5*xx" after USB-wake byte loss


def test_bad_checksum_field():
    with pytest.raises(ProtocolError):
        parse_host_line("MOVE 1 2 #3*ZZ")
    with pytest.raises(ProtocolError):
        parse_host_line("MOVE 1 2")


def test_seq_range():
    with pytest.raises(ValueError):
        encode("STATUS", 256)
    assert parse_host_line(encode("STATUS", 255).decode()).seq == 255


@pytest.mark.parametrize("line,kind,code", [
    ("ACK 17", "ACK", 17),
    ("NAK 17 checksum", "NAK", 17),
    ("STATUS 15 SLEWING", "STATUS", 15),
    ("ERR 63 busy", "ERR", 63),
    ("DONE 0.13 -0.12", "DONE", None),
    ("POS 59.87 0.12", "POS", None),
    ("ZERO SET", "ZERO", None),
    ("SUN OK -12.000 140.000", "SUN", None),
    ("RX: MOVE 1 2 #3*4A", "RX", None),
    ("HOLD TIMEOUT: drivers released", "OTHER", None),
    ("STATUS x", "OTHER", None),
])
def test_parse_fw_line(line, kind, code):
    m = parse_fw_line(line)
    assert m.kind == kind and m.code == code


def test_done_carries_residuals():
    m = parse_fw_line("DONE 0.13 -0.12")
    assert m.args == (0.13, -0.12)


def test_status_constants():
    assert protocol.STATUS_SLEWING == 15 and 13 in protocol.STATUS_HOMING
