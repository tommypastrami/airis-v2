import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import pytest  # noqa: E402

from airis import config  # noqa: E402
from airis.airis_bridge import Bridge  # noqa: E402
from airis.transport import SocketTransport  # noqa: E402
from sim.fake_arduino import Faults, make_pair  # noqa: E402
from sim.mount_sim import MountSim  # noqa: E402


@pytest.fixture
def fast_link(monkeypatch):
    """Shorten bridge timeouts so a test suite runs in seconds (sim runs at 25x)."""
    monkeypatch.setattr(config, "ACK_TIMEOUT_S", 0.3)
    monkeypatch.setattr(config, "MOVE_START_TIMEOUT_S", 1.0)
    monkeypatch.setattr(config, "MOVE_MARGIN_S", 1.0)
    monkeypatch.setattr(config, "MOVE_MAX_S", 3.0)
    monkeypatch.setattr(config, "CAL_START_TIMEOUT_S", 1.0)
    monkeypatch.setattr(config, "CAL_FINISH_TIMEOUT_S", 5.0)
    monkeypatch.setattr(config, "LINK_SILENCE_S", 1.5)
    monkeypatch.setattr(config, "KEEPALIVE_PERIOD_S", 0.6)
    yield


def make_bridge(tmp_path, faults=None, mount=None, **kw):
    host_sock, fa = make_pair(faults=faults, mount=mount, **kw)
    bridge = Bridge(SocketTransport(host_sock), moves_csv=str(tmp_path / "moves.csv"))
    return bridge, fa


@pytest.fixture
def bridge_and_fw(tmp_path, fast_link):
    bridge, fa = make_bridge(tmp_path)
    yield bridge, fa
    bridge.close()
    fa.shutdown()
