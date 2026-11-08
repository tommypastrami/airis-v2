"""Run the whole loop against the simulators and print a timeline.

    python -m sim.run_demo            # alert -> miss -> map -> hit, with a garbled link
"""
from __future__ import annotations

import logging
import os
import sys
import tempfile
import time
from datetime import datetime, timedelta, timezone

import numpy as np

from airis import config
from airis.adapt_link import Envelope, JsonCodec, Sender
from airis.airis_bridge import Bridge
from airis.airis_main import Orchestrator
from airis.detect import Pipeline
from airis.downlink import Downlink
from airis.nav import NavState, parse_sentence
from airis.pointing import Site, radec_to_altaz
from airis.scheduler import TileQueue
from airis.transport import SocketTransport
from sim.fake_arduino import Faults, make_pair
from sim.sky_sim import FakeCamera, SkyModel

try:
    import healpy as hp
except ImportError:
    print("healpy is required for the demo"); sys.exit(1)


class PrintingSender(Sender):
    def __init__(self):
        super().__init__(JsonCodec()); self.n = 0

    def connected(self): return True

    def send(self, env):
        self.n += 1
        if env.kind == "internal_message":
            h = env.fields.get("heading")
            if h != "image_manifest":
                print(f"   -> ground: {h}: {env.fields.get('message')[:70]}")
        return True


def gaussian_map(nside, ra0, dec0, sigma):
    npix = hp.nside2npix(nside)
    vec = hp.pix2vec(nside, np.arange(npix))
    v0 = hp.ang2vec(np.radians(90 - dec0), np.radians(ra0))
    d = np.degrees(np.arccos(np.clip(np.array(vec).T @ v0, -1, 1)))
    p = np.exp(-0.5 * (d / sigma) ** 2)
    return p / p.sum()


def main():
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s %(message)s")
    for k, v in dict(ACK_TIMEOUT_S=0.3, MOVE_START_TIMEOUT_S=1.0, MOVE_MARGIN_S=1.0, MOVE_MAX_S=3.0,
                     CAL_START_TIMEOUT_S=1.0, CAL_FINISH_TIMEOUT_S=5.0, LINK_SILENCE_S=1.5, KEEPALIVE_PERIOD_S=0.6).items():
        setattr(config, k, v)
    tmp = tempfile.mkdtemp()
    site = Site(31.78, -95.63, 159.0)
    sky = SkyModel(seed=9, ra0=83.8, dec0=-5.4)
    true = (sky.ra0 + 0.6, sky.dec0 + 0.2)
    when = datetime(2026, 12, 15, 0, 0, tzinfo=timezone.utc)
    while radec_to_altaz(sky.ra0, sky.dec0, site, when)[0] < 45:
        when += timedelta(hours=1)

    host, fa = make_pair(faults=Faults(garble_after_idle_s=2.0, garble_bytes=3))   # USB-wake garbling ON
    bridge = Bridge(SocketTransport(host), moves_csv=os.path.join(tmp, "moves.csv"))
    print(f"[t=0.0s] bridge: calibrating (sim at 25x)")
    t0 = time.monotonic()
    r = bridge.calibrate()
    print(f"[t={time.monotonic()-t0:4.1f}s] cal {r.code}: {r.msg}  encoder offsets now {fa.mount.el.enc_offset:+.2f} {fa.mount.az.enc_offset:+.2f}")

    cam = FakeCamera(sky, transient=(true[0], true[1], 7000.0), rot_deg=9.0, seed=8)
    nav = NavState(); nav.on_adapt(site.lat_deg, site.lon_deg, site.height_m, 2)
    sender = PrintingSender()
    orch = Orchestrator(bridge, cam, sky.catalog, Pipeline(csv_path=os.path.join(tmp, "candidates.csv")),
                        Downlink(sender, queue_dir=os.path.join(tmp, "q")), sender, nav,
                        tiles=TileQueue(n_candidates=1500), now_utc=lambda: when)
    box = orch.update_sun()
    print(f"[t={time.monotonic()-t0:4.1f}s] sun pushed to firmware: el={box.center.el:+.1f} az={box.center.az:+.1f} (below horizon tonight)")

    print(f"[t={time.monotonic()-t0:4.1f}s] ADAPT: gcn_alert burst 42 at ra={true[0]+8:.1f} dec={true[1]:.1f} (8 deg off)")
    orch.dispatch(Envelope("gcn_alert", {"burst_id": 42, "ra": true[0] + 8.0, "dec": true[1], "probability_score": 0.6}))
    time.sleep(0.15)                                           # idle gap so the next command is garbled on wake
    out = orch.step()
    print(f"[t={time.monotonic()-t0:4.1f}s] tile ra={out.tile.ra:.2f} dec={out.tile.dec:.2f} -> el={out.tile.el:.1f} az={out.tile.az:.1f}: {out.result} "
          f"({out.n_candidates} peaks, best conf {out.best_confidence:.2f}); firmware NAKs so far: {fa.naks}, moves executed: {len(fa.moves_executed)}")

    print(f"[t={time.monotonic()-t0:4.1f}s] ADAPT: localization_update burst 42 iteration 1 (sigma 3 deg around the truth)")
    orch.dispatch(Envelope("localization_update", {"burst_id": 42, "iteration": 1,
                                                   "healpix_grid": gaussian_map(32, true[0], true[1], 3.0).astype("<f8").tobytes()}))
    print(f"      queue: {len(orch.tiles)} tiles; top score {orch.tiles.peek().score:.4f} p_fov={orch.tiles.peek().p_fov:.2f} t_slew={orch.tiles.peek().t_slew:.1f}s")
    out = orch.step()
    c = out
    print(f"[t={time.monotonic()-t0:4.1f}s] tile ra={out.tile.ra:.2f} dec={out.tile.dec:.2f}: {out.result} (best conf {out.best_confidence:.2f}); lock-on={'yes' if orch.lock_on else 'no'}; "
          f"products sent: {sender.n} envelopes")
    print(f"      moves.csv rows: {sum(1 for _ in open(bridge.moves_csv)) - 1}, candidates.csv rows: {sum(1 for _ in open(os.path.join(tmp, 'candidates.csv'))) - 1}")
    print(f"      mount physical position: el={fa.mount.el.pos:+.2f} az={fa.mount.az.pos:+.2f}; commanded {orch.mount_pos.el:+.2f} {orch.mount_pos.az:+.2f}")
    orch.shutdown(); bridge.close(); fa.shutdown()
    print(f"[t={time.monotonic()-t0:4.1f}s] done; files in {tmp}")


if __name__ == "__main__":
    main()
