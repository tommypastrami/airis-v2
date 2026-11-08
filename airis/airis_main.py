"""The orchestrator. Decides where to look next and runs the loop; owns no hardware.

Two threads carry the science loop so the mount and the GPU work at the same time:

    acquire   pop the best tile -> point -> capture -> hand (tile, frame) to analyze
              -> mark the footprint imaged -> next tile. Bound by the mount.
    analyze   register -> subtract -> detect -> on a hit: downlink products and
              raise lock_on so acquire returns to the tile for photometry.

Every component is injected (mount client, camera, catalog, detector, downlink,
nav, scheduler), so the whole loop runs against the simulators in tests.
``step()`` runs one tile synchronously for deterministic tests; ``start()``
runs the two threads.
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable, Optional, Protocol

import numpy as np

from . import config
from .adapt_link import Envelope, Sender
from .detect import Frame, Pipeline, Star
from .downlink import Downlink, FULL, SLOW, STOP, products_for
from .errors import Code, Fault
from .nav import NavState
from .pointing import MountPos, SunBox, plan_route, radec_to_mount, sun_mount
from .scheduler import Tile, TileQueue

log = logging.getLogger("airis.main")


class MountLike(Protocol):
    def move(self, el: float, az: float): ...
    def calibrate(self): ...
    def set_sun(self, el: float, az: float): ...


class CameraLike(Protocol):
    def capture(self, tile_ra: float, tile_dec: float, exposure_s: float) -> Frame: ...
    def temperature_c(self) -> Optional[float]: ...


@dataclass
class Outcome:
    tile: Tile
    result: str                      # hit | miss | slew_failed | overheat | detect_failed
    code: str = "OK"
    n_candidates: int = 0
    best_confidence: float = 0.0
    duration_s: float = 0.0


class Orchestrator:
    def __init__(self, mount: MountLike, camera: CameraLike, catalog: Callable[[float, float], list[Star]],
                 detector: Pipeline, downlink: Downlink, sender: Sender, nav: NavState,
                 tiles: Optional[TileQueue] = None, now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
                 exposure_s: float = config.EXPOSURE_S, overheat_c: float = 60.0):
        self.mount, self.camera, self.catalog, self.detector = mount, camera, catalog, detector
        self.downlink, self.sender, self.nav = downlink, sender, nav
        self.tiles = tiles or TileQueue(exposure_s=exposure_s)
        self.now_utc, self.exposure_s, self.overheat_c = now_utc, exposure_s, overheat_c
        self.mount_pos = MountPos(0.0, 0.0)
        self.sun_box: Optional[SunBox] = None
        self.safe_mode = threading.Event()
        self.safe_mode_exit = threading.Event()
        self.lock_on: Optional[Tile] = None
        self.outcomes: list[Outcome] = []
        self.events: list[str] = []
        self.current_burst = 0
        self.refused: list[SunBox] = []      # firmware said sun-facing where our box did not: trust it
        self._analysis: "queue.Queue[tuple[Tile, Frame, Frame]]" = queue.Queue(maxsize=4)
        self._lock = threading.Lock()
        self.stop = threading.Event()
        self.threads: list[threading.Thread] = []

    # ---- inbound events ---------------------------------------------------------------
    def dispatch(self, env: Envelope) -> None:
        f = env.fields
        if env.kind == "gcn_alert":
            self._new_burst(int(f["burst_id"]), "gcn_alert")
            self.tiles.add_point(float(f["ra"]), float(f["dec"]), "adapt", int(f["burst_id"]))
            self._rescore()
            self.notify(f"GCN alert burst {f['burst_id']}")
        elif env.kind == "localization_update":
            bid, it = int(f["burst_id"]), int(f.get("iteration", 0))
            if bid == self.current_burst and it <= self.tiles.iteration:
                log.info("stale map iteration %d; ignoring", it)
                return
            self._new_burst(bid, "localization_update")
            grid = f["healpix_grid"]
            prob = np.frombuffer(grid, dtype=config.HEALPIX_DTYPE) if isinstance(grid, (bytes, bytearray)) else np.asarray(grid, dtype=float)
            try:
                self.tiles.set_map(prob, bid, it, bool(f.get("nested", config.HEALPIX_NESTED)))
            except Fault as e:
                log.error("%s", e)
                return
            self._rescore()
        elif env.kind == "comms_directive":
            mode = f.get("mode", FULL)
            mode = {0: FULL, 1: FULL, 2: SLOW, 3: STOP}.get(mode, mode) if isinstance(mode, int) else str(mode).upper().replace("COMMS_MODE_", "")
            self.downlink.set_mode(mode)
        elif env.kind == "navigation_update":
            self.nav.on_adapt(float(f.get("latitude", 0.0)), float(f.get("longitude", 0.0)),
                              float(f.get("altitude", 0.0)), int(f.get("flight_phase", 0)))
        elif env.kind == "safe_mode_exit_cmd":
            self.safe_mode_exit.set()
            self.safe_mode.clear()
            self.events.append("safe_mode_exit")
        elif env.kind == "pointing_cmd":
            el, az = float(f["target_elevation"]), float(f["target_azimuth"])
            self.events.append(f"pointing_cmd {el} {az}")
            self._point(MountPos(el, az))
        elif env.kind == "internal_message":
            log.info("internal_message [%s]: %s", f.get("heading"), f.get("message"))
        else:
            log.warning("unhandled envelope %s", env.kind)

    def _new_burst(self, burst_id: int, reason: str) -> None:
        if burst_id != self.current_burst:
            log.info("new burst %d via %s (was %d)", burst_id, reason, self.current_burst)
            self.current_burst = burst_id
            self.lock_on = None
            self.refused = []
            self.tiles.clear()

    # ---- sun and scheduling ---------------------------------------------------------------
    def update_sun(self) -> Optional[SunBox]:
        site, _ = self.nav.observer()
        try:
            center = sun_mount(site, self.now_utc(), self.nav.heading_deg())
        except Fault as e:
            log.warning("%s", e)
            return self.sun_box
        self.sun_box = SunBox(center)
        self.mount.set_sun(center.el, center.az)
        return self.sun_box

    def _rescore(self) -> int:
        site, src = self.nav.observer()
        return self.tiles.rescore(site, self.now_utc(), self.mount_pos, self.sun_box, self.nav.heading_deg(), self.refused)

    # ---- actions --------------------------------------------------------------------------
    def _point(self, target: MountPos) -> tuple[bool, str]:
        try:
            route = plan_route(self.mount_pos, target, self.sun_box)
        except Fault as e:
            log.warning("%s", e)
            return False, e.code.value
        for leg in route:
            r = self.mount.move(leg.el, leg.az)
            if not r.ok:
                log.error("slew failed: %s %s", r.code, r.msg)
                if r.code == Code.MNT_SUN.value:
                    log.warning("%s firmware sun box disagrees with ours; excluding around el=%.1f az=%.1f",
                                Code.PTG_SUN_RADIUS.value, leg.el, leg.az)
                    self.refused.append(SunBox(leg))
                return False, r.code
            self.mount_pos = leg
        return True, "OK"

    def notify(self, msg: str, kind: str = "event") -> None:
        self.sender.send(Envelope("internal_message", {"heading": kind, "message": msg}))

    def step(self) -> Optional[Outcome]:
        """One synchronous tile: pop -> point -> capture -> detect -> report. None if nothing queued."""
        if self.safe_mode.is_set():
            return None
        tile = self.lock_on or self.tiles.pop()
        if tile is None:
            return None
        t0 = time.monotonic()
        ok, code = self._point(MountPos(tile.el, tile.az))
        if not ok:
            self.tiles.mark_imaged(tile.ra, tile.dec)        # never retry a refused tile; move on
            self._rescore()
            out = Outcome(tile, "slew_failed", code, duration_s=time.monotonic() - t0)
            self.outcomes.append(out)
            return out
        temp = self.camera.temperature_c()
        if temp is not None and temp >= self.overheat_c:
            self.safe_mode.set()
            self.notify(f"{Code.CAM_OVERHEAT.value} sensor {temp:.1f} C; entering safe mode", "safe_mode_enter")
            out = Outcome(tile, "overheat", Code.CAM_OVERHEAT.value)
            self.outcomes.append(out)
            return out
        frame, second = self._capture_pair(tile)
        out = self._analyze(tile, frame, second)
        out.duration_s = time.monotonic() - t0
        self.tiles.mark_imaged(tile.ra, tile.dec)
        self._rescore()
        self.outcomes.append(out)
        return out

    def _capture_pair(self, tile: Tile) -> tuple[Frame, Frame]:
        """Two half-exposures: the second lets the vetting demand that a candidate persists."""
        a = self.camera.capture(tile.ra, tile.dec, self.exposure_s / 2)
        b = self.camera.capture(tile.ra, tile.dec, self.exposure_s / 2)
        for f in (a, b):
            f.tile_ra, f.tile_dec = tile.ra, tile.dec
        return a, b

    def _analyze(self, tile: Tile, frame: Frame, second: Optional[Frame] = None) -> Outcome:
        try:
            res = self.detector.run(frame, self.catalog(tile.ra, tile.dec), second=second, burst_id=self.current_burst)
        except Fault as e:
            log.error("%s", e)
            return Outcome(tile, "detect_failed", e.code.value)
        best = res.candidates[0].confidence if res.candidates else 0.0
        if res.transient_found:
            self.lock_on = tile
            self.notify(f"transient candidate burst {self.current_burst} conf={best:.2f}", "grb_confirmed")
            self.sender.send(Envelope("internal_message", {"heading": "attitude", "message": f"el={self.mount_pos.el:.2f} az={self.mount_pos.az:.2f}"}))
            prods = products_for(frame.data, res.candidates[:5], self.current_burst, (tile.ra, tile.dec),
                                 frame.utc.isoformat(), res.wcs.__dict__ if res.wcs else None)
            self.downlink.send_products(prods, self.current_burst)
            return Outcome(tile, "hit", "OK", len(res.candidates), best)
        return Outcome(tile, "miss", "OK", len(res.candidates), best)

    # ---- threaded mode ----------------------------------------------------------------------
    def start(self) -> None:
        self.threads = [threading.Thread(target=self._acquire_loop, daemon=True, name="acquire"),
                        threading.Thread(target=self._analyze_loop, daemon=True, name="analyze")]
        for t in self.threads:
            t.start()

    def _acquire_loop(self) -> None:
        while not self.stop.is_set():
            if self.safe_mode.is_set():
                time.sleep(0.05); continue
            tile = self.lock_on or self.tiles.pop()
            if tile is None:
                time.sleep(0.02); continue
            ok, code = self._point(MountPos(tile.el, tile.az))
            if not ok:
                self.tiles.mark_imaged(tile.ra, tile.dec); self._rescore()
                self.outcomes.append(Outcome(tile, "slew_failed", code)); continue
            temp = self.camera.temperature_c()
            if temp is not None and temp >= self.overheat_c:
                self.safe_mode.set(); self.outcomes.append(Outcome(tile, "overheat", Code.CAM_OVERHEAT.value)); continue
            frame, second = self._capture_pair(tile)
            self._analysis.put((tile, frame, second))
            self.tiles.mark_imaged(tile.ra, tile.dec); self._rescore()

    def _analyze_loop(self) -> None:
        while not self.stop.is_set():
            try:
                tile, frame, second = self._analysis.get(timeout=0.05)
            except queue.Empty:
                continue
            self.outcomes.append(self._analyze(tile, frame, second))

    def shutdown(self) -> None:
        self.stop.set()


def main() -> None:  # pragma: no cover - flight entry point
    import argparse
    from .adapt_link import JsonCodec, Listener
    from .bridge_client import MountClient
    p = argparse.ArgumentParser(description="AIRIS orchestrator v2")
    p.add_argument("--codec", choices=["json", "proto"], default="proto")
    a = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    codec = JsonCodec() if a.codec == "json" else __import__("airis.adapt_link", fromlist=["ProtoCodec"]).ProtoCodec()
    sender = Sender(codec)
    nav = NavState()
    import airis_photo_capture as camera_mod  # the camera module must provide capture()/temperature_c()
    orch = Orchestrator(MountClient(), camera_mod, camera_mod.catalog, Pipeline(), Downlink(sender), sender, nav)
    Listener(config.ADAPT_PORT_CMD, codec, orch.dispatch, timeout_s=config.ADAPT_RECV_TIMEOUT_CMD_S,
             max_bytes=config.ADAPT_MAX_ENVELOPE_CMD, sender=sender).start()
    Listener(config.ADAPT_PORT_NAV, codec, orch.dispatch, timeout_s=config.ADAPT_RECV_TIMEOUT_NAV_S,
             max_bytes=config.ADAPT_MAX_ENVELOPE_NAV, nav_port=True).start()
    threading.Thread(target=__import__("airis.nav", fromlist=["run_udp_listener"]).run_udp_listener, args=(nav,), daemon=True).start()
    orch.start()
    try:
        while True:
            orch.update_sun()
            time.sleep(config.SUN_PUSH_PERIOD_S)
    except KeyboardInterrupt:
        orch.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
