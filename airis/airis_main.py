"""The orchestrator."""
from __future__ import annotations

import logging
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Optional, Protocol

import numpy as np

from . import config
from .adapt_link import Envelope
from .detect import Frame, Pipeline, Star
from .downlink import Downlink, FULL, SLOW, STOP, products_for
from .errors import Code, Fault
from .nav import NavState
from .payload_hw import PHASE_ASCENT, PHASE_DESCENT, PHASE_FLOAT, PHASE_UNKNOWN
from .pointing import MountPos, SunBox, radec_to_mount, sun_mount
from .scheduler import Tile, TileQueue

log = logging.getLogger("airis.main")

# The tile itself cannot be pointed at: retire it at once.
_RETIRE = {Code.MNT_SUN.value, Code.MNT_ANGLE.value, Code.PTG_BELOW_HORIZON.value,
           Code.PTG_SUN_RADIUS.value, Code.PTG_RANGE.value}
_MOUNT_UNAVAILABLE = {Code.MNT_SUN_UNKNOWN.value, Code.MNT_BUSY.value, Code.MNT_NOT_HOMED.value,
                      Code.MNT_FAULT_LATCHED.value, Code.MNT_RESET.value, Code.PTG_HEADING_STALE.value,
                      Code.MNT_HOME_M1.value, Code.MNT_HOME_M2.value, Code.MNT_CATCH_M1.value,
                      Code.MNT_CATCH_M2.value, Code.MNT_LIMIT_M1.value, Code.MNT_LIMIT_M2.value}

_EVENT_TYPES = {"startup": "EVENT_STARTUP", "safe_mode_enter": "EVENT_SAFE_MODE_ENTER", "error": "EVENT_ERROR"}
_FLIGHT_PHASES = {"FLIGHT_PHASE_ASCENT": PHASE_ASCENT, "FLIGHT_PHASE_FLOAT": PHASE_FLOAT,
                  "FLIGHT_PHASE_DESCENT": PHASE_DESCENT}


def _mount_unavailable(code: str) -> bool:
    return code.startswith("LNK-") or code in _MOUNT_UNAVAILABLE


def _comms_mode(value) -> Optional[str]:
    """CommsDirective.mode as a name (MODE_FULL_UNSPECIFIED / MODE_SLOW / MODE_STOP) or its number (0 / 1 / 2)."""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return {0: FULL, 1: SLOW, 2: STOP}.get(value)
    s = str(value).upper()
    for prefix in ("COMMS_MODE_", "MODE_"):
        if s.startswith(prefix):
            s = s[len(prefix):]
            break
    s = s.replace("_UNSPECIFIED", "")
    return s if s in (FULL, SLOW, STOP) else None


def _flight_phase(value) -> int:
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    return _FLIGHT_PHASES.get(str(value).upper(), PHASE_UNKNOWN)


class MountLike(Protocol):
    def move(self, el: float, az: float): ...
    def calibrate(self): ...
    def set_sun(self, el: float, az: float): ...


class CameraLike(Protocol):
    def capture(self, tile_ra: float, tile_dec: float, exposure_s: float) -> Frame: ...
    def temperature_c(self) -> Optional[float]: ...


class SenderLike(Protocol):
    def send(self, env: Envelope) -> bool: ...


@dataclass
class Outcome:
    tile: Tile
    result: str                      # hit | miss | slew_failed | slew_retry | overheat | detect_failed
    code: str = "OK"
    n_candidates: int = 0
    best_confidence: float = 0.0
    duration_s: float = 0.0


@dataclass
class _Capture:
    tile: Tile
    frame: Frame
    second: Frame
    attitude: MountPos               # where the mount was when the frames were taken
    revisit: bool                    # a lock-on revisit: must not start another lock-on


class Orchestrator:
    def __init__(self, mount: MountLike, camera: CameraLike, catalog: Callable[[float, float], list[Star]],
                 detector: Pipeline, downlink: Downlink, sender: SenderLike, nav: NavState,
                 tiles: Optional[TileQueue] = None, now_utc: Optional[Callable[[], datetime]] = None,
                 exposure_s: float = config.EXPOSURE_S, overheat_c: float = config.OVERHEAT_C):
        self.mount, self.camera, self.catalog, self.detector = mount, camera, catalog, detector
        self.downlink, self.sender, self.nav = downlink, sender, nav
        self.tiles = tiles or TileQueue(exposure_s=exposure_s)
        self.now_utc, self.exposure_s, self.overheat_c = now_utc, exposure_s, overheat_c
        self.mount_pos = MountPos(0.0, 0.0)
        self.sun_box: Optional[SunBox] = None
        self.sun_pushed = False              # the mount confirmed the last sun push
        self.safe_mode = threading.Event()
        self.safe_mode_exit = threading.Event()
        self.lock_on: Optional[Tile] = None
        self.outcomes: "deque[Outcome]" = deque(maxlen=5000)
        self.events: "deque[str]" = deque(maxlen=1000)
        self.current_burst = 0
        self.refused: list[SunBox] = []      # firmware said sun-facing where our box did not: trust it
        self._analysis: "queue.Queue[_Capture]" = queue.Queue(maxsize=4)
        self._lock = threading.RLock()       # tiles, lock_on, refused, current_burst, manual, park
        self._manual: Optional[MountPos] = None
        self._park_pending = False
        self._lock_on_left = 0
        self._retries: dict[tuple[float, float], int] = {}
        self._nav_src: Optional[Code] = None
        self._last_idle_rescore = float("-inf")
        self.stop = threading.Event()
        self.threads: list[threading.Thread] = []

    # ---- time -----------------------------------------------------------------------------
    def _now(self) -> datetime:
        """UTC for pointing and the sun: GPS time while the fix is fresh, the system clock otherwise."""
        if self.now_utc is not None:
            return self.now_utc()
        return self.nav.gps_utc() or datetime.now(timezone.utc)

    # ---- inbound events ---------------------------------------------------------------
    def dispatch(self, env: Envelope) -> None:
        with self._lock:
            self._dispatch_locked(env)

    def _dispatch_locked(self, env: Envelope) -> None:
        f = env.fields
        if env.kind == "gcn_alert":
            bid = int(f.get("burst_id", 0))
            self._new_burst(bid, "gcn_alert")
            self.tiles.add_point(float(f["ra"]), float(f["dec"]), "adapt", bid)
            self._rescore()
            self.notify(f"GCN alert burst {bid}")
        elif env.kind == "localization_update":
            bid, it = int(f.get("burst_id", 0)), int(f.get("iteration", 0))
            if bid == self.current_burst and it <= self.tiles.iteration:
                log.info("stale map iteration %d; ignoring", it)
                return
            grid = f["healpix_grid"]
            try:
                prob = np.frombuffer(grid, dtype=config.HEALPIX_DTYPE) if isinstance(grid, (bytes, bytearray)) else np.asarray(grid, dtype=float)
                # set_map validates first and only then clears the previous burst's queue
                self.tiles.set_map(prob, bid, it, bool(f.get("nested", config.HEALPIX_NESTED)))
            except Fault as e:
                log.error("%s", e)
                return
            except ValueError as e:
                log.error("%s map for burst %d rejected: %s", Code.SCH_BAD_MAP.value, bid, e)
                return
            self._new_burst(bid, "localization_update", clear_tiles=False)
            self._rescore()
        elif env.kind == "comms_directive":
            mode = _comms_mode(f.get("mode", 0))
            if mode is None:
                log.warning("comms_directive with unknown mode %r ignored", f.get("mode"))
                return
            self.downlink.set_mode(mode)
        elif env.kind == "navigation_update":
            self.nav.on_adapt(float(f.get("latitude", 0.0)), float(f.get("longitude", 0.0)),
                              float(f.get("altitude", 0.0)), _flight_phase(f.get("flight_phase", 0)))
        elif env.kind == "safe_mode_exit_cmd":
            self.safe_mode_exit.set()
            self.safe_mode.clear()
            self._park_pending = False
            self.events.append("safe_mode_exit")
        elif env.kind == "pointing_cmd":
            el, az = float(f.get("target_elevation", 0.0)), float(f.get("target_azimuth", 0.0))
            self.events.append(f"pointing_cmd {el} {az}")
            self._manual = MountPos(el, az)          # one slew, done by the acquire thread
        else:
            log.warning("unhandled envelope %s", env.kind)

    def _new_burst(self, burst_id: int, reason: str, clear_tiles: bool = True) -> None:
        if burst_id != self.current_burst:
            log.info("new burst %d via %s (was %d)", burst_id, reason, self.current_burst)
            self.current_burst = burst_id
            self.lock_on = None
            self.refused = []
            self._retries = {}
            if clear_tiles:
                self.tiles.clear()

    # ---- sun and scheduling ---------------------------------------------------------------
    def update_sun(self) -> Optional[SunBox]:
        site, src = self.nav.observer()
        if src != self._nav_src:
            log.warning("%s observer position source is now %s", src.value, src.name)
            self._nav_src = src
        if self.now_utc is None:
            gps = self.nav.gps_utc()
            if gps is not None:
                off = (datetime.now(timezone.utc) - gps).total_seconds()
                if abs(off) > config.GPS_CLOCK_WARN_S:
                    log.warning("system clock is %+.1f s from GPS UTC; pointing uses GPS time", off)
        try:
            center = sun_mount(site, self._now(), self.nav.heading_deg())
        except Fault as e:
            log.warning("%s; sun box not updated", e)
            return self.sun_box
        self.sun_box = SunBox(center)
        r = self.mount.set_sun(center.el, center.az)
        self.sun_pushed = bool(getattr(r, "ok", r))
        if not self.sun_pushed:
            log.warning("sun push el=%.1f az=%.1f was not confirmed by the mount", center.el, center.az)
        return self.sun_box

    def _rescore(self) -> int:
        with self._lock:
            site, _ = self.nav.observer()
            try:
                return self.tiles.rescore(site, self._now(), self.mount_pos, self.sun_box, self.nav.heading_deg(), self.refused)
            except Fault as e:                               # e.g. heading stale with heading correction on
                log.warning("%s; queue not rescored (retried while idle)", e)
                return 0

    # ---- actions --------------------------------------------------------------------------
    def _point(self, target: MountPos) -> tuple[bool, str]:
        """One MOVE. The firmware routes around the sun, flips and enforces its azimuth range."""
        if not target.in_range():
            return False, Code.PTG_RANGE.value
        try:
            r = self.mount.move(target.el, target.az)
        except Fault as e:
            log.error("slew failed: %s", e)
            return False, e.code.value
        if not r.ok:
            log.error("slew failed: %s %s", r.code, r.msg)
            return False, r.code
        self.mount_pos = target
        return True, "OK"

    def notify(self, msg: str, kind: str = "event") -> None:
        """An event for the ground, as the event_notification the comms server forwards."""
        self.sender.send(Envelope("event_notification", {
            "event_type": _EVENT_TYPES.get(kind, "EVENT_UNSPECIFIED"),
            "timestamp": time.time(), "detail": f"{kind}: {msg}"}))

    def enter_safe_mode(self, reason: str, code: str = Code.CAM_OVERHEAT.value) -> None:
        """Stop imaging, park the mount once, tell the ground. Idempotent."""
        with self._lock:
            if self.safe_mode.is_set():
                return
            self.safe_mode.set()
            self.safe_mode_exit.clear()
            self._park_pending = True
        log.error("%s %s; entering safe mode", code, reason)
        self.notify(f"{code} {reason}; entering safe mode", "safe_mode_enter")

    def _do_manual(self) -> bool:
        with self._lock:
            target, self._manual = self._manual, None
        if target is None:
            return False
        ok, code = self._point(target)
        log.info("pointing_cmd el=%.2f az=%.2f: %s", target.el, target.az, code)
        if not ok:
            self.notify(f"pointing_cmd el={target.el:.2f} az={target.az:.2f} failed: {code}", "error")
        return True

    def _park(self) -> None:
        with self._lock:
            if not self._park_pending:
                return
        target = MountPos(config.SAFE_PARK_EL_DEG, config.SAFE_PARK_AZ_DEG)
        ok, code = self._point(target)
        if not ok and _mount_unavailable(code):
            self.stop.wait(config.SLEW_RETRY_BACKOFF_S)      # the mount is busy or down: try again
            return
        with self._lock:
            self._park_pending = False
        if not ok:
            self.notify(f"safe-mode park el={target.el:.1f} az={target.az:.1f} failed: {code}", "error")

    def _slew_failed(self, tile: Tile, code: str) -> Outcome:
        key = (tile.ra, tile.dec)
        with self._lock:
            if code in _RETIRE:
                retire = True
            elif _mount_unavailable(code):
                retire = False
            else:                                            # e.g. MNT-58 settle: may be this pointing
                self._retries[key] = self._retries.get(key, 0) + 1
                retire = self._retries[key] >= config.SLEW_MAX_RETRIES
            if retire:
                if code == Code.MNT_SUN.value:
                    log.warning("%s firmware refused el=%.1f az=%.1f as sun-facing; excluding around it",
                                Code.PTG_SUN_RADIUS.value, tile.el, tile.az)
                    self.refused.append(SunBox(MountPos(tile.el, tile.az)))
                self.tiles.mark_imaged(tile.ra, tile.dec)
                self._retries.pop(key, None)
                if self.lock_on is tile:
                    self.lock_on = None
            self._rescore()                                  # a tile that is not retired goes back in the heap
        return Outcome(tile, "slew_failed" if retire else "slew_retry", code)

    def _acquire_tile(self):
        """Manual slew, park, or one tile: pop -> point -> capture."""
        if self._do_manual():
            return None
        if self.safe_mode.is_set():
            self._park()
            return None
        with self._lock:
            revisit = self.lock_on is not None
            tile = self.lock_on or self.tiles.pop()
            if tile is None and time.monotonic() - self._last_idle_rescore >= config.IDLE_RESCORE_PERIOD_S:
                # Nothing queued: the sky, the sun and the heading move, and a rescore
                # that failed (stale heading) left the heap empty. Look again now and then.
                self._last_idle_rescore = time.monotonic()
                if self._rescore():
                    tile = self.tiles.pop()
        if tile is None:
            return None
        try:
            # el/az as of NOW: the tile's own values date from the last rescore
            site, _ = self.nav.observer()
            target = radec_to_mount(tile.ra, tile.dec, site, self._now(), self.nav.heading_deg())
        except Fault as e:
            log.warning("%s", e)
            return self._slew_failed(tile, e.code.value)
        tile.el, tile.az = target.el, target.az
        ok, code = self._point(target)
        if not ok:
            return self._slew_failed(tile, code)
        self._retries.pop((tile.ra, tile.dec), None)
        temp = self.camera.temperature_c()
        if temp is not None and temp >= self.overheat_c:
            self.enter_safe_mode(f"sensor {temp:.1f} C")
            return Outcome(tile, "overheat", Code.CAM_OVERHEAT.value)
        frame, second = self._capture_pair(tile)
        cap = _Capture(tile, frame, second, self.mount_pos, revisit)
        with self._lock:
            if revisit and self.lock_on is tile:
                self._lock_on_left -= 1
                if self._lock_on_left <= 0:
                    self.lock_on = None
            self.tiles.mark_imaged(tile.ra, tile.dec)
            self._rescore()
        return cap

    def step(self) -> Optional[Outcome]:
        """One synchronous pass: (manual slew | park | tile -> capture -> detect -> report). None if nothing was imaged."""
        t0 = time.monotonic()
        r = self._acquire_tile()
        if r is None:
            return None
        if isinstance(r, Outcome):
            r.duration_s = time.monotonic() - t0
            self.outcomes.append(r)
            return r
        out = self._analyze(r)
        out.duration_s = time.monotonic() - t0
        self.outcomes.append(out)
        return out

    def _capture_pair(self, tile: Tile) -> tuple[Frame, Frame]:
        """Two half-exposures: the second lets the vetting demand that a candidate persists."""
        a = self.camera.capture(tile.ra, tile.dec, self.exposure_s / 2)
        b = self.camera.capture(tile.ra, tile.dec, self.exposure_s / 2)
        for f in (a, b):
            f.tile_ra, f.tile_dec = tile.ra, tile.dec
        return a, b

    def _analyze(self, cap: _Capture) -> Outcome:
        tile, frame = cap.tile, cap.frame
        try:
            res = self.detector.run(frame, self.catalog(tile.ra, tile.dec), second=cap.second, burst_id=self.current_burst)
            best = res.candidates[0].confidence if res.candidates else 0.0
            if not res.transient_found:
                return Outcome(tile, "miss", "OK", len(res.candidates), best)
            with self._lock:
                if not cap.revisit and self.lock_on is None:
                    self.lock_on, self._lock_on_left = tile, config.LOCK_ON_REVISITS
            self.notify(f"transient candidate burst {self.current_burst} conf={best:.2f} "
                        f"el={cap.attitude.el:.2f} az={cap.attitude.az:.2f}", "grb_confirmed")
            prods = products_for(frame.data, res.candidates[:5], self.current_burst, (tile.ra, tile.dec),
                                 frame.utc.isoformat(), res.wcs.__dict__ if res.wcs else None)
            self.downlink.send_products(prods, self.current_burst)
            return Outcome(tile, "hit", "OK", len(res.candidates), best)
        except Fault as e:
            log.error("%s", e)
            return Outcome(tile, "detect_failed", e.code.value)
        except Exception:                                    # e.g. a full disk under candidates.csv
            log.exception("%s analysis of tile ra=%.2f dec=%.2f failed", Code.DET_SUBTRACTION.value, tile.ra, tile.dec)
            return Outcome(tile, "detect_failed", Code.DET_SUBTRACTION.value)

    # ---- threaded mode ----------------------------------------------------------------------
    def start(self) -> None:
        self.threads = [threading.Thread(target=self._acquire_loop, daemon=True, name="acquire"),
                        threading.Thread(target=self._analyze_loop, daemon=True, name="analyze")]
        for t in self.threads:
            t.start()

    def _acquire_loop(self) -> None:
        while not self.stop.is_set():
            try:
                r = self._acquire_tile()
                if r is None:
                    self.stop.wait(0.05)
                elif isinstance(r, Outcome):
                    self.outcomes.append(r)
                    if r.result == "slew_retry":
                        self.stop.wait(config.SLEW_RETRY_BACKOFF_S)
                else:
                    while not self.stop.is_set():            # back-pressure, but never block shutdown
                        try:
                            self._analysis.put(r, timeout=0.5)
                            break
                        except queue.Full:
                            continue
            except Exception:
                log.exception("acquire loop error; continuing")
                self.stop.wait(config.LOOP_ERROR_BACKOFF_S)

    def _analyze_loop(self) -> None:
        while not self.stop.is_set():
            try:
                cap = self._analysis.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self.outcomes.append(self._analyze(cap))
            except Exception:
                log.exception("analyze loop error; continuing")
                self.stop.wait(config.LOOP_ERROR_BACKOFF_S)

    def threads_alive(self) -> bool:
        return bool(self.threads) and all(t.is_alive() for t in self.threads)

    def shutdown(self) -> None:
        self.stop.set()


def main() -> None:  # pragma: no cover - flight entry point
    import argparse
    from . import nav as nav_mod
    from .adapt_link import JsonCodec, Listener, ProtoCodec, UdpSender, run_udp_envelope_listener
    from .bridge_client import MountClient
    from .payload_hw import ThermalWatchdog
    p = argparse.ArgumentParser(description="AIRIS orchestrator v2")
    p.add_argument("--codec", choices=["json", "proto"], default="proto")
    a = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    codec = JsonCodec() if a.codec == "json" else ProtoCodec()
    sender = UdpSender(codec)                 # downlink goes through the airis-comms server (UDP 9003)
    nav = NavState()
    import airis_photo_capture as camera_mod  # the camera module must provide capture()/temperature_c()
    orch = Orchestrator(MountClient(), camera_mod, camera_mod.catalog, Pipeline(), Downlink(sender), sender, nav)

    # Direct from ADAPT (framed TCP): alerts, maps, comms directives, navigation.
    Listener(config.ADAPT_PORT_CMD, codec, orch.dispatch, timeout_s=config.ADAPT_RECV_TIMEOUT_CMD_S,
             max_bytes=config.ADAPT_MAX_ENVELOPE_CMD).start()
    Listener(config.ADAPT_PORT_NAV, codec, orch.dispatch, timeout_s=config.ADAPT_RECV_TIMEOUT_NAV_S,
             max_bytes=config.ADAPT_MAX_ENVELOPE_NAV, nav_port=True).start()
    # Relayed by the airis-comms server (UDP 9001): pointing_cmd, safe_mode_exit_cmd.
    threading.Thread(target=run_udp_envelope_listener, args=(config.COMMS_PORT_COMMANDS, codec, orch.dispatch),
                     daemon=True, name="comms-commands").start()
    threading.Thread(target=nav_mod.run_udp_listener, args=(nav,), daemon=True, name="trimble").start()

    watchdog = ThermalWatchdog(read_camera=camera_mod.temperature_c, flight_phase=nav.flight_phase,
                               shutdown=lambda: orch.enter_safe_mode("Jetson over the thermal limit"))
    log.warning("thermal watchdog running WITHOUT the sensor RTD and heater output (not wired in main())")
    threading.Thread(target=watchdog.run, args=(orch.stop,), daemon=True, name="thermal").start()

    orch.notify("controls started", "startup")
    orch.update_sun()                         # the mount needs a sun position before its first move or CAL
    orch.start()
    try:
        while True:
            # the bridge will not home without a sun position: retry a failed push soon
            time.sleep(config.SUN_PUSH_PERIOD_S if orch.sun_pushed else config.SLEW_RETRY_BACKOFF_S)
            orch.update_sun()
            if not orch.threads_alive():
                log.critical("a science thread has stopped: %s",
                             ", ".join(f"{t.name}={'up' if t.is_alive() else 'DOWN'}" for t in orch.threads))
    except KeyboardInterrupt:
        orch.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
