"""Payload hardware: RTD thermometers, the heater policy, Jetson thermal zones, the watchdog."""
from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from . import config
from .errors import Code

log = logging.getLogger("airis.hw")

PHASE_UNKNOWN, PHASE_ASCENT, PHASE_FLOAT, PHASE_DESCENT = 0, 1, 2, 3


@dataclass
class HeaterDecision:
    on: bool
    reason: str


def heater_policy(sensor_c: Optional[float], flight_phase: Optional[int], duty_tick: int = 0,
                  was_on: bool = False) -> HeaterDecision:
    if sensor_c is None:
        if flight_phase == PHASE_ASCENT:
            return HeaterDecision(duty_tick % 2 == 0, "RTD unreadable during ascent: 50% duty")
        return HeaterDecision(False, "RTD unreadable: heater off")
    if sensor_c < config.SENSOR_HEAT_ON_C:
        return HeaterDecision(True, f"sensor {sensor_c:.1f} C below {config.SENSOR_HEAT_ON_C} C")
    if was_on and sensor_c < config.SENSOR_HEAT_OFF_C:     # hysteresis: no relay chatter at the threshold
        return HeaterDecision(True, f"sensor {sensor_c:.1f} C, holding on until {config.SENSOR_HEAT_OFF_C} C")
    return HeaterDecision(False, f"sensor {sensor_c:.1f} C ok")


class MAX31865:  # pragma: no cover - hardware
    """Minimal MAX31865 driver over spidev (PT100, 430 ohm reference)."""
    _CFG_RUN = 0b11000001

    def __init__(self, bus: int, device: int, ref_ohm: float = 430.0, r0: float = 100.0):
        import spidev
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 500_000
        self.spi.mode = 0b01
        self.spi.xfer2([0x80, self._CFG_RUN])
        self.ref_ohm, self.r0 = ref_ohm, r0
        time.sleep(0.1)

    def read_c(self) -> float:
        d = self.spi.xfer2([0x01, 0, 0])[1:]
        raw = ((d[0] << 8) | d[1]) >> 1
        if d[1] & 1:
            # The fault latches in the chip until it is cleared (config bit 1);
            # left latched, every later read would fail too.
            self.spi.xfer2([0x80, self._CFG_RUN | 0b10])
            raise RuntimeError("MAX31865 fault")
        r = raw / 32768.0 * self.ref_ohm
        return (r / self.r0 - 1.0) / 0.00385


def read_jetson_zones_c(root: str = "/sys/devices/virtual/thermal") -> dict[str, float]:
    out = {}
    for p in Path(root).glob("thermal_zone*/temp"):
        try:
            # Some Jetson models expose a PMIC zone fixed at 100 C; it is not a measurement.
            if "pmic" in (p.parent / "type").read_text().lower():
                continue
        except OSError:
            pass
        try:
            out[p.parent.name] = int(p.read_text().strip()) / 1000.0
        except (OSError, ValueError):
            pass
    return out


@dataclass
class ThermalSample:
    jetson_peak_c: Optional[float]
    sensor_c: Optional[float]
    obc_c: Optional[float]
    camera_c: Optional[float]
    heater_on: bool
    reason: str


class ThermalWatchdog:
    """One tick: read everything, decide the heater, log, and shut down above the limit."""

    def __init__(self, read_zones: Callable[[], dict] = read_jetson_zones_c,
                 read_rtd: Callable[[], tuple[Optional[float], Optional[float]]] = lambda: (None, None),
                 read_camera: Callable[[], Optional[float]] = lambda: None,
                 set_heater: Callable[[bool], None] = lambda on: None,
                 shutdown: Callable[[], None] = lambda: None,
                 flight_phase: Callable[[], Optional[int]] = lambda: None,
                 limit_c: float = config.JETSON_SHUTDOWN_C):
        self.read_zones, self.read_rtd, self.read_camera = read_zones, read_rtd, read_camera
        self.set_heater, self.shutdown, self.flight_phase, self.limit_c = set_heater, shutdown, flight_phase, limit_c
        self.ticks = 0
        self.heater_on = False
        self.history: "deque[ThermalSample]" = deque(maxlen=4320)   # one day at the default period

    def tick(self) -> ThermalSample:
        self.ticks += 1
        zones = self.read_zones()
        peak = max(zones.values()) if zones else None
        try:
            sensor_c, obc_c = self.read_rtd()
        except Exception as e:
            log.error("RTD read failed: %s", e)
            sensor_c, obc_c = None, None
        try:
            cam = self.read_camera()
        except Exception as e:
            log.error("camera temperature read failed: %s", e)
            cam = None
        d = heater_policy(sensor_c, self.flight_phase(), self.ticks, self.heater_on)
        self.heater_on = d.on
        self.set_heater(d.on)
        s = ThermalSample(peak, sensor_c, obc_c, cam, d.on, d.reason)
        self.history.append(s)
        if peak is not None and peak >= self.limit_c:
            log.critical("%s Jetson %.1f C >= %.1f C: shutting down", Code.CAM_OVERHEAT.value, peak, self.limit_c)
            self.shutdown()
        return s

    def run(self, stop: threading.Event, period_s: float = config.THERMAL_POLL_S) -> None:
        """Tick forever; a failed tick is logged and the next one still runs."""
        while not stop.is_set():
            try:
                self.tick()
            except Exception:
                log.exception("thermal watchdog tick failed")
            stop.wait(period_s)
