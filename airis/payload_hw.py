"""Payload hardware: RTD thermometers, the heater policy, Jetson thermal zones, the watchdog.

Everything hardware-facing takes an injectable reader/actuator so the policy is
testable on a laptop. The heater policy is explicit about the one case v1 got
backwards: an unreadable sensor temperature during ASCENT defaults the heater
ON (duty-cycled), because the paper's ascent profile reaches -60 C and the
sensor's floor is -25 C; at float it defaults OFF.
"""
from __future__ import annotations

import logging
import time
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


def heater_policy(sensor_c: Optional[float], flight_phase: Optional[int], duty_tick: int = 0) -> HeaterDecision:
    if sensor_c is None:
        if flight_phase == PHASE_ASCENT:
            return HeaterDecision(duty_tick % 2 == 0, "RTD unreadable during ascent: 50% duty")
        return HeaterDecision(False, "RTD unreadable: heater off")
    if sensor_c < config.SENSOR_HEAT_ON_C:
        return HeaterDecision(True, f"sensor {sensor_c:.1f} C below {config.SENSOR_HEAT_ON_C} C")
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
            raise RuntimeError("MAX31865 fault")
        r = raw / 32768.0 * self.ref_ohm
        return (r / self.r0 - 1.0) / 0.00385


def read_jetson_zones_c(root: str = "/sys/devices/virtual/thermal") -> dict[str, float]:
    out = {}
    for p in Path(root).glob("thermal_zone*/temp"):
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
        self.history: list[ThermalSample] = []

    def tick(self) -> ThermalSample:
        self.ticks += 1
        zones = self.read_zones()
        peak = max(zones.values()) if zones else None
        try:
            sensor_c, obc_c = self.read_rtd()
        except Exception as e:
            log.error("RTD read failed: %s", e)
            sensor_c, obc_c = None, None
        cam = self.read_camera()
        d = heater_policy(sensor_c, self.flight_phase(), self.ticks)
        self.set_heater(d.on)
        s = ThermalSample(peak, sensor_c, obc_c, cam, d.on, d.reason)
        self.history.append(s)
        if peak is not None and peak >= self.limit_c:
            log.critical("%s Jetson %.1f C >= %.1f C: shutting down", Code.CAM_OVERHEAT.value, peak, self.limit_c)
            self.shutdown()
        return s
