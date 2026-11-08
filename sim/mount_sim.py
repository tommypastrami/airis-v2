"""Kinematic model of the AIRIS two-axis mount, in simulated time.

Truth is the PHYSICAL frame (degrees from the mechanical home). The encoders
read ``physical - enc_offset`` per axis: at boot the offset is wherever the
mount sat (encoders zero on power-up), after homing it is the index position,
after SETZERO it is zero. That is exactly the firmware's three frames, so the
missing-SETZERO bug can be reproduced and the fix proven.

Motion: each axis follows a quintic position ramp whose duration comes from
``pointing.slew_time_s`` (the firmware's profile constants). Fault knobs:
encoder noise, per-move coupling slip, index-capture jitter, a stall angle,
and gravity sag while the drivers are released.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Optional

from airis import config
from airis.pointing import slew_time_s

ENC_CPR = 10000
INDEX_EL_PHYS = -config.HOME_EL_OFFSET_DEG   # index pulse sits here in the physical frame (+55)
INDEX_AZ_PHYS = -config.HOME_AZ_OFFSET_DEG   # (+35)


def quintic(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)


@dataclass
class AxisSim:
    name: str
    pos: float                      # physical degrees
    enc_offset: float               # encoder reads pos - enc_offset
    index_phys: float
    moving: bool = False
    t_start: float = 0.0
    duration: float = 0.0
    p0: float = 0.0
    p1: float = 0.0
    stall_at: Optional[float] = None    # physical angle where motion stops dead (snag)
    slip_per_move_deg: float = 0.0      # coupling slip: offset creeps every move
    noise_deg: float = 0.0
    rng: random.Random = field(default_factory=lambda: random.Random(1))

    def start(self, target_enc: float, now: float, land: bool = True) -> None:
        self.p0 = self.pos
        self.p1 = target_enc + self.enc_offset
        theta = abs(self.p1 - self.p0)
        self.duration = slew_time_s(theta, land) if theta > 1e-6 else 0.0
        self.t_start = now
        self.moving = self.duration > 0.0
        if self.slip_per_move_deg:
            self.enc_offset += self.slip_per_move_deg

    def advance(self, now: float) -> None:
        if not self.moving:
            return
        x = (now - self.t_start) / self.duration
        new = self.p0 + (self.p1 - self.p0) * quintic(x)
        if self.stall_at is not None and min(self.p0, self.p1) <= self.stall_at <= max(self.p0, self.p1):
            # a snag: the axis cannot pass stall_at
            if (self.p1 > self.p0 and new >= self.stall_at) or (self.p1 < self.p0 and new <= self.stall_at):
                self.pos = self.stall_at
                self.moving = False
                return
        self.pos = new
        if x >= 1.0:
            self.pos = self.p1
            self.moving = False

    def stop(self) -> None:
        self.moving = False

    def enc_deg(self) -> float:
        n = self.rng.gauss(0.0, self.noise_deg) if self.noise_deg else 0.0
        return self.pos - self.enc_offset + n

    def enc_counts(self) -> int:
        return int(round(self.enc_deg() * ENC_CPR / 360.0))


@dataclass
class MountSim:
    el: AxisSim
    az: AxisSim
    t: float = 0.0
    drivers_on: bool = False
    sag_rate_dps: float = 0.5      # el drifts down this fast while drivers are released
    sag_limit_deg: float = -20.0
    index_jitter_deg: float = 0.0
    rng: random.Random = field(default_factory=lambda: random.Random(7))

    @classmethod
    def at_boot(cls, el: float = 10.0, az: float = -20.0, **kw) -> "MountSim":
        """Power-up: the mount sits somewhere; encoders read zero there (not homed)."""
        return cls(AxisSim("EL", el, el, INDEX_EL_PHYS, **kw), AxisSim("AZ", az, az, INDEX_AZ_PHYS, **kw))

    # ---- time
    def advance(self, dt: float) -> None:
        self.t += dt
        self.el.advance(self.t)
        self.az.advance(self.t)
        if not self.drivers_on and not self.moving():
            if self.el.pos > self.sag_limit_deg:
                self.el.pos = max(self.sag_limit_deg, self.el.pos - self.sag_rate_dps * dt)

    def moving(self) -> bool:
        return self.el.moving or self.az.moving

    # ---- commands (encoder frame)
    def move_enc(self, el_enc: float, az_enc: float, land: bool = True) -> None:
        self.drivers_on = True
        self.el.start(el_enc, self.t, land)
        self.az.start(az_enc, self.t, land)

    def stop(self) -> None:
        self.el.stop()
        self.az.stop()

    def home_axis(self, axis: AxisSim) -> None:
        """Index capture: the encoder now reads zero at the index pulse (plus jitter)."""
        j = self.rng.gauss(0.0, self.index_jitter_deg) if self.index_jitter_deg else 0.0
        axis.enc_offset = axis.index_phys + j
        axis.pos = axis.index_phys       # homing ends at the index
        axis.moving = False

    def setzero(self) -> None:
        # firmware: enc -= offset*CPR/360, i.e. the position that read `offset` now reads 0
        self.el.enc_offset += config.HOME_EL_OFFSET_DEG
        self.az.enc_offset += config.HOME_AZ_OFFSET_DEG

    def reset(self) -> None:
        """A board reset (DTR): encoders zero wherever the mount sits."""
        self.stop()
        self.el.enc_offset = self.el.pos
        self.az.enc_offset = self.az.pos
        self.drivers_on = False

    def release(self) -> None:
        self.drivers_on = False

    def residual_enc(self, el_target: float, az_target: float) -> tuple[float, float]:
        return el_target - self.el.enc_deg(), az_target - self.az.enc_deg()

    def physical(self) -> tuple[float, float]:
        return self.el.pos, self.az.pos
