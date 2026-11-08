"""A fake Arduino: speaks protocol v5 over one end of a socketpair, drives a MountSim.

It models what the v5 firmware (firmware/FIRMWARE_V5_CHANGES.md) does at the
protocol level — ACK/NAK with sequence deduplication, distinct error codes,
STATUS pushes, DONE with residuals, bounded settle with no AZ nudge, SETZERO,
live SUN, hold timeout, heartbeat — plus fault injection for the link:

    Faults.garble_after_idle_s   corrupt the first bytes of a command that arrives
                                 after this much idle time (USB autosuspend wake)
    Faults.drop_byte_prob        random byte loss on the way in
    Faults.tx_corrupt_prob       random byte damage on the way out (echo/ACK damage)

Simulated time runs ``speed`` times faster than wall time so a 4 s slew takes
0.16 s in a test. Everything the firmware would print is also appended to
``self.log`` for assertions.
"""
from __future__ import annotations

import random
import re
import select
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from airis import config
from airis.protocol import ProtocolError, parse_host_line

from .mount_sim import MountSim

FW_ID = "MEGA2560-2AXIS-STEP v5.0-sim (AIRIS: seq+checksum, ACK/NAK, bounded settle, live SUN)"

NOT_HOMED, IDLE, MOVING, HOMING, FAULT = "NOT_HOMED", "IDLE", "MOVING", "HOMING", "FAULT"
_SEQ_RE = re.compile(r"#(\d{1,3})\*")


@dataclass
class Faults:
    garble_after_idle_s: float = 0.0
    garble_bytes: int = 2
    drop_byte_prob: float = 0.0
    tx_corrupt_prob: float = 0.0
    seed: int = 3


class FakeArduino(threading.Thread):
    def __init__(self, sock: socket.socket, mount: MountSim, speed: float = 25.0,
                 faults: Optional[Faults] = None, sun_center_phys: tuple[float, float] = (-55.0, -35.0),
                 hold_silence_s: float = config.HOLD_SILENCE_S, settle_tol: float = config.SETTLE_TOL_DEG,
                 settle_max: int = config.SETTLE_MAX_PASSES, cal_duration_s: float = 12.0,
                 heartbeat_s: float = 1.0):
        super().__init__(daemon=True, name="fake-arduino")
        self.sock = sock
        self.sock.setblocking(False)
        self.mount = mount
        self.speed = speed
        self.faults = faults or Faults()
        self.rng = random.Random(self.faults.seed)
        self.sun_el_phys, self.sun_az_phys = sun_center_phys
        self.box = config.SunBoxSpec()
        self.hold_silence_s, self.settle_tol, self.settle_max = hold_silence_s, settle_tol, settle_max
        self.cal_duration_s, self.heartbeat_s = cal_duration_s, heartbeat_s
        self.stop_evt = threading.Event()
        self.state = NOT_HOMED
        self.fault_code: Optional[int] = None
        self.homed = False
        self.target = (0.0, 0.0)
        self.settle_passes = 0
        self.settle_wait_until: Optional[float] = None
        self.last_cmd_t = 0.0
        self.last_rx_sim_t = -1e9
        self.last_hb_t = 0.0
        self.cal_t0 = 0.0
        self.cal_T = cal_duration_s
        self.cal_phase = 0
        self.last_seq: Optional[int] = None
        self.last_text: Optional[str] = None
        self.moves_executed: list[tuple[float, float]] = []
        self.log: list[str] = []
        self.rx_lines = 0
        self.naks = 0
        self._buf = b""
        self._lock = threading.Lock()
        self.mute = False               # True = link dead: ignore input, print nothing

    # ---- output
    def _write(self, text: str) -> None:
        line = (text + "\n").encode("ascii", "replace")
        if self.faults.tx_corrupt_prob and self.rng.random() < self.faults.tx_corrupt_prob and len(line) > 3:
            i = self.rng.randrange(0, len(line) - 1)
            line = line[:i] + bytes([line[i] ^ 0x80]) + line[i + 1:]
        with self._lock:
            self.log.append(text)
        if self.mute:
            return
        try:
            self.sock.sendall(line)
        except OSError:
            pass

    def _status(self) -> None:
        code = {IDLE: 0, NOT_HOMED: 0, MOVING: 15, HOMING: 12, FAULT: 0}[self.state]
        name = {IDLE: "IDLE", NOT_HOMED: "NOT_HOMED", MOVING: "SLEWING", HOMING: "HOMING_M2", FAULT: "FAULT"}[self.state]
        self._write(f"STATUS {code} {name}")

    def _err(self, code: int, msg: str) -> None:
        self._write(f"ERR {code} {msg}")

    # ---- frame helpers (encoder frame <-> physical)
    def _enc_to_phys(self, el: float, az: float) -> tuple[float, float]:
        return el + self.mount.el.enc_offset, az + self.mount.az.enc_offset

    def _in_sun_box(self, el_enc: float, az_enc: float) -> bool:
        """Both attitudes of the target are checked, as the firmware does (a flip can stare at the sun)."""
        el, az = self._enc_to_phys(el_enc, az_enc)
        flip_el = 180.0 - el if el >= 0 else -180.0 - el
        for e, a in ((el, az), (flip_el, az + 180.0)):
            d_az = (a - self.sun_az_phys + 180.0) % 360.0 - 180.0
            if abs(e - self.sun_el_phys) <= self.box.half_el and abs(d_az) <= self.box.half_az:
                return True
        return False

    # ---- main loop
    def run(self) -> None:
        last_real = time.monotonic()
        while not self.stop_evt.is_set():
            r, _, _ = select.select([self.sock], [], [], 0.002)
            now_real = time.monotonic()
            dt = (now_real - last_real) * self.speed
            last_real = now_real
            self.mount.advance(dt)
            if r:
                try:
                    data = self.sock.recv(4096)
                except (BlockingIOError, InterruptedError):
                    data = b""
                except OSError:
                    break
                if data == b"" and r:
                    break
                self._ingest(data)
            self._service()

    def _ingest(self, data: bytes) -> None:
        if not data or self.mute:
            return
        t = self.mount.t
        f = self.faults
        if f.garble_after_idle_s and t - self.last_rx_sim_t > f.garble_after_idle_s:
            n = min(f.garble_bytes, len(data))
            data = b"\xff" * n + data[n:]
        if f.drop_byte_prob:
            data = bytes(b for b in data if self.rng.random() >= f.drop_byte_prob)
        self.last_rx_sim_t = t
        self._buf += data
        while b"\n" in self._buf:
            raw, _, self._buf = self._buf.partition(b"\n")
            text = "".join(chr(b) for b in raw if 0x20 <= b <= 0x7E)   # non-printable filter
            if text:
                self._handle_line(text)

    # ---- command handling
    def _handle_line(self, text: str) -> None:
        self.rx_lines += 1
        self.last_cmd_t = self.mount.t
        self._write(f"RX: {text}")
        try:
            hl = parse_host_line(text)
        except ProtocolError as e:
            m = _SEQ_RE.search(text)
            seq = int(m.group(1)) if m else -1
            self.naks += 1
            self._write(f"NAK {seq} {e}")
            return
        if hl.seq == self.last_seq and text == self.last_text:
            self._write(f"ACK {hl.seq}")      # duplicate: acknowledge, do not execute again
            return
        self.last_seq, self.last_text = hl.seq, text
        self._write(f"ACK {hl.seq}")
        getattr(self, f"_cmd_{hl.cmd.lower()}", self._cmd_unknown)(hl.args)

    def _busy(self) -> bool:
        return self.state in (MOVING, HOMING) or self.mount.moving()

    def _cmd_move(self, args: list[str]) -> None:
        if self.state == FAULT:
            self._err(65, "fault latched (send ACK)"); return
        if self._busy():
            self._err(63, "busy"); return
        if len(args) < 2:
            self._err(62, "usage: MOVE <el> <az>"); return
        try:
            el, az = float(args[0]), float(args[1])
        except ValueError:
            self._err(62, "non-numeric"); return
        if not (-180.0 <= el <= 180.0 and -180.0 <= az <= 180.0):
            self._err(61, "target outside per-axis soft limits"); return
        if not self.homed:
            self._err(64, "not homed (send CAL)"); return
        if self._in_sun_box(el, az):
            self._err(57, "target is sun-facing (inside sun keep-out)"); return
        self.target = (el, az)
        self.settle_passes = 0
        self.settle_wait_until = None
        self.moves_executed.append((el, az))
        self.mount.move_enc(el, az, land=True)
        self.state = MOVING
        self._status()

    def _cmd_stop(self, args) -> None:
        self.mount.stop()
        self.state = IDLE if self.homed else NOT_HOMED
        self._write("STOPPED")
        self._status()

    def _cmd_cal(self, args) -> None:
        if self.state == FAULT:
            self._err(65, "fault latched (send ACK)"); return
        if self._busy():
            self._err(63, "busy"); return
        self.state = HOMING
        self.homed = False
        self.cal_t0 = self.mount.t
        self.cal_T = self.cal_duration_s
        self.cal_phase = 0
        self.mount.drivers_on = True
        self._status()

    _cmd_home = _cmd_cal

    def _cmd_calq(self, args) -> None:
        self._cmd_cal(args)
        self.cal_T = self.cal_duration_s / 5.0

    def _cmd_setzero(self, args) -> None:
        if self._busy():
            self._err(63, "busy"); return
        self.mount.setzero()
        self.target = (self.mount.el.enc_deg(), self.mount.az.enc_deg())
        self._write("ZERO SET")
        self._status()

    def _cmd_ack(self, args) -> None:
        if self.state == FAULT:
            self.state = NOT_HOMED
            self.homed = False
            self.fault_code = None
            self._write("OK")
            self._status()
        else:
            self._write("OK NO FAULT")

    def _cmd_status(self, args) -> None:
        self._status()

    def _cmd_pos(self, args) -> None:
        self._write(f"POS {self.mount.el.enc_deg():.2f} {self.mount.az.enc_deg():.2f}")

    def _cmd_sun(self, args) -> None:
        if len(args) < 2:
            self._err(62, "usage: SUN <el> <az>"); return
        el, az = float(args[0]), float(args[1])
        self.sun_el_phys, self.sun_az_phys = self._enc_to_phys(el, az)
        self._write(f"SUN OK {el:.3f} {az:.3f}")

    def _cmd_id(self, args) -> None:
        self._write(FW_ID)

    def _cmd_unknown(self, args) -> None:
        self._err(60, "unknown command (HELP for list)")

    # ---- state machine
    def _service(self) -> None:
        t = self.mount.t
        if self.state == HOMING:
            x = (t - self.cal_t0) / self.cal_T
            if self.cal_phase == 0 and x >= 0.4:
                self.mount.home_axis(self.mount.az)
                self._write("STATUS 11 HOMING_M1")
                self.cal_phase = 1
            elif self.cal_phase == 1 and x >= 0.8:
                self.mount.home_axis(self.mount.el)
                self._write("STATUS 13 HOMING_COMPLETE")
                self.cal_phase = 2
            elif self.cal_phase == 2 and x >= 1.0:
                self.homed = True
                self.state = IDLE
                self.target = (0.0, 0.0)
                self.last_cmd_t = t
                self._status()
        elif self.state == MOVING and not self.mount.moving():
            if self.settle_wait_until is None:
                self.settle_wait_until = t + 0.2
            elif t >= self.settle_wait_until:
                e1, e2 = self.mount.residual_enc(*self.target)
                if abs(e1) <= self.settle_tol and abs(e2) <= self.settle_tol:
                    self.state = IDLE
                    self.last_cmd_t = t
                    self._write(f"DONE {e1:.3f} {e2:.3f}")
                    self._status()
                else:
                    self.settle_passes += 1
                    self.settle_wait_until = None
                    if self.settle_passes > self.settle_max:
                        self.state = IDLE
                        self.last_cmd_t = t
                        self._err(58, f"settle {e1:.3f} {e2:.3f} after {self.settle_max} passes")
                        self._status()
                    else:
                        self.mount.move_enc(*self.target, land=False)   # trim: same target, never another axis
        if self.state == IDLE and self.mount.drivers_on and t - self.last_cmd_t > self.hold_silence_s:
            self.mount.release()
            self._write("HOLD TIMEOUT: no commands - drivers released")
        if self.state in (IDLE, NOT_HOMED) and not self.mount.moving() and t - self.last_hb_t >= self.heartbeat_s:
            self.last_hb_t = t
            self._write(f"POS {self.mount.el.enc_deg():.2f} {self.mount.az.enc_deg():.2f}")

    # ---- external events
    def reset(self) -> None:
        """What a DTR pulse does: reboot, encoders zeroed where they sit, not homed."""
        self.mount.reset()
        self.state = NOT_HOMED
        self.homed = False
        self.last_seq = None
        self._write(FW_ID)
        self._err(67, "RESET: controller (re)booted - encoders zeroed, not homed (send CAL)")

    def shutdown(self) -> None:
        self.stop_evt.set()


def make_pair(**kw) -> tuple[socket.socket, FakeArduino]:
    """Return (host_end_socket, running FakeArduino) joined by a socketpair."""
    a, b = socket.socketpair()
    mount = kw.pop("mount", None) or MountSim.at_boot()
    fa = FakeArduino(b, mount, **kw)
    fa.start()
    return a, fa
