"""Sole owner of the Arduino link."""
from __future__ import annotations

import csv
import logging
import math
import os
import random
import socket
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Callable, Optional

from . import config, protocol
from .errors import Code, Fault, fw_err
from .protocol import FwLine

log = logging.getLogger("airis.bridge")

FW_BANNER = "MEGA2560-2AXIS-STEP"       # first line the firmware prints after a boot
_LATCHING_ERRS = (51, 52, 53, 54, 55, 56)
# ERR lines carry no seq. These two can be provoked by line noise at any time
# and say nothing about the command in progress, so they never end a wait.
_UNRELATED_ERRS = (60, 66)


def _is_err(m: FwLine) -> bool:
    return (m.kind == "ERR" and m.code is not None and m.code >= protocol.ERROR_MIN
            and m.code not in _UNRELATED_ERRS)


@dataclass
class MoveResult:
    ok: bool
    code: str            # "OK" or an error code
    msg: str = ""
    el_cmd: float = 0.0
    az_cmd: float = 0.0
    el_err: float = float("nan")
    az_err: float = float("nan")
    duration_s: float = 0.0
    seq: int = -1
    attempts: int = 1
    recalibrated: bool = False


class Bridge:
    def __init__(self, transport, clock: Callable[[], float] = time.monotonic,
                 moves_csv: Optional[str] = config.MOVES_CSV, name: str = "bridge",
                 require_sun: bool = True):
        self.t = transport
        self.clock = clock
        self.moves_csv = moves_csv
        self.require_sun = require_sun
        # Random start: the firmware drops a line whose seq AND text equal the last
        # executed one, so a restarted bridge must not begin at the same seq again.
        self._seq = random.randrange(256)
        self._send_lock = threading.Lock()
        self._lines: list[tuple[float, FwLine]] = []
        self._cv = threading.Condition()
        self._busy_lock = threading.Lock()
        self._busy: Optional[str] = None
        self.last_rx = clock()
        self.status_code: Optional[int] = None
        self.homed = False
        self.sun_ok = False                   # a SUN accepted since the firmware last booted
        self.fault: Optional[int] = None
        self._auto_clears = 0                 # automatic fault clears in the current window
        self._auto_clear_t0 = clock()
        self.last_pos: Optional[tuple[float, float]] = None
        self.stop = threading.Event()
        self.reopened = 0
        self.last_cmd_t = clock()
        self._reader = threading.Thread(target=self._read_loop, daemon=True, name=f"{name}-reader")
        self._monitor = threading.Thread(target=self._monitor_loop, daemon=True, name=f"{name}-monitor")
        self._reader.start()
        self._monitor.start()

    # ---- reader ----------------------------------------------------------------
    def _read_loop(self) -> None:
        while not self.stop.is_set():
            raw = self.t.readline(0.2)
            if not raw:
                continue
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            msg = protocol.parse_fw_line(line)
            now = self.clock()
            self.last_rx = now
            if msg.kind == "STATUS":
                self.status_code = msg.code
            elif msg.kind == "ERR":
                log.warning("firmware ERR %s", line)
                if msg.code in _LATCHING_ERRS:
                    self.fault = msg.code
                if msg.code == 64:
                    self.homed = False
                if msg.code == 67:
                    self._on_firmware_boot(line)
            elif msg.kind == "POS":
                self.last_pos = msg.args
            elif msg.kind == "OTHER" and line.startswith(FW_BANNER):
                self._on_firmware_boot(line)
            with self._cv:
                self._lines.append((now, msg))
                if len(self._lines) > 500:
                    del self._lines[:-500]
                self._cv.notify_all()

    def _on_firmware_boot(self, line: str) -> None:
        """The Mega (re)booted: encoders are zeroed where they sit and the sun center is the default."""
        if self.homed or self.sun_ok:
            log.error("%s firmware booted (%s): not homed, sun unknown until the next push",
                      Code.MNT_RESET.value, line[:60])
        self.homed = False
        self.sun_ok = False

    def _wait_for(self, pred: Callable[[FwLine], bool], timeout: float, since: float) -> Optional[FwLine]:
        deadline = self.clock() + timeout
        with self._cv:
            while True:
                for t, m in self._lines:
                    if t >= since and pred(m):
                        return m
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return None
                self._cv.wait(min(remaining, 0.05))

    # ---- sending ---------------------------------------------------------------
    def send(self, cmd: str) -> int:
        """Send one command and require ACK."""
        with self._send_lock:
            seq = self._seq
            self._seq = (self._seq + 1) % 256
            data = protocol.encode(cmd, seq)
            nak_reason = ""
            for attempt in range(1, config.ACK_RETRIES + 1):
                since = self.clock()
                self.last_cmd_t = since
                try:
                    self.t.write(data)
                except OSError as e:
                    raise Fault(Code.LNK_PORT_LOST, str(e))
                reply = self._wait_for(lambda m: m.kind in ("ACK", "NAK") and m.code == seq,
                                       config.ACK_TIMEOUT_S, since)
                if reply is None:
                    log.warning("%s no ACK for seq %d (attempt %d): %s", Code.LNK_NO_ACK.value, seq, attempt, cmd)
                    continue
                if reply.kind == "ACK":
                    return seq
                nak_reason = " ".join(reply.args)
                log.warning("%s NAK seq %d (%s), resending", Code.LNK_NAK.value, seq, nak_reason)
            raise Fault(Code.LNK_NO_ACK if not nak_reason else Code.LNK_NAK,
                        f"{cmd} not accepted after {config.ACK_RETRIES} attempts", {"reason": nak_reason})

    # ---- busy guard ------------------------------------------------------------
    def _acquire(self, what: str) -> None:
        with self._busy_lock:
            if self._busy is not None:
                raise Fault(Code.MNT_BUSY, f"bridge is executing {self._busy}")
            self._busy = what

    def _release(self) -> None:
        with self._busy_lock:
            self._busy = None

    def busy(self) -> Optional[str]:
        with self._busy_lock:
            return self._busy

    # ---- MOVE ------------------------------------------------------------------
    def move(self, el: float, az: float) -> MoveResult:
        try:
            self._acquire("move")
        except Fault as f:
            return MoveResult(False, f.code.value, str(f), el, az)
        try:
            res = self._move_once(el, az)
            if not res.ok and res.code in (Code.MNT_NOT_HOMED.value, Code.MNT_FAULT_LATCHED.value):
                log.warning("%s on MOVE; calibrating once, then retrying", res.code)
                with self._busy_lock:
                    self._busy = "cal"                    # the monitor keeps the firmware awake during CAL
                cal = self._calibrate_locked(auto=True)
                with self._busy_lock:
                    self._busy = "move"
                if cal.ok:
                    res = self._move_once(el, az)
                    res.recalibrated = True
                    res.attempts += 1
                else:
                    res = MoveResult(False, cal.code, f"recalibration failed: {cal.msg}", el, az)
            self._log_move(res)
            return res
        finally:
            self._release()

    def _move_once(self, el: float, az: float) -> MoveResult:
        t0 = self.clock()
        cmd = f"MOVE {protocol.fmt_angle(el)} {protocol.fmt_angle(az)}"
        try:
            seq = self.send(cmd)
        except Fault as f:
            return MoveResult(False, f.code.value, str(f), el, az, duration_s=self.clock() - t0)
        start = self._wait_for(lambda m: (m.kind == "STATUS" and m.code == protocol.STATUS_SLEWING) or _is_err(m),
                               config.MOVE_START_TIMEOUT_S, t0)
        if start is None:
            return MoveResult(False, Code.LNK_TIMEOUT.value, "no SLEWING after ACK", el, az, seq=seq,
                              duration_s=self.clock() - t0)
        if start.kind == "ERR":
            f = fw_err(start.code, " ".join(start.args))
            return MoveResult(False, f.code.value, str(f), el, az, seq=seq, duration_s=self.clock() - t0)
        budget = config.MOVE_MAX_S
        done = self._wait_for(lambda m: m.kind == "DONE" or _is_err(m), budget, t0)
        dt = self.clock() - t0
        if done is None:
            log.error("%s move not done after %.1fs; sending STOP", Code.LNK_TIMEOUT.value, budget)
            try:
                self.send("STOP")
                self._wait_for(lambda m: m.kind == "STATUS" and m.code == protocol.STATUS_IDLE, 5.0, self.clock())
            except Fault:
                pass
            return MoveResult(False, Code.MNT_STOPPED.value, f"timeout after {budget:.1f}s, STOP sent", el, az,
                              seq=seq, duration_s=dt)
        if done.kind == "ERR":
            f = fw_err(done.code, " ".join(done.args))
            res = MoveResult(False, f.code.value, str(f), el, az, seq=seq, duration_s=dt)
            if done.code == 58:
                nums = []
                for a in done.args:
                    try:
                        nums.append(float(a))
                    except ValueError:
                        continue
                if len(nums) >= 2:
                    res.el_err, res.az_err = nums[0], nums[1]
            return res
        el_err, az_err = done.args
        return MoveResult(True, "OK", "", el, az, el_err, az_err, dt, seq)

    # ---- CAL -------------------------------------------------------------------
    def calibrate(self) -> MoveResult:
        try:
            self._acquire("cal")
        except Fault as f:
            return MoveResult(False, f.code.value, str(f))
        try:
            return self._calibrate_locked(auto=False)
        finally:
            self._release()

    def _start_cal(self) -> tuple[int, Optional[FwLine]]:
        t0 = self.clock()
        seq = self.send("CAL")
        started = self._wait_for(lambda m: (m.kind == "STATUS" and m.code in protocol.STATUS_HOMING) or _is_err(m),
                                 config.CAL_START_TIMEOUT_S, t0)
        return seq, started

    def _calibrate_locked(self, auto: bool) -> MoveResult:
        """``auto`` = triggered by a failed move, so the latched-fault clear is rate-limited."""
        t0 = self.clock()
        if self.require_sun and not self.sun_ok:
            return MoveResult(False, Code.MNT_SUN_UNKNOWN.value,
                              "no sun position pushed since the firmware booted; refusing to home")
        self.homed = False
        try:
            seq, started = self._start_cal()
            if started is not None and started.kind == "ERR" and started.code == 65:
                # A latched fault (homing failure, cable catch, limit trip). Only the ACK
                # command clears it; without this the mount is dead until the Mega reboots.
                if auto:
                    if self.clock() - self._auto_clear_t0 > config.FAULT_AUTO_CLEAR_WINDOW_S:
                        self._auto_clears, self._auto_clear_t0 = 0, self.clock()
                    if self._auto_clears >= config.FAULT_AUTO_CLEAR_MAX:
                        return MoveResult(False, Code.MNT_FAULT_LATCHED.value,
                                          f"fault {self.fault} latched; {self._auto_clears} automatic clears used "
                                          f"in the last {config.FAULT_AUTO_CLEAR_WINDOW_S:.0f} s", seq=seq)
                    self._auto_clears += 1
                log.warning("%s fault %s latched; sending ACK to clear it, then CAL", Code.MNT_FAULT_LATCHED.value,
                            self.fault)
                t1 = self.clock()
                self.send("ACK")
                self._wait_for(lambda m: m.kind == "STATUS", 2.0, t1)
                seq, started = self._start_cal()
            if started is None:
                return MoveResult(False, Code.LNK_TIMEOUT.value, "homing never started", seq=seq)
            if started.kind == "ERR":
                f = fw_err(started.code, " ".join(started.args))
                return MoveResult(False, f.code.value, str(f), seq=seq)
            finished = self._wait_for(lambda m: (m.kind == "STATUS" and m.code == protocol.STATUS_IDLE) or _is_err(m),
                                      config.CAL_FINISH_TIMEOUT_S, self.clock())
            if finished is None:
                log.error("%s homing not finished after %.0fs; sending STOP", Code.LNK_TIMEOUT.value,
                          config.CAL_FINISH_TIMEOUT_S)
                try:
                    self.send("STOP")
                except Fault:
                    pass
                return MoveResult(False, Code.LNK_TIMEOUT.value, "homing never finished, STOP sent", seq=seq)
            if finished.kind == "ERR":
                f = fw_err(finished.code, " ".join(finished.args))
                return MoveResult(False, f.code.value, str(f), seq=seq)
            # anchor the frame: index -> physical home
            res = self._move_once(config.HOME_EL_OFFSET_DEG, config.HOME_AZ_OFFSET_DEG)
            if not res.ok:
                return MoveResult(False, res.code, f"offset move failed: {res.msg}", seq=seq)
            t1 = self.clock()
            self.send("SETZERO")
            zero = self._wait_for(lambda m: m.kind == "ZERO" or _is_err(m), 5.0, t1)
            if zero is None:
                return MoveResult(False, Code.LNK_TIMEOUT.value, "SETZERO not confirmed", seq=seq)
            if zero.kind == "ERR":
                f = fw_err(zero.code, " ".join(zero.args))
                return MoveResult(False, f.code.value, f"SETZERO refused: {f}", seq=seq)
            self.homed = True
            self.fault = None
            self._auto_clears = 0
            log.info("calibration complete: frame anchored to physical home (%.1fs)", self.clock() - t0)
            return MoveResult(True, "OK", "homed and anchored", duration_s=self.clock() - t0, seq=seq)
        except Fault as f:
            return MoveResult(False, f.code.value, str(f))

    # ---- SUN / STATUS ------------------------------------------------------------
    def set_sun(self, el: float, az: float) -> bool:
        """Push the sun center (physical-home frame). False if not confirmed."""
        if not (math.isfinite(el) and math.isfinite(az) and -180.0 <= el <= 180.0 and -360.0 <= az <= 360.0):
            # A non-finite center would switch the firmware's keep-out off entirely.
            log.error("%s refusing to push sun el=%r az=%r", Code.MNT_ANGLE.value, el, az)
            return False
        try:
            t0 = self.clock()
            self.send(f"SUN {protocol.fmt_angle(el)} {protocol.fmt_angle(az)}")
            reply = self._wait_for(lambda m: m.kind == "SUN" or _is_err(m), 2.0, t0)
        except Fault as f:
            log.warning("sun push failed: %s", f)
            return False
        if reply is None or reply.kind != "SUN":
            log.warning("sun push not confirmed: %s", reply.raw if reply else "no reply")
            return False
        self.sun_ok = True
        return True

    def query_status(self) -> Optional[int]:
        try:
            t0 = self.clock()
            self.send("STATUS")
            m = self._wait_for(lambda m: m.kind == "STATUS", 2.0, t0)
            return m.code if m else None
        except Fault:
            return None

    # ---- monitor ---------------------------------------------------------------------
    def _monitor_loop(self) -> None:
        while not self.stop.is_set():
            time.sleep(0.1)
            now = self.clock()
            busy = self.busy()
            if busy == "cal":
                # CAL runs minutes with no host command; the firmware's homing sleep
                # counts from the last command and would release the drivers.
                if now - self.last_cmd_t > config.CAL_KEEPALIVE_PERIOD_S:
                    self.query_status()
                continue
            if busy is not None:
                continue
            if now - self.last_rx > config.LINK_SILENCE_S:
                log.error("%s no line for %.0fs; reopening port", Code.LNK_PORT_LOST.value, now - self.last_rx)
                self.t.reopen()
                self.reopened += 1
                self.last_rx = now
                if self.query_status() is None:
                    continue
            elif now - self.last_cmd_t > config.KEEPALIVE_PERIOD_S:
                self.query_status()

    # ---- bookkeeping ------------------------------------------------------------------
    def _log_move(self, r: MoveResult) -> None:
        if not self.moves_csv:
            return
        try:
            new = not os.path.exists(self.moves_csv)
            with open(self.moves_csv, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["utc"] + list(asdict(r).keys()))
                if new:
                    w.writeheader()
                row = {"utc": datetime.now(timezone.utc).isoformat()}
                row.update(asdict(r))
                w.writerow(row)
        except OSError as e:               # a full disk must not turn a finished move into a failure
            log.error("moves.csv not written: %s", e)

    def close(self) -> None:
        self.stop.set()
        self.t.close()


# ---- Unix-socket request server -------------------------------------------------

class RequestServer(threading.Thread):
    """Line protocol for clients: ``move el az`` | ``cal`` | ``sun el az`` | ``status``."""

    def __init__(self, bridge: Bridge, path: str = config.BRIDGE_SOCKET_PATH):
        super().__init__(daemon=True, name="bridge-server")
        self.bridge, self.path = bridge, path
        if os.path.exists(path):
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            probe.settimeout(0.5)
            try:
                probe.connect(path)
            except OSError:
                pass                                   # stale socket file: nobody is listening
            else:
                raise RuntimeError(f"another bridge is already serving {path}")
            finally:
                probe.close()
            os.unlink(path)
        self.srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.srv.bind(path)
        self.srv.listen(4)
        self.srv.settimeout(0.5)

    def run(self) -> None:
        while not self.bridge.stop.is_set():
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve_conn, args=(conn,), daemon=True).start()
        self.srv.close()

    def _serve_conn(self, conn: socket.socket) -> None:
        with conn:
            conn.settimeout(5.0)
            try:
                line = conn.makefile("r", encoding="ascii").readline().strip()
            except (OSError, UnicodeDecodeError):
                return
            parts = line.split()
            if not parts:
                return
            try:
                if parts[0] == "status":
                    b = self.bridge
                    pos = f"{b.last_pos[0]:.2f},{b.last_pos[1]:.2f}" if b.last_pos else "-"
                    _reply(conn, f"status homed={int(b.homed)} busy={b.busy() or '-'} code={b.status_code} "
                                 f"fault={b.fault} sun={int(b.sun_ok)} pos={pos}")
                elif parts[0] == "sun" and len(parts) == 3:
                    ok = self.bridge.set_sun(float(parts[1]), float(parts[2]))
                    _reply(conn, "ok" if ok else f"failed {Code.LNK_TIMEOUT.value} sun not confirmed")
                elif parts[0] == "cal":
                    if self.bridge.busy():
                        _reply(conn, f"failed {Code.MNT_BUSY.value} busy")
                        return
                    _reply(conn, "accepted")
                    r = self.bridge.calibrate()
                    _reply(conn, f"done nan nan {r.duration_s:.2f}" if r.ok else f"failed {r.code} {r.msg}")
                elif parts[0] == "move" and len(parts) == 3:
                    el, az = float(parts[1]), float(parts[2])
                    if not (config.ANGLE_MIN_DEG <= el <= config.ANGLE_MAX_DEG and config.ANGLE_MIN_DEG <= az <= config.ANGLE_MAX_DEG):
                        _reply(conn, f"failed {Code.PTG_RANGE.value} outside +/-180")
                        return
                    if self.bridge.busy():
                        _reply(conn, f"failed {Code.MNT_BUSY.value} busy")
                        return
                    _reply(conn, "accepted")
                    r = self.bridge.move(el, az)
                    _reply(conn, f"done {r.el_err:.3f} {r.az_err:.3f} {r.duration_s:.2f}" if r.ok else f"failed {r.code} {r.msg}")
                else:
                    _reply(conn, f"failed {Code.MNT_CMD.value} unrecognized request")
            except Fault as e:
                _reply(conn, f"failed {e.code.value} {e}")
            except ValueError as e:
                _reply(conn, f"failed {Code.MNT_CMD.value} {e}")
            except Exception as e:             # never leave a client waiting on a dead handler thread
                log.exception("request %r failed", line)
                _reply(conn, f"failed {Code.LNK_UNEXPECTED.value} {type(e).__name__}: {e}")


def _reply(conn: socket.socket, text: str) -> None:
    try:
        conn.sendall((text + "\n").encode("ascii", "replace"))
    except OSError:
        pass


def main() -> None:  # pragma: no cover - flight entry point
    import argparse
    import signal
    from .transport import SerialTransport
    p = argparse.ArgumentParser()
    p.add_argument("--socket", default=config.BRIDGE_SOCKET_PATH)
    p.add_argument("--no-cal", action="store_true")
    a = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    transport = SerialTransport()
    if transport.ser is None:
        log.error("%s no serial port opened (%s); retrying every %.0fs", Code.LNK_PORT_LOST.value,
                  ", ".join(config.SERIAL_CANDIDATES), config.LINK_SILENCE_S)
    b = Bridge(transport)
    RequestServer(b, a.socket).start()       # serve first: clients must be able to push the sun

    def _terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _terminate)

    calibrated = a.no_cal
    if not calibrated:
        log.info("startup calibration waits for the first sun push")
    try:
        while True:
            if not calibrated and b.sun_ok and b.busy() is None:
                calibrated = True                # one attempt; later moves re-CAL on MNT-64/65
                log.info("startup calibration: %s", b.calibrate())
            time.sleep(1)
    except KeyboardInterrupt:
        b.close()


if __name__ == "__main__":  # pragma: no cover
    main()
