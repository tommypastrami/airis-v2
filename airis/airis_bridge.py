"""Sole owner of the Arduino link. Talks protocol v5; serves a Unix socket.

What it does
    * Sends every command with a sequence number and checksum and waits for
      ``ACK``/``NAK`` (ACK_TIMEOUT_S, ACK_RETRIES). The firmware deduplicates by
      seq, so a resend after a lost ACK never executes twice.
    * MOVE: wait for ``STATUS 15`` (start), then ``DONE el_err az_err`` or an
      ``ERR``; on timeout send ``STOP`` and report. One request at a time; a
      second caller is answered ``failed MNT-63`` at once, never queued.
    * CAL: wait for homing to start and finish, then anchor the frame:
      ``MOVE HOME_EL_OFFSET HOME_AZ_OFFSET`` and ``SETZERO``.
    * ``ERR 64 not homed`` on a move triggers exactly one calibrate + retry.
    * Keepalive ``STATUS`` every KEEPALIVE_PERIOD_S while idle so the firmware
      never releases the hold; link silence > LINK_SILENCE_S reopens the port.
    * Every move is one row in moves.csv (the calibration dataset).

What it deliberately does not do any more
    re-CAL on "busy", unstick detours, resend-on-silence, re-exec of itself.
"""
from __future__ import annotations

import csv
import logging
import os
import socket
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Callable, Optional

from . import config, protocol
from .errors import Code, Fault, fw_err
from .pointing import slew_time_s
from .protocol import FwLine

log = logging.getLogger("airis.bridge")


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
                 moves_csv: Optional[str] = config.MOVES_CSV, name: str = "bridge"):
        self.t = transport
        self.clock = clock
        self.moves_csv = moves_csv
        self._seq = 0
        self._lines: list[tuple[float, FwLine]] = []
        self._cv = threading.Condition()
        self._busy_lock = threading.Lock()
        self._busy: Optional[str] = None
        self.last_rx = clock()
        self.status_code: Optional[int] = None
        self.homed = False
        self.fault: Optional[int] = None
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
                if msg.code == protocol.STATUS_IDLE and self.fault is None:
                    pass
            elif msg.kind == "ERR":
                log.warning("firmware ERR %s", line)
                if msg.code in (51, 52, 53, 54, 55, 56, 65):
                    self.fault = msg.code
                if msg.code in (64, 67):
                    self.homed = False
            elif msg.kind == "POS":
                self.last_pos = msg.args
            with self._cv:
                self._lines.append((now, msg))
                if len(self._lines) > 500:
                    del self._lines[:-500]
                self._cv.notify_all()

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
        """Send one command and require ACK. Raises Fault(LNK_NO_ACK/LNK_NAK)."""
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
            if not res.ok and res.code == Code.MNT_NOT_HOMED.value:
                log.warning("%s mount not homed; calibrating once, then retrying", res.code)
                cal = self._calibrate_locked()
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
        start = self._wait_for(lambda m: (m.kind == "STATUS" and m.code == protocol.STATUS_SLEWING) or m.kind == "ERR",
                               config.MOVE_START_TIMEOUT_S, t0)
        if start is None:
            return MoveResult(False, Code.LNK_TIMEOUT.value, "no SLEWING after ACK", el, az, seq=seq,
                              duration_s=self.clock() - t0)
        if start.kind == "ERR":
            f = fw_err(start.code, " ".join(start.args))
            return MoveResult(False, f.code.value, str(f), el, az, seq=seq, duration_s=self.clock() - t0)
        prev = self.last_pos or (0.0, 0.0)
        theta = max(abs(el - prev[0]), abs(az - prev[1]))
        budget = min(config.MOVE_MAX_S, slew_time_s(theta) + config.MOVE_MARGIN_S)
        done = self._wait_for(lambda m: m.kind == "DONE" or (m.kind == "ERR" and m.code >= 50), budget, t0)
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
        el_err, az_err = done.args if len(done.args) == 2 else (float("nan"), float("nan"))
        return MoveResult(True, "OK", "", el, az, el_err, az_err, dt, seq)

    # ---- CAL -------------------------------------------------------------------
    def calibrate(self) -> MoveResult:
        try:
            self._acquire("cal")
        except Fault as f:
            return MoveResult(False, f.code.value, str(f))
        try:
            return self._calibrate_locked()
        finally:
            self._release()

    def _calibrate_locked(self) -> MoveResult:
        t0 = self.clock()
        self.homed = False
        self.fault = None
        try:
            seq = self.send("CAL")
            started = self._wait_for(lambda m: (m.kind == "STATUS" and m.code in protocol.STATUS_HOMING) or m.kind == "ERR",
                                     config.CAL_START_TIMEOUT_S, t0)
            if started is None:
                return MoveResult(False, Code.LNK_TIMEOUT.value, "homing never started", seq=seq)
            if started.kind == "ERR":
                f = fw_err(started.code, " ".join(started.args))
                return MoveResult(False, f.code.value, str(f), seq=seq)
            finished = self._wait_for(lambda m: (m.kind == "STATUS" and m.code == protocol.STATUS_IDLE)
                                      or (m.kind == "ERR" and m.code in (51, 52, 53, 54)),
                                      config.CAL_FINISH_TIMEOUT_S, self.clock())
            if finished is None:
                return MoveResult(False, Code.LNK_TIMEOUT.value, "homing never finished", seq=seq)
            if finished.kind == "ERR":
                f = fw_err(finished.code, " ".join(finished.args))
                return MoveResult(False, f.code.value, str(f), seq=seq)
            # anchor the frame: index -> physical home
            self.homed = True  # index frame homed; needed so the offset MOVE is accepted
            res = self._move_once(config.HOME_EL_OFFSET_DEG, config.HOME_AZ_OFFSET_DEG)
            if not res.ok:
                self.homed = False
                return MoveResult(False, res.code, f"offset move failed: {res.msg}", seq=seq)
            t1 = self.clock()
            self.send("SETZERO")
            zero = self._wait_for(lambda m: m.kind == "ZERO" or m.kind == "ERR", 5.0, t1)
            if zero is None or zero.kind == "ERR":
                self.homed = False
                return MoveResult(False, Code.LNK_TIMEOUT.value, "SETZERO not confirmed", seq=seq)
            self.homed = True
            log.info("calibration complete: frame anchored to physical home (%.1fs)", self.clock() - t0)
            return MoveResult(True, "OK", "homed and anchored", duration_s=self.clock() - t0, seq=seq)
        except Fault as f:
            return MoveResult(False, f.code.value, str(f))

    # ---- SUN / STATUS ------------------------------------------------------------
    def set_sun(self, el: float, az: float) -> bool:
        try:
            t0 = self.clock()
            self.send(f"SUN {protocol.fmt_angle(el)} {protocol.fmt_angle(az)}")
            return self._wait_for(lambda m: m.kind == "SUN", 2.0, t0) is not None
        except Fault as f:
            log.warning("sun push failed: %s", f)
            return False

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
            if self.busy() is not None:
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
        new = not os.path.exists(self.moves_csv)
        with open(self.moves_csv, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["utc"] + list(asdict(r).keys()))
            if new:
                w.writeheader()
            row = {"utc": datetime.now(timezone.utc).isoformat()}
            row.update(asdict(r))
            w.writerow(row)

    def close(self) -> None:
        self.stop.set()
        self.t.close()


# ---- Unix-socket request server -------------------------------------------------

class RequestServer(threading.Thread):
    """Line protocol for clients: ``move el az`` | ``cal`` | ``sun el az`` | ``status``.

    Replies: ``accepted`` then ``done el_err az_err dt`` / ``failed CODE msg``;
    a request while busy gets ``failed MNT-63 busy`` immediately.
    """

    def __init__(self, bridge: Bridge, path: str = config.BRIDGE_SOCKET_PATH):
        super().__init__(daemon=True, name="bridge-server")
        self.bridge, self.path = bridge, path
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
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
                    _reply(conn, f"status homed={int(b.homed)} busy={b.busy() or '-'} code={b.status_code} fault={b.fault}")
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
            except (ValueError, Fault) as e:
                code = e.code.value if isinstance(e, Fault) else Code.MNT_CMD.value
                _reply(conn, f"failed {code} {e}")


def _reply(conn: socket.socket, text: str) -> None:
    try:
        conn.sendall((text + "\n").encode("ascii", "replace"))
    except OSError:
        pass


def main() -> None:  # pragma: no cover - flight entry point
    import argparse
    from .transport import SerialTransport
    p = argparse.ArgumentParser()
    p.add_argument("--socket", default=config.BRIDGE_SOCKET_PATH)
    p.add_argument("--no-cal", action="store_true")
    a = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    b = Bridge(SerialTransport())
    if not a.no_cal:
        log.info("startup calibration: %s", b.calibrate())
    RequestServer(b, a.socket).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        b.close()


if __name__ == "__main__":  # pragma: no cover
    main()
