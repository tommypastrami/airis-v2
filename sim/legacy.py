"""Models of the v1 pipeline, for before/after comparisons only.

* FakeArduinoV4: the v4.4 serial behaviour — no checksum, no ACK/NAK, an `RX:` echo,
  `ERR 50` for busy / not-homed / fault / settle, `DONE` without residuals. (The v4 AZ
  nudge on a stuck EL is not modelled; its exact pattern is in the .ino, lines 2685-2735.)
* LegacyBridge: the v1 airis_bridge control flow — echo verification with 3 resends,
  resend-on-silence, `ERR 50` => re-CAL-and-retry loop, and `_move_active` set before
  the send (so an error to a garbled first attempt is stored as the move's result).
* greedy_pixel_order: v1 tile selection — most probable HEALPix pixel above the horizon,
  skipping pixels already tried.
* v1_transient_found: image_subtraction.py's detector — autoscaled PNG, open/close, blob.

Timeouts are the v1 values divided by `scale` so an experiment runs in minutes.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from airis.transport import SocketTransport

from .fake_arduino import FakeArduino

V4_CODE = {58: 50, 63: 50, 64: 50, 65: 50, 66: 50, 67: 50}


class FakeArduinoV4(FakeArduino):
    """v4.4 serial behaviour on top of the same mount: every command line is executed as
    received (no checksum), echoed as ``RX:``; settle/busy/not-homed/fault/reset all say
    ``ERR 50``; ``DONE`` carries no residual. Motion is the shared model."""

    def _err(self, code: int, msg: str) -> None:
        super()._err(V4_CODE.get(code, code), msg)

    def _write(self, text: str) -> None:
        super()._write("DONE" if text.startswith("DONE") else text)

    def _handle_line(self, text: str) -> None:
        self.rx_lines += 1
        self.last_cmd_t = self.mount.t
        self._write(f"RX: {text}")
        parts = text.split()
        if not parts:
            return
        getattr(self, f"_cmd_{parts[0].lower()}", self._cmd_unknown)(parts[1:])


class LegacyBridge:
    """Just enough of airis_bridge v1 to reproduce its failure modes."""

    def __init__(self, transport: SocketTransport, scale: float = 5.0):
        self.t = transport
        self.k = scale
        self.ECHO, self.START, self.MOVE, self.CAL_START, self.CAL_FINISH, self.GAP, self.BUDGET = (
            1.0 / scale, 10.0 / scale, 20.0 / scale, 15.0 / scale, 120.0 / scale, 5.0 / scale, 300.0 / scale)
        self._lines: list[tuple[float, str]] = []
        self._cv = threading.Condition()
        self.stop = threading.Event()
        self.move_active = False
        self.move_result: Optional[str] = None
        self.saw_slewing = False
        self.mount_homing = False
        self.homed = False
        self.cals_sent = 0
        self.reloads = 0
        self.resends = 0
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        while not self.stop.is_set():
            raw = self.t.readline(0.2)
            if not raw:
                continue
            line = raw.decode("ascii", "ignore").strip()   # v1: damaged bytes silently deleted
            with self._cv:
                self._lines.append((time.monotonic(), line))
                parts = line.split()
                if line == "DONE" and self.move_active and self.move_result is None:
                    self.move_result = "ok"
                elif len(parts) >= 2 and parts[0] in ("STATUS", "ERR"):
                    try:
                        code = int(parts[1])
                    except ValueError:
                        code = -1
                    if code in (11, 12, 13):
                        self.mount_homing = True
                    elif code == 0 and self.mount_homing:
                        self.mount_homing = False
                        self.homed = True
                    if self.move_active:
                        if code == 15:
                            self.saw_slewing = True
                        elif code == 0 and self.saw_slewing and self.move_result is None:
                            self.move_result = "ok"
                        elif code >= 50 and self.move_result is None:
                            self.move_result = f"err {line}"
                self._cv.notify_all()

    def _wait_line(self, text: str, timeout: float, since: float) -> bool:
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                if any(t >= since and l == text for t, l in self._lines):
                    return True
                if time.monotonic() >= deadline:
                    return False
                self._cv.wait(0.02)

    def send_verified(self, text: str) -> bool:
        for attempt in range(3):
            since = time.monotonic()
            self.t.write((text + "\n").encode())
            if self._wait_line(f"RX: {text}", self.ECHO, since):
                return True
            self.resends += 1
        self.reloads += 1                      # v1 would re-exec itself here
        return False

    def _exec_move(self, cmd: str, total: float, resend_on_silence: bool) -> str:
        with self._cv:
            self.move_active, self.saw_slewing, self.move_result = True, False, None
        try:
            self.send_verified(cmd)
            start = time.monotonic()
            resent = False
            while True:
                with self._cv:
                    r, slewing = self.move_result, self.saw_slewing
                now = time.monotonic()
                if r is not None:
                    return r
                if not slewing and not resent and resend_on_silence and now - start > self.START:
                    self.send_verified(cmd)
                    resent = True
                if now - start > total:
                    return "timeout"
                time.sleep(0.01)
        finally:
            with self._cv:
                self.move_active = False

    def run_cal(self) -> str:
        self.cals_sent += 1
        with self._cv:
            self.mount_homing, self.homed = False, False
        self.send_verified("CAL")
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.CAL_START:
            with self._cv:
                if self.mount_homing:
                    break
            time.sleep(0.01)
        else:
            return "err never started"
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.CAL_FINISH:
            with self._cv:
                if not self.mount_homing:
                    return "ok" if self.homed else "err"
            time.sleep(0.01)
        return "err timeout"

    def move(self, el: float, az: float) -> str:
        cmd = f"MOVE {el:.3f} {az:.3f}"
        while True:
            with self._cv:
                h = self.mount_homing
            if not h:
                break
            time.sleep(0.01)
        r = self._exec_move(cmd, self.MOVE, True)
        deadline = time.monotonic() + self.BUDGET
        while r.startswith("err") and "ERR 50" in r and time.monotonic() < deadline:
            self.run_cal()
            time.sleep(self.GAP)
            r = self._exec_move(cmd, self.MOVE, True)
        return r

    def close(self) -> None:
        self.stop.set()


# ---- v1 tile selection --------------------------------------------------------------

def greedy_pixel_order(prob: np.ndarray, nside: int, visible: np.ndarray, n: int) -> list[int]:
    order = np.argsort(prob)[::-1]
    out = []
    for pix in order:
        if visible[pix]:
            out.append(int(pix))
        if len(out) >= n:
            break
    return out


# ---- v1 detector --------------------------------------------------------------------

def v1_transient_found(diff: np.ndarray) -> bool:
    """image_subtraction.py's transientFound(): autoscale to 8-bit, open/close 5x5, blur, blob detect."""
    import cv2
    lo, hi = float(diff.min()), float(diff.max())
    img = ((diff - lo) / max(hi - lo, 1e-9) * 255.0).astype(np.uint8)
    kernel = np.ones((5, 5), np.uint8)
    opened = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    smoothed = cv2.GaussianBlur(cleaned, (3, 3), 0)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea, params.minArea = True, 10
    params.filterByColor, params.blobColor = True, 255
    params.filterByCircularity, params.minCircularity = True, 0.1
    return len(cv2.SimpleBlobDetector_create(params).detect(smoothed)) > 0
