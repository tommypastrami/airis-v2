# Firmware v5 changes (motorsworking_715.ino v4.4 -> v5.0 -> v5.1)

Sections 1-6 describe v5.0: six changes in the serial/command layer and the settle
logic. **Section 7 lists the v5.1 changes, which supersede parts of sections 1, 2
and 4** (marked where they do). `motorsworking_v5.ino` is the v5.1 source.

The .ino also contains behaviour that was never listed here and predates v5.0:
STOP releases the drivers, the 30-minute hold, the -140..+180 azimuth range with
the mandatory elevation flip below -140, MOVE running on the DANCE speed profile
(50 dps, 45 dps2), and the autonomous "educated re-cal" (now disabled, section 7).

The fake Arduino in `sim/fake_arduino.py` models the protocol only. It does NOT
model sun routing, the azimuth range, fault latching, STOP releasing the drivers
or the real move timing, so passing `tests/test_bridge.py` is not proof that the
board behaves. The acceptance run at the end of this file is.

Line numbers refer to v4.4 as uploaded 2026-10-17.

## 1. Sequence number + checksum, ACK/NAK, de-duplication (replaces the sync bytes and the echo check)

Every host line is `<CMD> [args] #<seq>*<cs>`; `cs` = XOR of every byte before `*`, two hex digits
(**v5.1: CRC-16, four hex digits — section 7**; the code below is the v5.0 text).
A line that fails the check is answered `NAK <seq> <reason>` and never executed. A line whose
`seq` and text equal the last executed one is answered `ACK <seq>` and not executed again.

Add near the protocol codes (after line 276):

```cpp
#define PY_ERR_SETTLE     58   // bounded settle failed: "ERR 58 settle <el_err> <az_err>"
#define PY_ERR_SUN_UNKNOWN 59  // reserved: host reports sun unknown
#define PY_ERR_BUSY       63   // was ERR 50 "busy"
#define PY_ERR_NOT_HOMED  64   // was ERR 50 "not homed"
#define PY_ERR_FAULT      65   // was ERR 50 "fault latched"
#define PY_ERR_LINE_LONG  66   // was ERR 50 "line too long"
#define PY_ERR_RESET      67   // was ERR 50 "RESET: ..."
#define SETTLE_MAX_PASSES 3
#define RX_QUIET_MS       50   // pause POS/TGT/HOME_LOG this long after any received byte
```

Replace `handleLine()` (lines 2199-2211) with:

```cpp
static uint8_t lastSeq = 255;
static char    lastLine[64] = "";
static unsigned long rxQuietUntil = 0;

/* Validate "#<seq>*<cs>", strip it from the line, return the seq. */
static bool parseSeqChecksum(char *line, uint8_t &seq, int &looseSeq) {
  looseSeq = -1;
  char *star = strrchr(line, '*');
  char *hash = strrchr(line, '#');
  if (hash) looseSeq = atoi(hash + 1);
  if (!star || !hash || hash > star) return false;
  uint8_t cs = 0;
  for (char *p = line; p < star; ++p) cs ^= (uint8_t)*p;
  if (strlen(star + 1) != 2) return false;
  if (cs != (uint8_t)strtol(star + 1, NULL, 16)) return false;
  int s = atoi(hash + 1);
  if (s < 0 || s > 255) return false;
  seq = (uint8_t)s;
  *hash = '\0';                                   // command text only
  while (hash > line && hash[-1] == ' ') *--hash = '\0';
  return true;
}

void handleLine(char *line) {
  lastCmdMs = millis();
  if (holdTrimming) { axisStop(m1); axisStop(m2); holdTrimming = false; }
  Serial.print(F("RX: ")); Serial.println(line);   // human echo; the host ignores it

  uint8_t seq; int loose;
  if (!parseSeqChecksum(line, seq, loose)) {
    Serial.print(F("NAK ")); Serial.print(loose); Serial.println(F(" checksum"));
    return;                                        // NEVER execute a damaged line
  }
  if (seq == lastSeq && !strcmp(line, lastLine)) { // resend after a lost ACK
    Serial.print(F("ACK ")); Serial.println(seq);
    return;                                        // already executed: no ghost move
  }
  lastSeq = seq; strncpy(lastLine, line, sizeof(lastLine) - 1);
  Serial.print(F("ACK ")); Serial.println(seq);

  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  /* ... existing dispatch unchanged from here, with the code changes in section 2 ... */
```

`pollSerial()` keeps its non-printable filter (it is what rescues a line after a USB
wake). Add one line at the top of the `while (Serial.available())` loop body:

```cpp
    rxQuietUntil = millis() + RX_QUIET_MS;         // host is talking: hold our telemetry
```

and in `loop()`, guard the three telemetry emitters (lines 2608-2615, 2619-2631, 2634-2638):

```cpp
  bool rxQuiet = (long)(millis() - rxQuietUntil) >= 0;
  if (rxQuiet && sysState != ST_HOMING && ...) { /* heartbeat */ }
  if (rxQuiet && sysState == ST_HOMING && millis() - lastPosPushMs >= POS_STREAM_MS) { ... }
  if (rxQuiet && (sysState == ST_MOVING || sysState == ST_DANCING) && ...) { printPos(); printTgt(); }
```

The bridge sends no sync bytes any more; keep the filter anyway.

## 2. Distinct error codes

Mechanical replacement of every `pushErrMsg(PY_ERR_GENERAL, ...)` whose text is one of:

| Text (v4.4)                                      | Lines                               | v5 code |
| ------------------------------------------------ | ----------------------------------- | ------- |
| `busy`                                           | 2216, 2321, 2345, 2381, 2393, 2417  | `PY_ERR_BUSY` 63 |
| `not homed (send CAL)`                           | 2228, 2383, 2420                    | `PY_ERR_NOT_HOMED` 64 |
| `fault latched (send ACK)`                       | 2213, 2318, 2378, 2414              | `PY_ERR_FAULT` 65 |
| `line too long`                                  | 2460                                | `PY_ERR_LINE_LONG` 66 |
| `RESET: controller (re)booted ...`               | 2523                                | `PY_ERR_RESET` 67 (v5.0 never actually sent it; v5.1 does, section 7) |
| `position outside +/-180 - send CAL first`       | 2236                                | `PY_ERR_NOT_HOMED` 64 |

`PY_ERR_GENERAL` 50 stays for anything genuinely general. The host maps every code
one-to-one to `MNT-nn` (see `airis/errors.py`), so a grep for `MNT-63` finds every busy.

## 3. Bounded settle, no AZ nudge, DONE with residuals

Replace the settle block (lines 2685-2735) with:

```cpp
      } else if (sysState == ST_MOVING) {
        if (settleWaitEnd == 0) {
          settleWaitEnd = millis() + SETTLE_WAIT_MS; if (settleWaitEnd == 0) settleWaitEnd = 1;
        } else if ((long)(millis() - settleWaitEnd) >= 0) {
          float e1 = m1.targetDeg - encDeg(m1);
          float e2 = m2.targetDeg - encDeg(m2);
          bool ok = fabsf(e1) <= SETTLE_TOL_DEG && fabsf(e2) <= SETTLE_TOL_DEG;
          if (ok) {
            settleWaitEnd = 0; sysState = ST_IDLE;
            Serial.print(F("DONE ")); Serial.print(e1, 3); Serial.print(' '); Serial.println(e2, 3);
            printPos(); pushStatus(PY_IDLE);
          } else if (++settlePasses > SETTLE_MAX_PASSES) {
            settleWaitEnd = 0; sysState = ST_IDLE;         // give up honestly; the host decides
            Serial.print(F("ERR ")); Serial.print(PY_ERR_SETTLE);
            Serial.print(F(" settle ")); Serial.print(e1, 3); Serial.print(' '); Serial.println(e2, 3);
            pushStatus(PY_IDLE);
          } else {
            settleWaitEnd = 0;                              // one more trim, SAME targets, no other axis
            if (fabsf(e1) > SETTLE_TOL_DEG) axisStartMoveDeg(m1, e1, TRIM_SPEED_DPS, TRIM_ACCEL_DPS2, false, false);
            if (fabsf(e2) > SETTLE_TOL_DEG) axisStartMoveDeg(m2, e2, TRIM_SPEED_DPS, TRIM_ACCEL_DPS2, false, false);
          }
        }
      }
```

Delete `STUCK_EL_PASSES`, `NUDGE_AZ_DEG`, `stuckElCount`, `nudgeDir` and the `PERR` line
(the residual now travels in `DONE`). An EL snag during a MOVE is reported, not worked around:
also turn the snag watch ON for pointing legs in `startSlewTo()` (`watch=true` in both
`axisStartMoveDeg` calls) so a stall halts within 200 ms instead of being discovered at settle.

## 4. Live sun center

**Superseded by v5.1 (section 7): SUN is taken in the physical-home frame and stored
as given.** The v5.0 text below subtracted the frame offsets, which was wrong for
every push that arrived before SETZERO.

Line 670-671: `const float sunEl = ...; const float sunAz = ...;` -> `float sunEl = ...; float sunAz = ...;`.
Replace the `SUN <el> <az>` ignore branch (lines 2403-2412) with:

```cpp
    if (a1 && a2) {
      float el = atof(a1), az = atof(a2);              // in the CURRENT encoder frame
      sunEl = el - frameElOff;                          // store in the physical frame
      sunAz = wrap180(az - frameAzOff);
      Serial.print(F("SUN OK ")); Serial.print(el, 3); Serial.print(' '); Serial.println(az, 3);
      return;
    }
```

Everything downstream (`sunElF()`, `sunAzF()`, the box, the detour planner, the flip check)
already reads `sunEl/sunAz` through the frame offsets, so nothing else changes. The host
pushes `SUN` every 60 s from GPS + UTC + heading (`airis_main.update_sun`).

## 5. CAL confirmation and anchoring

No firmware change: CAL already pushes `STATUS 12` at once and `STATUS 13`, `STATUS 0` at the
end; `SETZERO` already prints `ZERO SET`. The bridge now sends the missing
`MOVE -55.000 -35.000` + `SETZERO` after every CAL (`airis_bridge.Bridge._calibrate_locked`).
If the team decides the index IS the home, set `HOME_EL_OFFSET_DEG` / `HOME_AZ_OFFSET_DEG`
to 0 in BOTH the firmware and `airis/config.py` and re-measure `SUN_CENTER_*`.

## 6. Keep, but unused by the host

`DANCE`, `MOTOROFF`, `POS`, `CALQ`, `HOME_LOG`, `TGT` stay. Drop `POS_STREAM_MS` to 200 ms
(5 Hz) to lighten the link; the bridge never reads the stream.

## 7. v5.1 changes

The host (`airis/`) was changed in step; flash and deploy together.

1. **Checksum is CRC-16/CCITT-FALSE, four hex digits** (`crc16()`, poly 0x1021, init
   0xFFFF; `"123456789"` gives `29B1`, `POS #7` gives `7735`). The one-byte XOR accepted
   `MOVE 1.000` for `MOVE 100.000` (two identical dropped bytes cancel). A v5.0 host
   or firmware on either side now gets `NAK` for every line.
2. **`SUN <el> <az>` is in the physical-home frame, always**, and is stored as given.
   Out-of-range or NaN values are refused with `ERR 61` and the keep-out is unchanged
   (a NaN center made every box test false: no keep-out).
3. **`ERR 67 RESET: ...` is sent once from `setup()`.** The host treats it as "not
   homed, sun center back at the compile-time default" and will not send CAL until
   it has pushed SUN again.
4. **The educated re-cal is off (`MOVES_PER_RECAL 0`).** After the host's SETZERO the
   index is at -HOME_*_OFFSET, not (0, 0), so the sweep around home always missed,
   fell back to a full homing and left the encoders in the index frame while the
   host kept commanding in the physical frame.
5. **A sun-detour leg must arrive before the next one starts** (`SUN_WP_TOL_DEG`, 3 deg;
   bench-tune it). A stalled or short leg aborts the move with
   `ERR 58 settle <el_err> <az_err> sun-detour waypoint not reached`. The snag watch
   now stays on for every detour leg.
6. **After `ERR 58` the active hold is anchored where the mount is**, not left on the
   unreached target (it used to re-trim toward it every 500 ms, unwatched).
7. **SETZERO is safe to repeat**: when the frame is already physical it answers
   `ZERO SET` without shifting again. It also shifts the index re-zero reference, so
   the continuous index re-zero works after SETZERO. That drift control had never
   run with non-zero home offsets; watch for `IDX ... re-zeroed` lines on the bench.
8. **`BENCH_COMMANDS`** (default 1) gates `RAW` and bare `SUN` (slew straight at the
   sun, keep-out bypassed). **Set it to 0 for the flight build**; both then answer `ERR 60`.

Not changed, and still open: `HOME_SPEED_DPS` is 200 while the comments say AZ lost
sync at 135; STOP still releases the drivers; the sun box is a fixed el/az rectangle.

## Acceptance

With the Mega flashed and `python -m airis.airis_bridge --socket /tmp/b.sock` running,
`python -m tools.bridge_smoke /tmp/b.sock` must print `sun ok`, `cal ok`,
`move ok el_err az_err dt`, `busy MNT-63`, in that order. The bridge refuses to home
(`MNT-59`) until a sun position has been pushed.
