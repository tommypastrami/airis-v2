# Firmware v5 changes (motorsworking_715.ino v4.4 -> v5.0)

Six changes, all in the serial/command layer and the settle logic. The motion
profile, homing, snag ladder and sun geometry are untouched. The fake Arduino in
`sim/fake_arduino.py` implements exactly this behaviour, and `tests/test_bridge.py`
is the acceptance test for the real board: run the bridge against the Mega with
`python -m airis.airis_bridge` and the same requests must produce the same replies.

Line numbers refer to v4.4 as uploaded 2026-10-17.

## 1. Sequence number + checksum, ACK/NAK, de-duplication (replaces the sync bytes and the echo check)

Every host line is `<CMD> [args] #<seq>*<cs>`; `cs` = XOR of every byte before `*`, two hex digits.
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
| `RESET: controller (re)booted ...`               | 2523                                | `PY_ERR_RESET` 67 |
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

## Acceptance

With the Mega flashed and `python -m airis.airis_bridge --socket /tmp/b.sock` running,
`python -m tools.bridge_smoke /tmp/b.sock` must print `cal ok`, `move ok el_err az_err dt`,
`busy MNT-63`, `sun ok`, in that order.
