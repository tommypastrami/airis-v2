/***************************************************************************
 *  AIRIS Mount Controller — Arduino Mega 2560, two-axis stepper mount
 *  M1 = Elevation, M2 = Azimuth              FW v5.0
 *
 *  Hardware (unchanged, wiring identical to v3.x):
 *    2x DM556T step/dir drivers
 *    2x NEMA 23 steppers
 *    2x USA Digital 2500 PPR quadrature encoders w/ index (Z) channel
 *
 *  ------------------------------------------------------------------
 *  HOST COMPATIBILITY — DO NOT CHANGE WITHOUT UPDATING airis_bridge.py
 *  ------------------------------------------------------------------
 *  Serial: 115200 baud, one ASCII command per line, case-insensitive.
 *
 *  LINE FRAMING (v5): every host line is  "<CMD> [args] #<seq>*<cs>"
 *      seq = 0..255, cs = XOR of every byte before '*', two hex digits.
 *      A line that fails the check  -> "NAK <seq> checksum", NOT executed.
 *      A line whose seq AND text equal the last executed line (a resend
 *      after a lost ACK) -> "ACK <seq>", NOT executed again (no ghost move).
 *      Otherwise -> "ACK <seq>" and the command runs.
 *      (Replaces the v4.x 4-byte sync header and link-recovery machinery.)
 *
 *  The Jetson host (airis_bridge.py) machine-parses ONLY these outputs:
 *      "ACK <seq>" / "NAK <seq> <reason>"  per received line
 *      "STATUS <n> <NAME>"   pushed on change and on request
 *      "ERR <n> <msg>"       any n >= 50; the host replies ACK
 *      "DONE <el_err> <az_err>"  move settled; residuals in deg
 *      "POS <el> <az>"       on POS request and streamed 5 Hz mid-slew
 *      "ZERO SET"            SETZERO confirmation (substring match)
 *      "SUN OK <el> <az>"    SUN <el> <az> accepted
 *      "HOME_LOG <9 fields>" optional debug rows captured to CSV during CAL
 *  Everything else (RX:, VERIFY..., HELP text) is logged or ignored by the
 *  host, but the texts are kept stable for humans/logs. Unsolicited
 *  telemetry (POS/TGT/HOME_LOG/heartbeat) is held for RX_QUIET_MS after
 *  any received byte so it never collides with a host transaction.
 *
 *  Commands (identical set to v3.x):
 *    MOVE <m1> <m2>   slew to absolute angles (m1 = elevation, m2 = azimuth)
 *    STOP             halt all motion (works mid-slew; jerk-limited ramp)
 *    CAL [az] / HOME  home both motors. A numeric arg is accepted for
 *                     protocol compatibility but IGNORED — the sun-safe
 *                     park is always computed from the firmware sun center
 *    SETZERO          re-anchor encoder zero at the current physical position
 *    ACK              clear a latched error
 *    STATUS           report status code
 *    POS              report position -> POS <el> <az>
 *    ID               report firmware id
 *    DANCE            perform a little routine (requires CAL)
 *    RAW <1|2|both> <cw|c> <dps> <sec>  BENCH: blocking open-loop run of the
 *                     selected motor(s) — no homing/limit/sun/snag checks,
 *                     no ramps; reports encoder change when the time is up
 *    MOTOROFF         de-energize the drivers (host idle timeout)
 *    SUN [<el> <az>]  with args: set the sun keep-out center (values in the
 *                     CURRENT encoder frame; stored in the physical frame).
 *                     The host pushes this every 60 s from GPS+UTC+heading.
 *                     Bare SUN = TEST point directly at the current center
 *    HELP             reprint this list
 *
 *  Status / error codes (must match airis/errors.py exactly; host maps
 *  each to MNT-nn):
 *    0 IDLE   11 HOMING_M1   12 HOMING_M2   13 HOMING_COMPLETE   15 SLEWING
 *    50 GENERAL  51/52 HOMING_M1/M2 (index never found)
 *    53/54 CATCH_M1/M2 (cable catch: motion blocked even after a punch)
 *    55/56 STEP_LIMIT  57 SUN_FACING  58 SETTLE (bounded settle failed)
 *    59 SUN_UNKNOWN (reserved)  60 CMD_UNREC  61 INVALID_ANGLE
 *    62 BYTE_UNDERFLOW  63 BUSY  64 NOT_HOMED  65 FAULT_LATCHED
 *    66 LINE_TOO_LONG  67 RESET (reserved)
 *
 *  ------------------------------------------------------------------
 *  COORDINATE SYSTEM — read this first
 *  ------------------------------------------------------------------
 *  Two axes, both in DEGREES relative to the PHYSICAL HOME (see anchoring
 *  below). All MOVE / POS / SUN values live in this one frame.
 *
 *    el  (M1, elevation): 0 at home, positive raises the boresight.
 *        Mechanical range -180..+180. |el| <= 90 is the normal hemisphere;
 *        el beyond +/-90 tips the camera over the top — a valid attitude.
 *    az  (M2, azimuth):   0 at home, LIMITED, ASYMMETRIC ROM of -140..+180
 *        (320 deg total; M2_MIN/MAX_DEG). The band below -140 (down to -180)
 *        is OUTLAWED: those azimuths are unreachable directly and are instead
 *        served by the elevation flip (see below), which reflects the az by
 *        180 back inside the ROM. NO wrap-around (cable management):
 *        commanding +130 then -130 is a 260 deg swing back through 0, never a
 *        100 deg step across the seam. Only azimuth is ROM-limited; elevation
 *        keeps its full -180..+180 range.
 *
 *  TWO NAMES FOR EVERY SKY DIRECTION (camera flip):
 *    Because el can pass over the top, every boresight direction has
 *    exactly two mount attitudes:
 *        (el, az)  ==  (180 - el, az + 180)          for el >= 0
 *        (el, az)  ==  (-180 - el, az + 180)         for el <  0
 *        (az + 180 wrapped into +/-180)
 *    Examples: (30, 170) == (150, -10);  (100, 100) == (80, -80);
 *    and notably (180, 180) == (0, 0) — commanding el 180 as a "range
 *    test" will be optimized into (almost) no motion, because it IS the
 *    same pointing. Use below-horizon targets or DANCE to exercise the
 *    mechanism through large angles.
 *
 *  HOW TO REFERENCE IT:
 *    - MOVE accepts EITHER representation (any values inside +/-180).
 *    - OUTLAWED AZIMUTH (az in -180..-140): the azimuth ROM cannot reach it,
 *      so MOVE ALWAYS executes the elevation flip to point at the same sky
 *      direction with the azimuth reflected back inside -140..+180 ("AZ-LIMIT:
 *      ..." is logged). This flip is mandatory and applies to sky AND
 *      below-horizon targets alike (it is the only way to reach that
 *      direction), so it is the one exception to the "below-horizon never
 *      flips" rule below.
 *    - For sky targets (el >= 0) with in-ROM azimuth the firmware executes
 *      whichever attitude is faster to reach ("FLIP: ..." is logged when it
 *      takes the flip).
 *    - POS reports the representation CLOSEST TO YOUR LAST COMMANDED
 *      TARGET, so readback and error arithmetic stay in your frame:
 *      command (100, 100) -> POS reads ~(100, 100) even if the mount
 *      physically sits at (80, -80). A commanded outlawed az reads back at
 *      the commanded (outlawed) value too, even though the mount sits flipped.
 *    - Below-horizon targets (el < 0) with IN-ROM azimuth are literal
 *      mechanical positioning: never flipped, executed exactly as commanded.
 *      (The host's post-CAL offset drive before SETZERO relies on this; it
 *      uses az well inside the ROM, so it never triggers the outlawed flip.)
 *    - The DONE residuals, TGT and HOME_LOG stay in the raw mechanical frame
 *      (servo error and homing diagnostics respectively).
 *
 *  FRAME ANCHORING (CAL -> offset MOVE -> SETZERO) — REPEATABLE BY DESIGN:
 *    CAL zeroes each encoder on its index pulse, always re-capturing the
 *    SAME edge of the pulse from the SAME approach direction. The host
 *    then MOVEs by the index-to-home offset (HOME_EL/AZ_OFFSET_DEG, which
 *    MUST match airis_main) and sends SETZERO. SETZERO anchors the frame
 *    by ARITHMETIC — encoder counts shift by exactly the offsets — NOT by
 *    blessing wherever the mount parked, so encoder (0,0) lands on the
 *    identical physical point every calibration (parking error within the
 *    settle tolerance just reads back honestly on POS). Everything — soft
 *    limits, the hard watchdog, the sun keep-out — is measured from that
 *    zero. Between an axis's index-zeroing and SETZERO the frame is the
 *    INDEX frame; the firmware transforms all sun geometry through the
 *    frame offsets so CAL's parks and sweeps guard the REAL sun there too.
 *
 *  ------------------------------------------------------------------
 *  Motion profile (why it doesn't wobble any more)
 *  ------------------------------------------------------------------
 *  v3.x used a cubic-smoothstep velocity profile. Its acceleration is
 *  continuous, but its JERK is discontinuous at the start and end of each
 *  ramp — the decel ramp ends with a step change in jerk exactly at the
 *  stop point, which rings the frame and is what you saw as end-of-move
 *  wobble. v4 changes two things:
 *
 *   1. QUINTIC ("smootherstep") velocity ramps: v(t) = Vp*q(x),
 *      q(x) = 10x^3 - 15x^4 + 6x^5.  q'(0)=q'(1)=0 (zero accel at ramp
 *      endpoints, same as before) AND q''(0)=q''(1)=0 — jerk is now zero
 *      and CONTINUOUS at the stop point, so nothing excites the structure
 *      when motion ends. Still pure polynomial: no sqrt/sin in the
 *      per-step hot path, an AVR keeps up with the 1200 steps/s AZ rate.
 *
 *   2. ASYMMETRIC ramps: full accel getting up to speed, DECEL_RATIO of
 *      that slowing down. Braking torque disturbs the frame more than
 *      driving torque (the load "catches up" on the belt/coupling lash),
 *      so the decel is stretched while the accel stays aggressive.
 *
 *   3. CRAWL LANDING (v4.1): the killer of end-of-move oscillation. A
 *      pointing slew no longer decelerates to zero AT the target — that
 *      still dumps the decel-phase momentum into the frame right where
 *      accuracy matters. Instead the profile is five phases:
 *        accel -> cruise -> decel down to LAND_SPEED_DPS -> CRAWL through
 *        the final LAND_ZONE_DEG (3 deg) at that creep speed -> a very
 *        gentle ramp to zero (LAND_ACCEL_DPS2).
 *      One continuous, jerk-limited velocity curve: the transition from
 *      fast slewing to the tiny final movement is smooth, any structural
 *      ringing excited by the main decel damps out DURING the crawl, and
 *      the final stop happens from ~4 deg/s — negligible energy, nothing
 *      left to oscillate. Homing legs, trim moves and sun-detour corner
 *      waypoints skip the landing (corners don't need precision; homing
 *      is already slow), so only real pointing legs pay the ~1 s cost.
 *
 *  Timing budget (v4.2, half-speed slews for motor safety): 90 deg ~3.6 s,
 *  180 deg ~4.9 s. The camera-flip pointing caps the effective travel for
 *  sky targets near 180 deg, so worst-case SKY pointing stays ~5.1 s
 *  including the settle check. Below-horizon frame-positioning moves
 *  (never flipped) can take up to ~7.6 s for a full 360 — acceptable,
 *  they're not GRB-response moves. If pointing must be strictly < 5 s,
 *  raise SLEW_SPEED_DPS (the motors ran 2x this rate cleanly).
 *
 *  CAMERA-FLIP POINTING: (el, az) and (180-el, az+180) aim the boresight
 *  at the same sky direction. MOVE picks whichever is faster for sky
 *  targets (el >= 0). POS reports the CANONICAL boresight direction
 *  (|el| <= 90, relative to mechanical zero), so a commanded (30, 170)
 *  executed as a flip still reads back as (30, 170) and host error
 *  readouts stay accurate. The FLIP log line records the raw attitude;
 *  HOME_LOG and the DONE residuals stay in the raw frame. Sun safety checks BOTH
 *  representations of every target and path sample (a flipped attitude
 *  can stare at the sun while its coordinates sit far from the direct
 *  sun box).
 *
 *  ------------------------------------------------------------------
 *  Homing (CAL/HOME) — per axis
 *  ------------------------------------------------------------------
 *  Homing sweeps run FAST (HOME_SPEED_DPS = slew rate): bench experience
 *  showed slow sweeps stick-slip on the bearings — no momentum to carry
 *  past static friction, causing stuttering and false snag trips. Wire
 *  safety comes from the continuous snag watch (halts a real catch within
 *  ~0.2 s), and PRECISION comes from the slow index creep, not the sweep
 *  speed. Sequence per axis:
 *    1. If a previous zero is known, return to it first (stay inside the
 *       proven cable envelope), then sweep.
 *    2. Sweep 180 deg watching the index (EL sweeps POSITIVE first, AZ
 *       NEGATIVE first — M*_HOME_SWEEP_SIGN). If the sweep completes OR the
 *       axis catches (see snag policy below), reverse and sweep back far
 *       enough that total angular coverage reaches the axis field of view
 *       (M*_HOME_SWEEP_TOTAL_DEG): 365 deg for EL (a full rev + 5 deg margin,
 *       guaranteeing a crossing of the 1/rev index) but only 320 deg for AZ,
 *       whose -140..+180 ROM is all it has — the index must lie inside that arc.
 *    3. When the index fires, keep going 10 deg PAST it, then creep back
 *       at IDX_CREEP_DPS with the index re-armed. The slow re-capture is
 *       the real zero (the fast capture is smeared by ISR latency at
 *       sweep speed); the fast capture is kept as a fallback.
 *    4. Return to zero.
 *  If the reverse sweep finishes with no index -> ERR 51/52 (encoder or
 *  index wiring problem). If BOTH directions catch -> ERR 53/54.
 *
 *  CAL sequence (sun safety):
 *    1. EL blindly moves EL_BLIND_PARK_DEG (-45; no calibrated frame yet).
 *       If that direction properly snags, EL goes to
 *       EL_BLIND_PARK_RETRY_DEG (+315) from the initial rest position
 *       instead — the same physical attitude reached the long way around;
 *       only if BOTH directions are blocked does CAL fault (ERR 53).
 *    2. AZ homes
 *    3. AZ turns 90 deg away from the sun center (computed from the
 *       firmware's own sun keep-out center; a host-supplied "CAL <az>"
 *       overrides it — the host computes the identical value)
 *    4. EL homes (safe: AZ is 90 deg off-sun)
 *    5. EL validation sweep (0 -> +180 -> -180 -> 0)
 *    6. EL parks at EL_AZVAL_PARK_DEG (below the sun window)
 *    7. AZ verification is REMOVED (user spec) — AZ is NOT range-swept; it
 *       simply returns to home (0) so the mount ends CAL at a known place.
 *
 *  ------------------------------------------------------------------
 *  Snag ("hit") policy — continuous detection, applied to ALL homing and
 *  verification legs
 *  ------------------------------------------------------------------
 *  A hit = the axis ESSENTIALLY STOPPED while being commanded to move:
 *  200 ms of near-zero encoder rate (< SNAG_STALL_DPS) while the command
 *  advances (> SNAG_CMD_MIN_DPS). Lag alone never trips — a motor that
 *  keeps advancing, however raggedly, is not caught. Legs that finish far
 *  from their target without ever stalling are caught by the end-of-leg
 *  check (VERIFY_END_TOL_DEG) on parks and verify sweeps. On a hit:
 *    1. Halt, back off 10 deg (away from the snag, at the leg speed).
 *    2. Punch (AZIMUTH ONLY): retry toward the target at SNAG_PUNCH_DPS
 *       (135 dps, above the homing rate) — the momentum carries the wire
 *       past a small obstruction. ELEVATION never punches
 *       (M1_SNAG_PUNCH_TRIES = 0): an EL hit is a proper snag immediately,
 *       never rammed at speed.
 *    3. If the punch also stalls (or the axis has no punch), it is a
 *       PROPER SNAG. What that means depends on where it happened:
 *         - forward homing search  -> just reverse (that direction is done)
 *         - reverse homing search  -> ERR 53/54 (blocked both ways)
 *         - verification sweep     -> return to zero, unwind the axis field
 *           of view AWAY from the snag direction (365 deg EL / 320 deg AZ,
 *           limits bypassed while homing), then re-run this axis's full
 *           homing + verification. After VERIFY_MAX_RETRY failed recoveries
 *           -> ERR 53/54.
 *
 *  ------------------------------------------------------------------
 *  Post-home verification (EL ONLY — CAL steps 5-7 above)
 *  ------------------------------------------------------------------
 *  Only ELEVATION is range-verified. EL validates its full +/-180 sweep (AZ
 *  parked 90 deg off-sun so the sweep never crosses the sun window), then EL
 *  parks below the window and AZ simply RETURNS TO HOME — AZ range
 *  verification is removed (user spec: the -140..+180 azimuth ROM is trusted, not
 *  swept). A leg that completes proves the encoder tracked the whole way (the
 *  continuous snag watch enforces it), so no separate end-of-leg position
 *  check is needed. EL clean + AZ home -> HOMED, STATUS 13, STATUS 0.
 *
 *  ------------------------------------------------------------------
 *  Sun keep-out — FIRMWARE-OWNED (the host does NO sun planning)
 *  ------------------------------------------------------------------
 *  Box centered on (sunEl, sunAz), spanning SUN_WIN_* inflated by half the
 *  camera FOV plus SUN_CLEAR_DEG. MOVE targets inside the box are refused
 *  (ERR 57). If the direct path would cross the box, the move is staged:
 *    1. EL moves as close to its final value as it can but STOPS outside
 *       the box's elevation band;
 *    2. AZ moves until it is past/clear of the box's azimuth band;
 *    3. only then does EL finish.
 *  If the current elevation is INSIDE the el band, AZ is not allowed near
 *  the box at all until EL has exited the band (the pop-out leg runs
 *  first). Every generated leg moves a single axis, so the concurrent-axis
 *  diagonal can never cut a corner into the window, and every leg is
 *  re-verified against the sampled path model before the move starts.
 *
 *  The sun center boots at the compile-time default (SUN_CENTER_EL/AZ_DEG,
 *  bench-measured 2026-07-15) and is then kept LIVE by the host:
 *  "SUN <el> <az>" (CURRENT encoder frame) updates it — the host pushes it
 *  every 60 s from GPS + UTC + gondola heading. It is stored in the
 *  physical frame, so it survives SETZERO. Bare "SUN" still test-points
 *  at the current center.
 ***************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <math.h>

#define FW_ID "MEGA2560-2AXIS-STEP v5.0 (AIRIS: seq+checksum ACK/NAK, dedup, distinct err codes, bounded settle, DONE residuals, live SUN, rx-quiet telemetry, STOP releases motors, 30-min hold, az-norm, az-120-160-bias, az-ROM-140-to-180, az-320-homing, az-verify-removed, outlaw-az-flip)"

/* ---------------- Host (Jetson AIRIS controls) protocol codes ---------- */
#define PY_IDLE          0
#define PY_HOMING_M1     11
#define PY_HOMING_M2     12
#define PY_HOMING_DONE   13
#define PY_SLEWING       15
#define PY_ERR_GENERAL   50
#define PY_ERR_HOME_M1   51
#define PY_ERR_HOME_M2   52
#define PY_ERR_CATCH_M1  53   // cable catch on M1 (was defined host-side, now used)
#define PY_ERR_CATCH_M2  54   // cable catch on M2
#define PY_ERR_LIMIT_M1  55
#define PY_ERR_LIMIT_M2  56
#define PY_ERR_SUN       57
#define PY_ERR_SETTLE     58   // bounded settle failed: "ERR 58 settle <el_err> <az_err>"
#define PY_ERR_SUN_UNKNOWN 59  // reserved: host reports sun unknown
#define PY_ERR_CMD       60
#define PY_ERR_ANGLE     61
#define PY_ERR_UNDERFLOW 62
#define PY_ERR_BUSY       63   // was ERR 50 "busy"
#define PY_ERR_NOT_HOMED  64   // was ERR 50 "not homed"
#define PY_ERR_FAULT      65   // was ERR 50 "fault latched"
#define PY_ERR_LINE_LONG  66   // was ERR 50 "line too long"
#define PY_ERR_RESET      67   // reserved: "RESET: ..." (no reset report in this build)
#define SETTLE_MAX_PASSES 3    // closed-loop trim passes before ERR 58
#define RX_QUIET_MS       50   // pause POS/TGT/HOME_LOG this long after any received byte

/* ------------------------------ Pins (unchanged) ----------------------- */
#define M1_PULSE_PIN     4
#define M1_DIR_PIN       7
#define M1_ENC_A_PIN     18
#define M1_ENC_B_PIN     19
#define M1_ENC_INDEX_PIN 20

#define M2_PULSE_PIN     11
#define M2_DIR_PIN       8
#define M2_ENC_A_PIN     2
#define M2_ENC_B_PIN     3
#define M2_ENC_INDEX_PIN 21

#define M1_ENA_PIN       23      // ENA+ driver 1
#define M2_ENA_PIN       25      // ENA+ driver 2
// DM556T with ENA- tied to GND: energizing the ENA opto DISABLES the driver
// (pin HIGH = motor free, pin LOW = motor energized). Flip if wired opposite.
#define ENA_DISABLE_LEVEL HIGH
#define ENA_SETTLE_MS    5       // driver wake-up time before the first pulse

/* --------------------------- Mechanics --------------------------------- */
#define M1_STEPS_PER_REV 1600UL  // MUST match each DM556T DIP microstep
#define M2_STEPS_PER_REV 3200UL  // setting INDIVIDUALLY (they differ!)
#define ENC_CPR         10000L   // 2500 PPR x4 quadrature decode

#define M1_INVERT_DIR   false    // flip if the axis moves the wrong way
#define M2_INVERT_DIR   true
#define M1_INVERT_ENC   false    // flip if the encoder counts opposite to
#define M2_INVERT_ENC   false    // the motor (positive steps MUST increase
                                 // the encoder count — everything closed-
                                 // loop here relies on that convention)

#define PULSE_WIDTH_US  4        // DM556T needs >= 2.5 us
#define DIR_SETUP_US    10       // dir-to-pulse setup time

/* --------------------------- Motion tuning ----------------------------- *
 * ALL speeds/accels are in DEGREES so both motors move at the same
 * physical rate despite different microstep settings; they are converted
 * to steps/s per axis at move start (M2 automatically gets 2x the step
 * rate of M1). SLEW values below are the exact angular rates the proven
 * v3.x firmware ran (600/1200 steps/s), just expressed in degrees.       */
#define SLEW_SPEED_DPS    67.5f  // MOVE slew speed. Half the proven 135 dps:
#define SLEW_ACCEL_DPS2  112.5f  // motor/frame loads scale with speed^2, so
                                 // halving speed AND accel quarters the
                                 // kinetic energy and halves peak torque —
                                 // much easier on the motors. The camera-flip
                                 // pointing (below) keeps sky slews inside
                                 // the time budget despite the slower rate.
#define DECEL_RATIO      0.75f   // decel peak = this fraction of accel peak.
                                 // <1 stretches the braking ramp. Since the
                                 // crawl landing (below) now owns the final
                                 // stop, this only shapes the fast part of
                                 // the decel; 0.75 balances smoothness vs
                                 // the 5 s pointing budget.

/* ---- Crawl landing (end-of-move oscillation killer) ----
 * Pointing slews decelerate to LAND_SPEED_DPS, creep through the final
 * LAND_ZONE_DEG, then ramp to zero at LAND_ACCEL_DPS2. Stopping from a few
 * deg/s carries ~1/1000th the kinetic energy of stopping from slew speed,
 * and the frame's ringing from the main decel damps out during the crawl.
 * The landing costs ~1 s per pointing move; homing legs, trims, and
 * sun-detour corner waypoints skip it. */
#define LAND_ZONE_DEG     3.0f   // "super slow" final approach zone
#define LAND_SPEED_DPS    4.0f   // crawl speed through the zone
#define LAND_ACCEL_DPS2  15.0f   // gentleness of the very last ramp to zero

#define HOME_SPEED_DPS   200.0f   // homing/verify sweeps and CAL parks. FAST
                                 // on purpose (= slew rate): at low speed the
                                 // bearings stick-slip — the axis stutters
                                 // go-stop-go and the encoder lag falsely
                                 // trips the snag watch. Momentum carries it
                                 // smoothly over the friction; the PRECISION
                                 // still comes from the slow index creep
                                 // (IDX_CREEP_DPS), which is unchanged. Wire
                                 // safety is preserved by the continuous
                                 // snag watch: a real catch halts within
                                 // ~0.2 s (~13 deg of pull) even at this rate.
#define VERIFY_SPEED_DPS 90.0f   // the +/-180 verification sweeps (and the
                                 // rest of the verify machine): standard
                                 // open-loop profile moves, stall watchdog
                                 // armed, at UNDER HALF the normal movement
                                 // speed (user spec) — comfortably inside
                                 // the axis's always-works band
#define HOME_ACCEL_DPS2  200.0f  // homing peak accel. Deliberately BELOW the
                                 // slew accel: acceleration torque is this
                                 // axis's peak demand — bench logs show the
                                 // encoder falling behind DURING the ramp at
                                 // 225 — while the stiction band (<~5 dps)
                                 // is still crossed in ~0.1 s. Gentle up,
                                 // full cruise speed.

#define IDX_OVERSHOOT_DEG 10.0f  // continue this far past the index pulse...
#define IDX_RESWEEP_DEG   12.0f  // ...then creep back up to this far (2 deg
                                 // margin past the overshoot so the pulse is
                                 // always bracketed)
#define QCAL_WIN_DEG       5.0f  // quick re-cal (CALQ): half-width of the
                                 // index search window around the host's
                                 // hint. After a link-death reset the mount
                                 // hasn't moved, so the index must be within
                                 // a few degrees of where the old frame put
                                 // it; if it isn't, the hint is wrong and
                                 // the full CAL takes over.
#define IDX_CREEP_DPS      2.0f  // re-capture creep rate: slow enough that
                                 // ISR latency (~us) is < 0.01 deg of error

/* ---- Encoder service + drift control ----
 * The quadrature ISRs update enc on every edge (finer than any poll), but a
 * 1 kHz service samples the encoders, derives travel direction, and applies
 * the two anti-drift mechanisms below. Runs from loop() via micros(). */
#define ENC_SERVICE_US    1000UL // 1 ms (1 kHz) encoder service period
/* Index re-zero: whenever the axis physically crosses the home index pulse
 * during normal motion, snap the frame so the index reads exactly its
 * calibrated count — this erases accumulated open-loop/quadrature drift once
 * per pass through home. Applied ONLY on the canonical travel direction
 * (encoder-negative, the same edge homing zeroed on: the pulse has physical
 * width, so the opposite edge sits one width away and would shift home). A
 * capture whose implied correction exceeds this tolerance is treated as
 * spurious and ignored. */
#define INDEX_REZERO_TOL_DEG 3.0f
/* Educated re-cal cadence: after this many completed moves, autonomously run
 * a quick re-cal (go to where 0,0 should be, sweep +/-QCAL_WIN_DEG for the
 * index; on a miss fall back to a full CAL with the +/-180 validation
 * skipped). Keeps drift bounded without a full 2-min CAL every time. */
#define MOVES_PER_RECAL      3

/* ---- Snag ("hit") detection ----
 * A catch means the axis has ESSENTIALLY STOPPED while being commanded to
 * move — NOT merely lagging some degrees behind the profile (user spec: a
 * motor that keeps advancing, however raggedly, is not caught). Every
 * SNAG_CHECK_MS the along-travel encoder rate is compared to the commanded
 * rate; SNAG_STALL_CHECKS consecutive samples with the command advancing
 * (> SNAG_CMD_MIN_DPS) while the encoder is essentially still
 * (< SNAG_STALL_DPS) trip the halt. Legs that finish with the axis far
 * from their target despite never stalling are caught separately by the
 * end-of-leg check (VERIFY_END_TOL_DEG) on parks and verify sweeps. */
#define SNAG_CHECK_MS    25      // rate-sample cadence while a watched leg runs
#define SNAG_STALL_DPS    3.0f   // encoder below this = essentially stopped
#define SNAG_CMD_MIN_DPS 10.0f   // only judge while commanding at least this
                                 // (creep moves are never judged — too slow)
#define SNAG_STALL_CHECKS 8      // 8 x 25 ms = 200 ms of stall to trip
#define VERIFY_END_TOL_DEG 15.0f // parks & verify sweeps: leg completed but
                                 // axis this far from target = treat as snag
#define SNAG_REST_MS   400UL     // full-stop rest after a snag halt before the
                                 // backoff, and after the backoff before the
                                 // punch. Lets the rotor and mechanics settle
                                 // so the motor re-syncs cleanly from rest —
                                 // no more go-stop-go stumbling after a trip
#define SNAG_BACKOFF_DEG 10.0f   // back off this far before the punch (user spec)
#define SNAG_PUNCH_DPS   90.0f   // punch speed — above the homing rate for
                                 // momentum (user spec: "push through at a
                                 // higher speed"), but INSIDE this axis's
                                 // real ability: at 135 the AZ motor made
                                 // 50 deg and lost sync, so a 135 punch was
                                 // a recovery step that could never succeed.
/* Final recovery rung: after a failed punch, retry the leg ONCE at a slow
 * mid-band speed — above the stiction band (~22 dps stutters), far below
 * the dropout band. The hardware has a speed range where it always works
 * ("spins fine at a reasonable speed"); this rung lives in it. A transient
 * motor dropout recovers here; a REAL cable stop fails all three rungs
 * (nominal, punch, slow) and still faults properly. AZ only, like the
 * punch — an EL snag stays immediately final. */
#define SNAG_SLOW_DPS    40.0f
#define M1_SNAG_SLOW_TRIES 0     // EL: no slow retry — if it snags it snags
#define M2_SNAG_SLOW_TRIES 1     // AZ: one slow attempt after the punch
/* Progress-earned punch retries (user spec): if a punch gains more than
 * PUNCH_PROGRESS_DEG of real encoder travel before stalling, the
 * obstruction is YIELDING — refund the punch and try again instead of
 * counting it against the allowance. A true hard stop gains < 1 deg and
 * faults on the normal ladder. Capped at 5 per leg (user spec) so a
 * slowly-grinding obstruction can't stall the sequence indefinitely
 * (each retry cycle is ~2 s, so <= ~10 s added worst case). */
#define PUNCH_PROGRESS_DEG 1.0f
#define PUNCH_PROGRESS_RETRIES_MAX 2
/* Punch attempts per leg, PER AXIS, before a hit is a proper snag.
 * EL is 0 by design: an elevation snag is treated as real immediately —
 * never rammed at speed. AZ keeps one backoff+punch attempt. ONE exception:
 * the blind-park +315 retry leg (the last resort before a CAL fault)
 * overrides EL to a single punch — see calService C_EL_PARK. */
#define M1_SNAG_PUNCH_TRIES 0    // EL: no punch-through — if it snags it snags
#define M2_SNAG_PUNCH_TRIES 1    // AZ: one backoff + punch attempt

/* ---- Homing sweep geometry ---- */
#define HOME_SWEEP_FWD_DEG   180.0f // first search leg (user spec)
/* Per-axis first-sweep direction. AZ flipped by request: its CAL search now
 * runs NEGATIVE first, then the reverse/bounce legs mirror. The index
 * overshoot + creep re-capture are NOT mirrored — the canonical edge
 * (overshoot to the positive side, creep back negative) must stay identical
 * for both sweep directions or home shifts by the index pulse width. */
#define M1_HOME_SWEEP_SIGN   1.0f   // EL: first search leg runs positive
#define M2_HOME_SWEEP_SIGN  -1.0f   // AZ: first search leg runs negative
/* Homing/unwind coverage is PER-AXIS because the two axes have different
 * ranges of motion. EL is unrestricted (full 360 mechanical), so it sweeps
 * 365 deg — index is 1/rev, >=360 guarantees a crossing with 5 deg margin.
 * AZ is limited to a -140..+180 (320 deg) ROM (M2_MIN/MAX_DEG), so homing
 * needs — and can only use — a 320 deg field of view: the index must lie
 * inside that arc, and the recovery unwind may not exceed it either. */
#define M1_HOME_SWEEP_TOTAL_DEG 365.0f // EL: full-rev sweep (>=360 + margin)
#define M2_HOME_SWEEP_TOTAL_DEG 320.0f // AZ: limited to the -140..+180 ROM
#define M1_UNWIND_DEG           365.0f // EL verify-failure recovery rotation
#define M2_UNWIND_DEG           320.0f // AZ: capped to the 320 deg ROM
#define VERIFY_MAX_RETRY     1      // unwind/re-home attempts per axis per CAL
#define HOME_SEARCH_RETRY    1      // EL homing search: gentle rest+backoff+retry
                                    // attempts per direction before it counts as
                                    // BLOCKED (user spec: "snag twice, then rotate
                                    // the other way"). Also lets the reverse sweep
                                    // recover a post-stall sync stumble instead of
                                    // freezing — an EL snag otherwise re-launches
                                    // the reverse with no rest and stalls again.

#define EL_BLIND_PARK_DEG  -270.0f // CAL step 1: uncalibrated pre-home park —
                                 // EL blindly moves this far (signed) from
                                 // wherever it was resting. MUST stay opposite
                                 // in sign to EL_BLIND_PARK_RETRY_DEG: the retry
                                 // reaches the SAME attitude the long way around
                                 // (-45 <-> +315), so a +value here would send
                                 // the retry the same direction into the same
                                 // snag (bench 2026-07-17: +90 broke this).
#define EL_BLIND_PARK_RETRY_DEG 90.0f // if that direction properly snags,
                                 // park HERE instead (signed offset from the
                                 // same starting anchor — +315 reaches the
                                 // same physical attitude as -45 by going
                                 // all the way around the other way; homing
                                 // is exempt from the +/-180 watchdog and
                                 // the snag watch guards the whole sweep)
#define EL_AZVAL_PARK_DEG 180.0f // CAL step 6: EL held here during the AZ
                                 // validation sweep. Must stay below the sun
                                 // window's lower edge (sunEl - ~22.5 deg)
#define CAL_SUN_OFF_DEG   90.0f  // CAL step 3: AZ parks this far (wrapped to
                                 // +/-180) from the sun-center azimuth while
                                 // EL homes/validates

/* ---- Soft/hard limits (all measured from encoder zero = physical home
 *      after SETZERO) ---- */
#define M1_MIN_DEG     -180.0f
#define M1_MAX_DEG      180.0f
#define M2_MIN_DEG     -140.0f    // AZ ROM is ASYMMETRIC: -140..+180 (320 deg
#define M2_MAX_DEG      180.0f    // span). The band below -140 (down to -180)
                                 // is OUTLAWED — reached via the elevation
                                 // flip, not by driving the azimuth there.
#define LIMIT_DEG       180.0f   // hard encoder watchdog, +/- from zero
#define LIMIT_GUARD_DEG  10.0f   // watchdog margin (tolerates brief overshoot
                                 // on full-range pivots). Watchdog runs only
                                 // during MOVE/DANCE; homing legally exceeds
                                 // +/-180 (365 sweeps and unwinds).

/* ---- Settle / trim (post-slew accuracy) ---- */
#define SETTLE_TOL_DEG   1.5f    // final pointing tolerance
#define SETTLE_WAIT_MS   200UL   // pause after the last step before reading
                                 // the encoder (short: the crawl landing
                                 // already arrives with the frame calm)

/* ---- Active hold (post-move station keeping) ----
 * Once a move completes the drivers STAY energized and holding. Position is
 * maintained until a STOP command is received (which releases the drivers
 * immediately) or HOLD_SILENCE_MS (30 min) elapses with no serial commands
 * (dead-link failsafe). While holding, encoder drift beyond HOLD_TRIM_DEG
 * is quietly re-trimmed. NOTE: if the mount jiggles WITH drivers energized,
 * check DM556T DIP SW4 — idle-current reduction cuts holding torque to 50%
 * 0.4 s after motion stops; set it to full-current hold. */
#define HOLD_TRIM_DEG    2.0f    // re-trim when drift while holding exceeds this
#define HOLD_CHECK_MS    500UL   // drift check cadence while holding
#define TRIM_SPEED_DPS   13.5f   // slow closed-loop trim rate (proven EL value)
#define TRIM_ACCEL_DPS2  67.5f
                                 // (v5: the AZ "nudge to free a stuck EL
                                 // wire" is gone — settle is bounded by
                                 // SETTLE_MAX_PASSES and a stall is reported
                                 // honestly as ERR 58; the host decides.)

/* ---- Dance ---- */
#define DANCE_SPEED_DPS  50.0f
#define DANCE_ACCEL_DPS2 45.0f
#define DANCE_DWELL_MS   250UL

/* ---- Camera-flip pointing ------------------------------------------------
 * The camera can rotate past the zenith: the mount attitude (el, az) points
 * at the SAME sky direction as (180-el, az+180) (and (-180-el, az+180) for
 * negative elevations). For sky targets (el >= 0) MOVE evaluates both
 * solutions and takes the cheaper one — often turning a near-180 azimuth
 * swing into a much shorter flip over the top. Below-horizon targets are
 * NEVER flipped: those are mechanical frame-positioning moves (e.g. the
 * host's post-CAL offset drive before SETZERO), where the literal encoder
 * coordinates matter, not just the boresight direction. */
#define FLIP_MARGIN_DEG  10.0f   // flip only if it saves more than this
                                 // (hysteresis: prefer the direct solution)

/* ---- Sun keep-out ---- */
#define SUN_CENTER_EL_DEG 85.0f // BOOT DEFAULT sun-facing direction IN THE
#define SUN_CENTER_AZ_DEG -68.0f // PHYSICAL-HOME FRAME, canonical form
                                 // (|el| <= 90). Bench-measured 2026-07-15
                                 // post-SETZERO: "MOVE -55 -35" points at
                                 // the sun. v5: the host keeps this LIVE via
                                 // "SUN <el> <az>" (every 60 s); the default
                                 // only covers the window before the first
                                 // push. The real sun moves (and the gondola
                                 // yaws), so the default is only correct near
                                 // the measurement time/place/heading. With the
                                 // sun below the horizon (el < 0), sky
                                 // pointing (el >= 0) can never enter the
                                 // box — it guards frame moves and the
                                 // over-the-top paths.

/* Index-to-physical-home offsets. MUST MATCH airis_main's
 * HOME_EL_OFFSET_DEG / HOME_AZ_OFFSET_DEG: after CAL zeroes each encoder on
 * its index, the host MOVEs by these amounts and SETZEROes, putting the
 * physical home at encoder (0,0). Between an axis's index-zeroing and that
 * SETZERO, the encoder frame = physical frame + this offset — and every
 * sun computation transforms through it so the keep-out stays anchored to
 * the REAL sun even mid-CAL (frameElOff/frameAzOff below). */
#define HOME_EL_OFFSET_DEG -55.0f
#define HOME_AZ_OFFSET_DEG -35.0f
#define SUN_WIN_EL_DEG  25.0f    // full elevation span of the sun window
#define SUN_WIN_AZ_DEG  35.0f    // full azimuth span of the sun window
#define CAM_FOV_DEG     15.0f    // camera full FOV (inflates the box)
#define SUN_CLEAR_DEG    2.0f    // extra clearance for detour waypoints

/* ---- Housekeeping ---- */
#define KILL_TIMER_MS    60000UL // COMMAND-LOSS KILL TIMER: if no serial
                                 // command arrives for this long while motion
                                 // is in progress, the host is presumed dead —
                                 // ramp-stop everything and power the drivers
                                 // down. Exempt during CAL (homing runs
                                 // minutes with no commands by design; the
                                 // HOMING_SLEEP_MS release covers a stalled
                                 // CAL instead).
#define IDLE_SLEEP_MS    20000UL // de-energize drivers after this long with no
                                 // serial commands while idle (stationary)
#define HOLD_SILENCE_MS 1800000UL// DEAD-LINK FAILSAFE while ACTIVELY HOLDING
                                 // (homed + on target): hold position for up
                                 // to 30 min; a STOP command releases earlier.
                                 // If NOTHING arrives for 30 min the link or
                                 // host is dead — release the drivers so the
                                 // mount doesn't hold indefinitely on a dead
                                 // link (bench 2026-07-14: USB died and motors
                                 // held at full current with no way out).
#define HOMING_SLEEP_MS 120000UL // same, but while a CAL runs (CAL has long
                                 // command-free stretches by design)
#define POS_STREAM_MS      200UL // POS/TGT push cadence during MOVE/DANCE
                                 // (5 Hz; v5 — the bridge never reads the
                                 // stream, so it is kept light for the link)
#define IDLE_HEARTBEAT_MS 1000UL // keepalive POS while stationary. A silent
                                 // CDC link gets AUTOSUSPENDED by the host
                                 // OS — Linux's default idle timer is 2 s,
                                 // and the bench logs show the link dying in
                                 // every >2 s command gap right after CAL
                                 // (the first quiet moment of a session) and
                                 // garbling the first bytes of each command
                                 // on wake ("RX: CA" -> ERR 60 storms). A
                                 // 1 s heartbeat keeps the bus busy faster
                                 // than the 2 s suspend timer fires. It also
                                 // doubles as the host's liveness signal —
                                 // every host parser ignores stray POS lines
                                 // by design. (Root fix is host-side: udev
                                 // rule forcing power/control=on for the
                                 // Mega; this heartbeat is the belt to that
                                 // suspender.)
#define HOME_LOG_MS        200UL // HOME_LOG push cadence during CAL (host
                                 // captures these rows to a debug CSV)
/* ---- Telemetry yield (link-saturation fix, bench 2026-07-16) ----
 * The unsolicited periodic streams (20 Hz POS+TGT mid-move, 5 Hz HOME_LOG,
 * 1 Hz idle heartbeat) exist only to feed the host's debug CSV and the
 * quick-recal raw hint. Sized against raw baud math they look cheap, but
 * the real bottleneck is the ATmega16U2 USB bridge: tiny buffers, no UART
 * flow control. A sustained stream keeps it near-full, and any brief
 * host-side USB stall then clips whatever is in flight — IN BOTH
 * DIRECTIONS (motor_debug_20260716: 'MOVE 20.000 3.000' echoed as
 * 'RX: MO 0.000' while a TGT line reached the host missing its 'TGT '
 * prefix, same instant). Two rules fix it without losing any parsed
 * functionality:
 *   1. RX PRIORITY: never start an unsolicited push while command bytes
 *      are waiting, a line is mid-assembly, or any byte arrived within the
 *      last RX_QUIET_MS (v5 rxQuietUntil). Costs at most one sample,
 *      exactly when that sample was colliding anyway.
 *   2. NEVER BLOCK: only push when the TX ring can take the whole burst
 *      (Serial.write busy-waits when the 64-byte ring is full, stalling
 *      the loop AND stuffing the congested bridge harder). If there's no
 *      room, DROP the sample — the stream self-throttles to what the
 *      link actually carries.
 * Command REPLIES and state-change pushes (ACK/NAK/STATUS/ERR/DONE/ZERO SET)
 * are exempt: host transactions block on those, and they fire right after
 * a complete command when RX is naturally quiet. */
#define TELEM_TX_MIN_FREE   60   // TX-ring free bytes required before an
                                 // unsolicited burst (Mega ring = 64, so
                                 // availableForWrite() tops out at 63:
                                 // 60 ≈ "ring effectively empty", enough
                                 // for a POS+TGT pair without blocking)
#define LINE_STALE_MS      250UL // discard a PARTIAL command line older than
                                 // this. Protocol lines arrive as one burst
                                 // (<5 ms at 115200); a fragment aging alone
                                 // means its terminator was lost on the bus,
                                 // and left in place it eats the front of
                                 // every later command (the persistent
                                 // 'MOVE'->'MO' poisoning). 250 ms heals
                                 // before the host's 0.5 s retry. CAVEAT:
                                 // hand-typing into minicom slower than one
                                 // key per 250 ms now gets purged — paste
                                 // whole lines during manual bench sessions.
#define MIN_STOP_SPEED_SPS 30.0f // floor for a STOP ramp-down, steps/s

/* Quintic ramp constants: peak accel on a ramp = 1.875*Vp/Ta (max of q'),
 * ramp distance = 0.5*Vp*Ta (integral of q over the ramp). */
#define QUINTIC_PEAK_ACC 1.875f

/* --------------------------- System state ------------------------------ */
enum : uint8_t {
  ST_IDLE       = 0,   // homed, ready
  ST_MOVING     = 1,
  ST_HOMING     = 2,
  ST_NOT_HOMED  = 3,
  ST_DANCING    = 4,
  ST_FAULT_HOME = 10,  // latched: homing/verify failed (codes 51-54)
  ST_FAULT_LIMIT= 11   // latched: encoder crossed the hard limit (55/56)
};
uint8_t sysState  = ST_NOT_HOMED;
uint8_t faultCode = 0;           // PY_* code reported while a fault is latched
bool    stopPending  = false;    // STOP issued, axes still ramping down
bool    motorsEnabled = false;

/* CAL top-level stages (sun-safe ordering, see header) */
enum : uint8_t { C_NONE = 0, C_EL_PARK, C_AZ_HOME, C_AZ_PARK,
                 C_EL_HOME, C_VERIFY,
                 C_Q_AZ, C_Q_EL };         // quick re-cal (CALQ) windows
uint8_t calStage  = C_NONE;
float   calSafeAz = 36.0f;       // sun-safe AZ park; recomputed from the sun
                                 // center (+CAL_SUN_OFF_DEG) at every CAL, or
                                 // overridden by the host via "CAL <az>"
float   calElStartDeg  = 0.0f;   // EL rest position when CAL began (anchor
                                 // for the mirrored +315 blind-park retry)
bool    calElParkRetry = false;  // blind park already retried mirrored?
float   calAzParkAlt   = 0.0f;   // the OTHER 90-deg-off-sun azimuth (fallback
                                 // if the park toward the nearer one snags)
bool    calAzParkRetry = false;  // AZ park already retried mirrored?
bool    qcalActive     = false;  // quick re-cal (CALQ) in progress: homing
                                 // misses/snags fall back to a full CAL
                                 // instead of faulting
float   qcalElDelta    = 0.0f;   // EL index hint (deg from position at CALQ
                                 // receipt; EL hasn't moved when it's used)
bool    calSkipVerify  = false;  // full CAL runs without the +/-180 validation
                                 // spins (educated-recal fallback: the range
                                 // was proven on the last full CAL and nothing
                                 // has physically changed)
uint8_t movesSinceRecal = 0;     // completed moves since the last (educated or
                                 // full) re-cal; triggers one at MOVES_PER_RECAL
bool    recalSweepPending = false; // educated re-cal phase 1 (slew to 0,0) is
                                 // running; its completion starts the sweep
unsigned long lastEncSvcUs = 0;  // micros() of the last 1 kHz encoder service

/* Per-axis homing phases. H_EXT = third search leg: if the reverse sweep
 * hits a REAL range limit before the index fires (homing started near a
 * cable stop, so the +180 leg ran away from the index), bounce and sweep
 * forward far enough to cover the entire reachable range. Appended at the
 * end so HOME_LOG phase numbers stay comparable across logs. */
enum : uint8_t { H_NONE = 0, H_PREZERO, H_FWD, H_REV,
                 H_OVER, H_CREEP, H_RETURN, H_DONE, H_EXT,
                 H_REST, H_RETRY_BACK };  // gentle search retry: rest, then a
                                          // backoff run-up, then re-issue the
                                          // same sweep (appended so HOME_LOG
                                          // phase numbers stay comparable)

/* Supervised-leg (snag-watched move) phases and results. REST phases are
 * full-stop pauses (SNAG_REST_MS) so the motor re-syncs from standstill.
 * Recovery ladder on a snag: backoff -> PUNCH (fast) -> SLOW (mid-band). */
enum : uint8_t { LEG_IDLE = 0, LEG_MAIN,
                 LEG_REST_BACKOFF, LEG_BACKOFF,
                 LEG_REST_PUNCH,   LEG_PUNCH,
                 LEG_REST_SLOW,    LEG_SLOW };
enum : uint8_t { LEG_RUNNING = 0, LEG_DONE, LEG_SNAGGED };

/* Verification stages (EL first — AZ is still parked 90 deg off-sun —
 * then EL drops to -30 and AZ validates) */
enum : uint8_t { V_IDLE = 0,
                 V_EL_FWD, V_EL_BACK, V_EL_RET,   // EL 0 -> +180 -> -180 -> 0
                 V_EL_PARK,                       // EL parking at -30 for AZ
                 V_AZ_FWD, V_AZ_BACK, V_AZ_RET,   // AZ 0 -> +180 -> -180 -> 0
                 V_REC_RET,                       // snagged axis returning to 0
                 V_UNWIND,                        // 365 deg away from the snag
                 V_REHOME };                      // re-homing after the unwind
uint8_t verStage = V_IDLE;

/* Sun keep-out center in the PHYSICAL frame. Boots at SUN_CENTER_* and is
 * updated LIVE by "SUN <el> <az>" (v5) — the only writer is that handler. */
float sunEl = SUN_CENTER_EL_DEG;
float sunAz = SUN_CENTER_AZ_DEG;

/* Current-frame-from-physical offsets. 0 after SETZERO (encoder frame ==
 * physical frame). Between an axis's index re-zeroing and SETZERO the
 * frame is the INDEX frame: offset = HOME_*_OFFSET (set per axis at its
 * re-zero moment). All sun geometry — box center, flip equivalence,
 * hemisphere selection — transforms through these so the keep-out tracks
 * the REAL sun in whatever frame the encoders currently speak. */
float frameElOff = HOME_EL_OFFSET_DEG;   // boot guess: index-like frame
float frameAzOff = HOME_AZ_OFFSET_DEG;
uint8_t sunLegIdx = 0, sunLegCount = 0;  // pending detour waypoints for MOVE
float sunWpEl[4], sunWpAz[4];

/* Last host-REQUESTED target (raw, as commanded — before any camera flip).
 * POS reports the current boresight in whichever representation is closer
 * to this, so host-side error readouts always compare like with like. */
float reqEl = 0.0f, reqAz = 0.0f;

/* Camera-flip pointing optimisation (see MOVE handler / header). When OFF,
 * MOVE lands LITERALLY at the commanded (el, az): one command -> one
 * mechanical attitude, always repeatable — required for bench testing and
 * for any use that cares about a deterministic mount position. When ON, sky
 * targets (el >= 0) may be executed at their over-the-top equivalent
 * (180-el, az+180) when that is a shorter slew or the only sun-safe route;
 * the boresight points the same sky direction either way, but WHICH of the
 * two attitudes is chosen depends on the current position, so re-issuing an
 * identical command can leave the mount at a different mechanical pose.
 * Default OFF; toggle at runtime with the FLIP command. Flight builds that
 * want the shorter-slew / larger sun-safe envelope should enable it
 * explicitly (e.g. host sends "FLIP 1" at startup) so the mode is logged. */
const bool flipEnable = false;

/* Active hold: station-keeping trims while idle (yield to any command). */
bool holdTrimming = false;
unsigned long lastHoldChkMs = 0;

/* Settle / dance bookkeeping */
uint8_t  settlePasses  = 0;      // trim passes taken (bounded by SETTLE_MAX_PASSES)
uint32_t settleWaitEnd = 0;      // millis() deadline before the encoder read

/* DANCE drives axes directly (no MOVE flip), so its azimuth targets must stay
 * inside the -140..+180 ROM. Elevation keeps its full +/-180 range. */
const float DANCE_SEQ[][2] = {
  {-180, -140}, { 180,  180}, {   0,    0},
  {-180,  180}, { 180, -140}, {   0,    0},
};
const uint8_t DANCE_LEN = sizeof(DANCE_SEQ) / sizeof(DANCE_SEQ[0]);
uint8_t danceIdx = 0;
unsigned long danceDwellStart = 0;

unsigned long lastCmdMs     = 0; // last received serial command (idle sleep)
unsigned long lastPosPushMs = 0; // last streamed POS during a move
unsigned long lastHomeLogMs = 0; // last streamed HOME_LOG during CAL
unsigned long lastHeartbeatMs = 0; // last idle keepalive POS

/* ---- v5 line protocol state ---- */
static uint8_t lastSeq = 255;          // seq of the last EXECUTED line
static char    lastLine[64] = "";      // its command text (for de-dup)
static unsigned long rxQuietUntil = 0; // hold unsolicited telemetry until this

/* ------------------------------ Axis ----------------------------------- */
struct Axis {
  const char *name;              // "M1 (EL)" / "M2 (AZ)"
  uint8_t pulsePin, dirPin;
  bool    invertDir;
  unsigned long stepsPerRev;

  /* encoder (ISR-owned) */
  volatile long    enc;
  volatile uint8_t qPrev;
  int8_t           encSign;
  volatile bool    indexArmed, indexFound;
  volatile long    encAtIndex;

  /* continuous index re-zero (drift control, see ENC_SERVICE_US) */
  bool  idxRezeroArmed;          // true only after a successful home
  long  indexRefEnc;             // enc count the index sits at when calibrated
  long  encPrevSvc;              // enc at the previous 1 kHz service tick

  /* step generator + quintic profile (times s, distances steps).
   * Five phases: accel (Ta1) / cruise (Tflat) / decel to Vland (Ta2) /
   * crawl at Vland (Tcrawl) / gentle stop (Ta3). A plain move (no crawl
   * landing) is simply the same profile with Vland=0, Tcrawl=Ta3=0. */
  bool   moving;
  int8_t dir;                    // +1 / -1
  long   totalSteps, stepsDone;
  unsigned long t0us;            // micros() at profile start
  bool   pulseHigh;              // step pin currently HIGH (non-blocking pulse)
  unsigned long pulseHighUs;     // micros() when the pulse was raised
  float  Vp;                     // peak velocity reached (steps/s)
  float  Vland;                  // crawl speed (steps/s); 0 = no landing
  float  apk;                    // peak accel used to plan (steps/s^2)
  float  Ta1, Tflat, Ta2, Tcrawl, Ta3;  // phase durations
  float  T2, T3, T4, Ttot;       // cumulative phase-end times
  float  Dr1;                    // accel-ramp distance (steps)
  float  S2, S3, S4;             // cumulative phase-end positions (steps)
  bool   decelOnly;              // running a STOP ramp-down

  /* continuous snag watch (stall detector) */
  bool  snagWatch, snagged;
  long  moveStartEnc;
  long  lastChkEnc;              // encoder count at the previous rate sample
  long  lastChkSteps;            // stepsDone at the previous rate sample
  unsigned long lastSnagChkMs;
  uint8_t snagOverCnt;           // consecutive stalled samples (debounce)

  /* supervised leg (snag-handled move with backoff+punch) */
  uint8_t legPhase;
  float   legTargetDeg, legDps, legAccel;
  uint8_t punchesUsed;
  uint8_t slowUsed;              // slow-retry rungs consumed on this leg
  uint8_t punchProgressRetries;  // progress-earned punch retries this leg
  float punchStartDeg;           // encoder position when the punch launched
  unsigned long legResumeMs;     // rest-phase deadline (SNAG_REST_MS)
  uint8_t legPunchTries;         // punches allowed on THIS leg (legStart sets
                                 // the per-axis default; owners may override
                                 // for specific legs, e.g. the blind-park
                                 // +315 retry punches once even on EL)

  /* homing / verification */
  uint8_t homePhase;
  bool    homed;
  bool    zeroKnown;             // homed at least once; zero survives faults
  long    idxRoughEnc;           // fast-sweep index capture (fallback zero)
  uint8_t verRetry;              // unwind/re-home attempts used this CAL

  /* Gentle homing-search retry (EL): rest so the stalled rotor re-syncs, back
   * off for a run-up, then re-attempt the SAME sweep. A direction counts as
   * blocked only after the retries are spent ("snag twice"). */
  uint8_t       searchRetry;        // gentle retries used in the current direction
  unsigned long homeRestUntil;      // millis() deadline for the H_REST pause
  uint8_t       searchResumePhase;  // H_FWD/H_REV/H_EXT to resume after the retry
  float         searchResumeTarget; // sweep target to re-issue on the retry

  float targetDeg;               // last commanded MOVE target (encoder deg)
};

Axis m1, m2;
Axis *verFailAxis = NULL;        // axis the verify machine is recovering
int8_t verFailDir = 0;           // travel direction at the verify snag

/* ---------------------------- Prototypes ------------------------------- */
void  printPos();
void  printTgt();
bool  qcalHandleSnag(Axis &a);
void  qcalBegin(Axis &a, float deltaDeg);
void  qcalFallback();
void  calBeginFull();
void  encService();
void  armIndexRezero(Axis &a);
void  beginEducatedRecal();
void  beginRecalSweep();
void  homingFinish();
void  pushStatus(uint8_t c);
void  legStart(Axis &a, float targetDeg, float dps, float accelDps2);
void  homeBegin(Axis &a);
void  verEnter(uint8_t s);
static inline float sunHalfEl();
static inline float sunHalfAz();

/* ----------------------- Encoder ISRs (x4 decode) ---------------------- *
 * Taken verbatim from the proven v3.x firmware — encoders and their ISRs
 * are known-good and are the ground truth for every closed-loop decision. */
static const int8_t QTAB[16] = { 0,-1, 1, 0,  1, 0, 0,-1,
                                -1, 0, 0, 1,  0, 1,-1, 0 };

inline void quadUpdate(Axis &a, uint8_t pinA, uint8_t pinB) {
  uint8_t s = (digitalRead(pinA) << 1) | digitalRead(pinB);
  a.enc += a.encSign * QTAB[(a.qPrev << 2) | s];
  a.qPrev = s;
}

void isrM1Quad() { quadUpdate(m1, M1_ENC_A_PIN, M1_ENC_B_PIN); }
void isrM2Quad() { quadUpdate(m2, M2_ENC_A_PIN, M2_ENC_B_PIN); }

void isrM1Index() {
  if (m1.indexArmed && !m1.indexFound) { m1.encAtIndex = m1.enc; m1.indexFound = true; }
}
void isrM2Index() {
  if (m2.indexArmed && !m2.indexFound) { m2.encAtIndex = m2.enc; m2.indexFound = true; }
}

/* ------------------------------ Helpers -------------------------------- */
long encRead(Axis &a) {
  noInterrupts();
  long c = a.enc;
  interrupts();
  return c;
}

float encDeg(Axis &a) {
  return (float)encRead(a) * (360.0f / (float)ENC_CPR);
}

bool isFault() { return sysState == ST_FAULT_HOME || sysState == ST_FAULT_LIMIT; }

/* ---- 1 kHz encoder service: continuous index re-zero (drift control) ----
 * Homing manages the index pulse itself (idxRezeroArmed stays false through
 * CAL/CALQ); this runs only in normal operation. When the ISR reports a
 * fresh index crossing, snap the encoder frame so the index lands on its
 * calibrated count — but only when travelling in the canonical (encoder-
 * negative) direction, so we always latch the SAME physical edge of the
 * finite-width pulse that homing zeroed on. All shifts are done in whole
 * encoder counts (long), so no fractional data is lost or truncated. */
static void indexRezero(Axis &a, long encNow) {
  if (!a.idxRezeroArmed || !a.indexFound) return;

  noInterrupts();
  long cap = a.encAtIndex;            // enc count latched at the rising edge
  a.indexFound = false;              // consume; ISR re-arms on the next edge
  interrupts();

  int8_t travelDir = (encNow > a.encPrevSvc) ? 1 :
                     (encNow < a.encPrevSvc) ? -1 : 0;
  if (travelDir >= 0) return;         // wrong edge/stationary: don't re-zero

  long err = cap - a.indexRefEnc;     // how far the frame has drifted
  if (labs(err) >
      (long)lround((double)INDEX_REZERO_TOL_DEG / 360.0 * (double)ENC_CPR))
    return;                           // implausible: ignore as spurious
  if (err == 0) return;

  noInterrupts();
  a.enc      -= err;                  // slide the whole frame onto the index
  a.encAtIndex -= err;
  interrupts();

  serialWrite(F("IDX "));
  serialWrite(a.name);
  serialWrite(F(": re-zeroed on index pass (drift "));
  serialWrite((float)err * (360.0f / (float)ENC_CPR), 3);
  serialWriteLine(F(" deg corrected)"));
}

/* Latch the current index as the calibrated reference (enc 0) and re-arm the
 * ISR so the 1 kHz service catches the next crossing. */
void armIndexRezero(Axis &a) {
  a.indexRefEnc    = 0;
  a.encPrevSvc     = encRead(a);
  a.idxRezeroArmed = true;
  noInterrupts();
  a.indexFound = false;
  a.indexArmed = true;
  interrupts();
}

void encService() {
  unsigned long now = micros();
  if (now - lastEncSvcUs < ENC_SERVICE_US) return;
  lastEncSvcUs = now;

  long e1 = encRead(m1);
  long e2 = encRead(m2);
  indexRezero(m1, e1);
  indexRezero(m2, e2);
  m1.encPrevSvc = e1;
  m2.encPrevSvc = e2;
}

float wrap180(float x) {
  while (x >  180.0f) x -= 360.0f;
  while (x < -180.0f) x += 360.0f;
  return x;
}

/* The sun center expressed in the CURRENT encoder frame (physical frame
 * constants + the active frame offsets). All keep-out geometry uses these,
 * never the raw constants, so CAL-time sweeps (index frame) and normal
 * pointing (physical frame) both guard the real sun. */
static inline float sunElF() { return sunEl + frameElOff; }
static inline float sunAzF() { return wrap180(sunAz + frameAzOff); }

/* ---- Quintic ramp shape --------------------------------------------- *
 * rampV(x) = velocity fraction at ramp fraction x (smootherstep)
 * rampQ(x) = its integral = position fraction of the ramp distance
 * rampQ(1) = 0.5, so a full ramp covers 0.5*Vp*Ta.                       */
static inline float rampV(float x) {
  return x * x * x * (10.0f - 15.0f * x + 6.0f * x * x);
}
static inline float rampQ(float x) {
  float x2 = x * x;
  return x2 * x2 * (2.5f - 3.0f * x + x2);     // 2.5x^4 - 3x^5 + x^6
}

/* Plan a move of |steps| at peak speed vmax (steps/s), peak accel apk.
 * The decel ramp uses apk*DECEL_RATIO (gentler braking, see header).
 *
 * landSteps/vland/aland > 0 request the CRAWL LANDING: the decel ramp ends
 * at vland instead of zero, the axis creeps through the final landSteps at
 * vland, and a last gentle ramp (peak accel aland) brings it to rest. The
 * landing only engages when the move is long and fast enough for it to
 * make sense (> 2x the zone, vmax comfortably above the crawl); everything
 * else — trims, index creeps — plans as a plain profile (Vland = 0), which
 * is the identical math with a zero-length crawl. */
void axisPlan(Axis &a, long steps, float vmax, float apk,
              long landSteps, float vland, float aland) {
  a.dir        = (steps > 0) ? 1 : -1;
  a.totalSteps = labs(steps);
  a.stepsDone  = 0;
  a.apk        = apk;
  a.decelOnly  = false;
  float D  = (float)a.totalSteps;
  float K1 = QUINTIC_PEAK_ACC / apk;                 // Ta per unit velocity,
  float K2 = QUINTIC_PEAK_ACC / (apk * DECEL_RATIO); // accel and decel ramps

  float Vl = 0.0f;
  a.Ta3 = 0.0f;
  a.Tcrawl = 0.0f;
  float LZ = (float)landSteps;
  bool land = (landSteps > 0 && vland > 0.0f &&
               D > 2.0f * LZ && vmax > 1.5f * vland);
  if (land) {
    Vl    = vland;
    a.Ta3 = QUINTIC_PEAK_ACC * Vl / aland;           // final Vl -> 0 ramp
    float d3 = 0.5f * Vl * a.Ta3;
    if (d3 > LZ) {                                   // zone smaller than the
      a.Ta3 = 2.0f * LZ / Vl;                        // ramp: stretch the ramp
      d3    = LZ;                                    // over the whole zone
    }
    a.Tcrawl = (LZ - d3) / Vl;
  }
  a.Vland = Vl;

  // Main phase covers everything before the landing zone. Ramp distances:
  //   accel 0->Vp:   0.5*K1*Vp^2
  //   decel Vp->Vl:  0.5*K2*(Vp^2 - Vl^2)
  float Dmain = D - (land ? LZ : 0.0f);
  float d1 = 0.5f * K1 * vmax * vmax;
  float d2 = 0.5f * K2 * (vmax * vmax - Vl * Vl);
  if (d1 + d2 <= Dmain) {                            // reaches vmax + cruise
    a.Vp    = vmax;
    a.Tflat = (Dmain - d1 - d2) / vmax;
  } else {                                           // short: triangular main
    a.Vp = sqrtf((2.0f * Dmain + K2 * Vl * Vl) / (K1 + K2));
    if (a.Vp < Vl) a.Vp = Vl;                        // degenerate guard
    a.Tflat = 0.0f;
  }
  a.Ta1 = K1 * a.Vp;
  a.Ta2 = K2 * (a.Vp - Vl);
  a.Dr1 = 0.5f * a.Vp * a.Ta1;

  // Cumulative phase boundaries (times and positions).
  a.T2   = a.Ta1 + a.Tflat;
  a.T3   = a.T2 + a.Ta2;
  a.T4   = a.T3 + a.Tcrawl;
  a.Ttot = a.T4 + a.Ta3;
  a.S2   = a.Dr1 + a.Vp * a.Tflat;
  a.S3   = a.S2 + 0.5f * (a.Vp + Vl) * a.Ta2;
  a.S4   = a.S3 + Vl * a.Tcrawl;
}

/* Commanded step position along the profile at time tau (s since t0us). */
float axisCmdPos(Axis &a, float tau) {
  if (tau <= 0.0f)   return 0.0f;
  if (tau >= a.Ttot) return (float)a.totalSteps;
  if (a.decelOnly)   return (float)a.totalSteps - a.Vp * a.Ta2 * rampQ((a.Ttot - tau) / a.Ta2);
  if (tau < a.Ta1)   return a.Vp * a.Ta1 * rampQ(tau / a.Ta1);          // accel
  if (tau < a.T2)    return a.Dr1 + a.Vp * (tau - a.Ta1);               // cruise
  if (tau < a.T3) {                                                     // decel Vp->Vland
    float dt = tau - a.T2;
    return a.S2 + a.Vp * dt - (a.Vp - a.Vland) * a.Ta2 * rampQ(dt / a.Ta2);
  }
  if (tau < a.T4)    return a.S3 + a.Vland * (tau - a.T3);              // crawl
  float dt = tau - a.T4;                                                // landing
  return a.S4 + a.Vland * dt - a.Vland * a.Ta3 * rampQ(dt / a.Ta3);
}

/* Commanded velocity (steps/s) at time tau — used to seed a STOP ramp. */
float axisCmdVel(Axis &a, float tau) {
  if (tau <= 0.0f || tau >= a.Ttot) return 0.0f;
  if (a.decelOnly) return a.Vp * rampV((a.Ttot - tau) / a.Ta2);
  if (tau < a.Ta1) return a.Vp * rampV(tau / a.Ta1);
  if (tau < a.T2)  return a.Vp;
  if (tau < a.T3)  return a.Vp - (a.Vp - a.Vland) * rampV((tau - a.T2) / a.Ta2);
  if (tau < a.T4)  return a.Vland;
  return a.Vland * (1.0f - rampV((tau - a.T4) / a.Ta3));
}

/* ------------------------- Serial output ------------------------------- */
/* All host-bound output goes through Serial.write(); Serial.print is not
 * used. Numbers are formatted into a stack buffer first, so the bytes on
 * the wire are identical to what Serial.print produced (Line = CRLF, float
 * default 2 decimals, char prints the character, uint8_t the number). */

void serialWrite(const __FlashStringHelper *s) {
  const char *p = reinterpret_cast<const char *>(s);
  uint8_t c;
  while ((c = pgm_read_byte(p++)) != 0) Serial.write(c);
}
void serialWrite(const char *s)      { Serial.write(s); }
void serialWrite(char c)             { Serial.write((uint8_t)c); }
void serialWrite(long v)             { char b[12]; ltoa(v, b, 10); Serial.write(b); }
void serialWrite(unsigned long v)    { char b[11]; ultoa(v, b, 10); Serial.write(b); }
void serialWrite(int v)              { serialWrite((long)v); }
void serialWrite(unsigned int v)     { serialWrite((unsigned long)v); }
void serialWrite(unsigned char v)    { serialWrite((unsigned long)v); }
void serialWrite(double v, int prec) { char b[20]; dtostrf(v, 0, (unsigned char)prec, b); Serial.write(b); }
void serialWrite(double v)           { serialWrite(v, 2); }

void serialWriteLine()                             { Serial.write("\r\n"); }
void serialWriteLine(const __FlashStringHelper *s) { serialWrite(s); serialWriteLine(); }
void serialWriteLine(const char *s)                { serialWrite(s); serialWriteLine(); }
void serialWriteLine(char c)                       { serialWrite(c); serialWriteLine(); }
void serialWriteLine(long v)                       { serialWrite(v); serialWriteLine(); }
void serialWriteLine(unsigned long v)              { serialWrite(v); serialWriteLine(); }
void serialWriteLine(int v)                        { serialWrite(v); serialWriteLine(); }
void serialWriteLine(unsigned int v)               { serialWrite(v); serialWriteLine(); }
void serialWriteLine(unsigned char v)              { serialWrite(v); serialWriteLine(); }
void serialWriteLine(double v, int prec)           { serialWrite(v, prec); serialWriteLine(); }
void serialWriteLine(double v)                     { serialWrite(v); serialWriteLine(); }

/* ------------------------- Driver enable ------------------------------- */
unsigned long motorsReadyUs = 0;   // micros() when the drivers finish waking
                                   // (non-blocking ENA_SETTLE_MS — the first
                                   // step of the next move is deferred past
                                   // this instead of delay()ing the loop)

void motorsEnable() {
  if (motorsEnabled) return;
  uint8_t en = (ENA_DISABLE_LEVEL == HIGH) ? LOW : HIGH;
  digitalWrite(M1_ENA_PIN, en);
  digitalWrite(M2_ENA_PIN, en);
  motorsEnabled = true;
  motorsReadyUs = micros() + (unsigned long)ENA_SETTLE_MS * 1000UL;
  serialWriteLine(F("MOTORS ON"));
}

void motorsDisable() {
  if (!motorsEnabled) return;
  digitalWrite(M1_ENA_PIN, ENA_DISABLE_LEVEL);
  digitalWrite(M2_ENA_PIN, ENA_DISABLE_LEVEL);
  motorsEnabled = false;
  serialWriteLine(F("MOTORS OFF"));
}

/* --------------------------- Move primitives --------------------------- */
/* Soft halt of one axis' step generation + supervision (no ramp; used when
 * a machine intentionally interrupts its own leg, e.g. on index capture). */
void axisHalt(Axis &a) {
  a.moving    = false;
  a.snagWatch = false;
  a.snagged   = false;
  a.legPhase  = LEG_IDLE;
}

/* Immediate full stop — reserved for latched faults. */
void axisStop(Axis &a) {
  axisHalt(a);
  a.indexArmed = false;
  a.homePhase  = H_NONE;
}

/* Begin a relative move of deltaDeg. Speeds in DEGREES; converted to this
 * axis' step units here so both motors always match angular rates.
 * watch=true arms the continuous snag detector for this move.
 * land=true adds the crawl landing (pointing legs only — see axisPlan). */
void axisStartMoveDeg(Axis &a, float deltaDeg, float dps, float accelDps2,
                      bool watch, bool land) {
  long steps = lround((double)deltaDeg / 360.0 * (double)a.stepsPerRev);
  a.snagged = false;
  if (steps == 0) { a.moving = false; a.snagWatch = false; return; }
  motorsEnable();                          // no-op if already on
  bool dirLevel = (steps > 0);
  if (a.invertDir) dirLevel = !dirLevel;
  digitalWrite(a.dirPin, dirLevel ? HIGH : LOW);
  // DIR setup time (DIR_SETUP_US) needs no delay: the planning math below
  // takes far longer than 10 us before the first pulse can possibly fire.
  float vmax  = dps       * (float)a.stepsPerRev / 360.0f;  // deg/s   -> steps/s
  float apk   = accelDps2 * (float)a.stepsPerRev / 360.0f;  // deg/s^2 -> steps/s^2
  long  lz    = land ? lround(LAND_ZONE_DEG   / 360.0 * (double)a.stepsPerRev) : 0;
  float vland = land ? LAND_SPEED_DPS  * (float)a.stepsPerRev / 360.0f : 0.0f;
  float aland =        LAND_ACCEL_DPS2 * (float)a.stepsPerRev / 360.0f;
  axisPlan(a, steps, vmax, apk, lz, vland, aland);
  a.moveStartEnc  = encRead(a);
  a.lastChkEnc    = a.moveStartEnc;
  a.lastChkSteps  = 0;
  a.lastSnagChkMs = millis();
  a.snagOverCnt   = 0;
  a.snagWatch     = watch;
  a.t0us   = micros();
  {
    // Non-blocking driver wake-up: if the drivers were just energized,
    // start the profile clock in the future so the first step lands after
    // ENA_SETTLE_MS — same guarantee the old delay() gave, no loop stall.
    long settle = (long)(motorsReadyUs - a.t0us);
    if (settle > 0) a.t0us += (unsigned long)settle;
  }
  a.moving = true;
}

/* Replace the active profile with a jerk-limited ramp-down from the CURRENT
 * speed to zero at the planned peak accel. Pure profile swap — no state
 * side effects; callers decide what happens after the axis stops. NEVER
 * dead-stop a moving stepper by just cutting the pulse train: the rotor and
 * load inertia carry it past the commanded electrical angle and it can shed
 * synchronization (seen on the bench: the AZ axis wandering AGAINST its
 * commanded direction after an instantaneous halt at the index capture). */
void axisRampDown(Axis &a) {
  if (!a.moving || a.decelOnly) return;
  float tau  = (float)(micros() - a.t0us) * 1e-6f;
  float vnow = axisCmdVel(a, tau);
  if (vnow < MIN_STOP_SPEED_SPS) vnow = MIN_STOP_SPEED_SPS;  // ramp must finish
  a.Vp     = vnow;
  a.Vland  = 0.0f;
  a.Ta1    = 0.0f;
  a.Tflat  = 0.0f;
  a.Tcrawl = 0.0f;
  a.Ta3    = 0.0f;
  a.Ta2    = QUINTIC_PEAK_ACC * vnow / a.apk;
  a.T2 = a.T3 = a.T4 = 0.0f;
  a.Ttot   = a.Ta2;
  a.decelOnly  = true;
  long dr      = (long)(0.5f * vnow * a.Ta2 + 0.5f);
  a.totalSteps = a.stepsDone + dr;
  a.t0us       = micros();
}

/* Controlled stop for the STOP command / kill timer: ramp down AND abandon
 * whatever supervised activity was in progress. */
void axisStopRamp(Axis &a) {
  a.indexArmed = false;
  a.homePhase  = H_NONE;
  a.snagWatch  = false;
  a.legPhase   = LEG_IDLE;
  axisRampDown(a);
}

/* Step generator; call as fast as possible. Emits at most one step per call,
 * chasing the commanded profile position. The loop cycles far faster than
 * the shortest step interval (~830 us at the 1200 steps/s AZ cap), so the
 * output tracks the plan with negligible jitter.
 * NON-BLOCKING PULSE: the step pin is raised here and lowered on a later
 * pass once PULSE_WIDTH_US has elapsed — no delayMicroseconds() in the hot
 * path. One loop pass takes longer than the 4 us minimum width anyway, so
 * in practice the pulse falls on the very next call. */
void axisService(Axis &a) {
  if (a.pulseHigh) {                       // finish the in-flight pulse first
    if ((unsigned long)(micros() - a.pulseHighUs) < PULSE_WIDTH_US) return;
    digitalWrite(a.pulsePin, LOW);
    a.pulseHigh = false;
  }
  if (!a.moving) return;
  float tau  = (float)(micros() - a.t0us) * 1e-6f;
  float scmd = axisCmdPos(a, tau);
  if ((float)a.stepsDone < scmd && a.stepsDone < a.totalSteps) {
    digitalWrite(a.pulsePin, HIGH);
    a.pulseHighUs = micros();
    a.pulseHigh   = true;
    a.stepsDone++;
  }
  if (a.stepsDone >= a.totalSteps) a.moving = false;
}

/* ----------------------- RAW open-loop run (bench) ---------------------- *
 * "RAW <1|2|both> <cw|c> <dps> <seconds>": drive the selected motor(s)
 * (mask bit0 = M1, bit1 = M2) at a constant step rate for a fixed time with
 * EVERYTHING bypassed — no homed/fault gating, no soft/hard limits, no sun
 * keep-out, no snag watch, no ramps. cw = the encoder-POSITIVE direction on
 * both axes, c (counterclockwise) = negative (labels only — the signed
 * encoder report is the ground truth). BLOCKING:
 * serial (including STOP) is not serviced until the time is up, which is
 * why the handler caps duration. The encoders keep counting throughout,
 * and each axis's measured change is reported against the stepped amount
 * at the end — a healthy axis matches, a stalled/desynced one doesn't,
 * which is the point of the tool. Ends by re-anchoring the hold/settle
 * targets to wherever the run left the mount so the active hold doesn't
 * immediately drag it back. */
#define RAW_MAX_S    30.0f   // blocking-run duration cap (no STOP possible)
#define RAW_MAX_DPS 360.0f   // keeps the step interval well above the
                             // generator's worst-case blocking pass time
void rawRun(uint8_t mask, bool cw, float dps, float seconds) {
  Axis *ax[2] = { &m1, &m2 };
  unsigned long intUs[2] = { 0, 0 }, nextUs[2];   // 0 = axis not selected
  long startEnc[2] = { 0, 0 }, stepsOut[2] = { 0, 0 };

  motorsEnable();                          // no-op if already on
  for (uint8_t i = 0; i < 2; i++) {
    if (!(mask & (1 << i))) continue;
    Axis &a = *ax[i];
    float sps = dps * (float)a.stepsPerRev / 360.0f;   // deg/s -> steps/s
    intUs[i] = (unsigned long)(1e6f / sps);
    bool dirLevel = cw;
    if (a.invertDir) dirLevel = !dirLevel;
    digitalWrite(a.dirPin, dirLevel ? HIGH : LOW);
    startEnc[i] = encRead(a);
  }
  delayMicroseconds(DIR_SETUP_US);
  long settle = (long)(motorsReadyUs - micros());      // driver wake-up if
  if (settle > 0) delayMicroseconds((unsigned long)settle);  // ENA just rose

  unsigned long runUs = (unsigned long)(seconds * 1e6f);
  unsigned long t0 = micros();
  nextUs[0] = nextUs[1] = t0;
  while ((unsigned long)(micros() - t0) < runUs) {
    for (uint8_t i = 0; i < 2; i++) {
      if (!intUs[i]) continue;
      if ((long)(micros() - nextUs[i]) < 0) continue;
      digitalWrite(ax[i]->pulsePin, HIGH);
      delayMicroseconds(PULSE_WIDTH_US);
      digitalWrite(ax[i]->pulsePin, LOW);
      stepsOut[i]++;
      nextUs[i] += intUs[i];
    }
  }

  float sign = cw ? 1.0f : -1.0f;
  for (uint8_t i = 0; i < 2; i++) {
    if (!(mask & (1 << i))) continue;
    Axis &a = *ax[i];
    float stepped = sign * (float)stepsOut[i] * (360.0f / (float)a.stepsPerRev);
    float encD    = (float)(encRead(a) - startEnc[i]) * (360.0f / (float)ENC_CPR);
    serialWrite(F("RAW "));
    serialWrite(a.name);
    serialWrite(F(": encoder moved "));
    serialWrite(encD, 2);
    serialWrite(F(" deg (stepped "));
    serialWrite(stepped, 2);
    serialWrite(F("), now at "));
    serialWriteLine(encDeg(a), 2);
  }
  m1.targetDeg = encDeg(m1);
  m2.targetDeg = encDeg(m2);
  reqEl = m1.targetDeg;
  reqAz = m2.targetDeg;
  lastCmdMs = millis();                    // full idle window after the block
  serialWriteLine(F("RAW DONE"));
}

/* ---------------------- Continuous snag detection ----------------------- *
 * While a watched move runs, the commanded step position and the encoder
 * must agree. A stalled stepper (wire catch, hard stop) freezes the encoder
 * while the command marches on, so the mismatch grows at the slew rate and
 * trips within a fraction of a second. Checked every SNAG_CHECK_MS — cheap
 * enough to never disturb step timing.                                     */
void snagService(Axis &a) {
  if (!a.snagWatch || !a.moving) return;
  unsigned long now = millis();
  if (now - a.lastSnagChkMs < SNAG_CHECK_MS) return;
  a.lastSnagChkMs = now;
  // STALL DETECTOR (user spec): a catch is the axis ESSENTIALLY NOT MOVING
  // while the command advances — never mere lag. Compare along-travel rates
  // over this sample window. Lag without stall (motor advancing raggedly,
  // spring assists, partial slip) does not trip; a leg that finishes far
  // short of target is handled by the end-of-leg check instead.
  long encNow   = encRead(a);
  long stepsNow = a.stepsDone;
  float cmdD = (float)(stepsNow - a.lastChkSteps) * (360.0f / (float)a.stepsPerRev);
  float encD = (float)(encNow - a.lastChkEnc) * (float)a.dir * (360.0f / (float)ENC_CPR);
  a.lastChkEnc   = encNow;
  a.lastChkSteps = stepsNow;
  float dt = (float)SNAG_CHECK_MS * 1e-3f;         // nominal window (loop
                                                   // jitter only lengthens it,
                                                   // which biases AGAINST a
                                                   // false trip)
  bool cmdMoving = cmdD > SNAG_CMD_MIN_DPS * dt;   // command meaningfully moving
  bool encStill  = encD < SNAG_STALL_DPS  * dt;    // axis essentially stopped
  if (cmdMoving && encStill) {
    if (++a.snagOverCnt < SNAG_STALL_CHECKS) return;
    a.snagOverCnt = 0;
    a.moving    = false;                   // stop pulsing into the stall NOW
    a.snagWatch = false;
    a.snagged   = true;                    // owner machine reacts next pass
    serialWrite(F("SNAG "));
    serialWrite(a.name);
    serialWrite(F(": axis stalled (encoder still for "));
    serialWrite(SNAG_STALL_CHECKS * SNAG_CHECK_MS);
    serialWrite(F(" ms) at "));
    serialWrite(encDeg(a), 1);
    serialWriteLine(F(" deg — motion halted"));
  } else {
    a.snagOverCnt = 0;
  }
}

/* Did the just-completed leg actually arrive near its target? With the
 * stall-only detector, an axis that advances at a fraction of the commanded
 * rate never trips mid-leg — the profile finishes with the axis far short.
 * Safety-relevant legs (parks, verify sweeps) call this and treat a short
 * arrival as a snag so the normal recovery ladders engage. */
bool legShort(Axis &a) {
  return fabsf(encDeg(a) - a.legTargetDeg) > VERIFY_END_TOL_DEG;
}

/* ------------------------- Supervised leg ------------------------------ *
 * One absolute-target move with the full snag policy baked in:
 *   MAIN -> (hit) -> BACKOFF 10 deg -> PUNCH at 3x home speed -> (hit again)
 *   -> LEG_SNAGGED (a "proper snag": the OWNER decides what that means).
 * Used for every homing, parking, and verification move so the behavior is
 * identical everywhere. Normal MOVE slews do NOT use legs (the hard-limit
 * watchdog and the settle loop cover them).                                */
void legStart(Axis &a, float targetDeg, float dps, float accelDps2) {
  a.legTargetDeg = targetDeg;
  a.legDps       = dps;
  a.legAccel     = accelDps2;
  a.punchesUsed  = 0;
  a.slowUsed     = 0;
  a.punchProgressRetries = 0;
  // Per-axis default; an owner may raise it AFTER legStart for one leg.
  a.legPunchTries = (&a == &m1) ? M1_SNAG_PUNCH_TRIES : M2_SNAG_PUNCH_TRIES;
  a.legPhase     = LEG_MAIN;
  // No crawl landing on supervised legs: homing/verify sweeps are already
  // slow, and their endpoints are range proofs, not pointing targets.
  axisStartMoveDeg(a, targetDeg - encDeg(a), dps, accelDps2, true, false);
}

uint8_t legService(Axis &a) {
  // Punch allowance is per-leg (set at legStart from the per-axis default:
  // EL 0 = a snag is real immediately, AZ 1 = one backoff+punch attempt).
  uint8_t punchTries = a.legPunchTries;
  uint8_t slowTries  = (&a == &m1) ? M1_SNAG_SLOW_TRIES : M2_SNAG_SLOW_TRIES;
  switch (a.legPhase) {
    case LEG_MAIN:
    case LEG_PUNCH:
    case LEG_SLOW:
      if (a.snagged) {
        a.snagged = false;
        /* PROGRESS-EARNED RETRY (user spec): a punch that gained more than
         * PUNCH_PROGRESS_DEG of real encoder travel before stalling again
         * is GRINDING THROUGH a yielding obstruction, not hitting a wall.
         * Refund the punch so the ladder cycles backoff -> punch again,
         * up to PUNCH_PROGRESS_RETRIES_MAX (5) times per leg. A true hard
         * stop gains < 1 deg and exhausts the ladder normally. */
        if (a.legPhase == LEG_PUNCH &&
            a.punchProgressRetries < PUNCH_PROGRESS_RETRIES_MAX) {
          float prog = (encDeg(a) - a.punchStartDeg) * (float)a.dir;
          if (prog > PUNCH_PROGRESS_DEG) {
            a.punchProgressRetries++;
            a.punchesUsed--;               // this punch earned another try
            serialWrite(F("SNAG "));
            serialWrite(a.name);
            serialWrite(F(": punch gained "));
            serialWrite(prog, 1);
            serialWrite(F(" deg — obstruction yielding, retrying ("));
            serialWrite(a.punchProgressRetries);
            serialWrite('/');
            serialWrite((int)PUNCH_PROGRESS_RETRIES_MAX);
            serialWriteLine(')');
          }
        }
        if (a.punchesUsed >= punchTries) {   // no punches (left) for this axis
          if (a.slowUsed < slowTries) {      // last rung: mid-band slow retry
            a.legPhase    = LEG_REST_SLOW;
            a.legResumeMs = millis() + SNAG_REST_MS;
            serialWrite(F("SNAG "));
            serialWrite(a.name);
            serialWriteLine(F(": resting, then retrying slow (mid-band)"));
            return LEG_RUNNING;
          }
          a.legPhase = LEG_IDLE;
          serialWrite(F("SNAG "));
          serialWrite(a.name);
          serialWriteLine(punchTries ? F(": all recovery rungs failed — proper snag")
                                    : F(": proper snag (no punch on this axis)"));
          return LEG_SNAGGED;
        }
        // REST first: the rotor may have shed sync in the stall — moving
        // again instantly is what caused the go-stop-go stumbling. A full
        // stop lets it settle, then the backoff starts cleanly from rest.
        serialWrite(F("SNAG "));
        serialWrite(a.name);
        serialWriteLine(F(": resting, then backing off 10 deg for a run-up"));
        a.legPhase    = LEG_REST_BACKOFF;
        a.legResumeMs = millis() + SNAG_REST_MS;
      } else if (!a.moving) {
        a.legPhase = LEG_IDLE;
        return LEG_DONE;
      }
      return LEG_RUNNING;

    case LEG_REST_BACKOFF:                          // settle before moving
      if ((long)(millis() - a.legResumeMs) < 0) return LEG_RUNNING;
      a.legPhase = LEG_BACKOFF;                     // away from the snag
      axisStartMoveDeg(a, -(float)a.dir * SNAG_BACKOFF_DEG,
                       a.legDps, a.legAccel, true, false);
      return LEG_RUNNING;

    case LEG_BACKOFF:
      if (a.snagged) {                              // blocked both ways
        a.snagged  = false;
        a.legPhase = LEG_IDLE;
        serialWrite(F("SNAG "));
        serialWrite(a.name);
        serialWriteLine(F(": back-off blocked too — proper snag"));
        return LEG_SNAGGED;
      }
      if (!a.moving) {                              // rest again before punch
        a.legPhase    = LEG_REST_PUNCH;
        a.legResumeMs = millis() + SNAG_REST_MS;
      }
      return LEG_RUNNING;

    case LEG_REST_PUNCH:                            // settle before the punch
      if ((long)(millis() - a.legResumeMs) < 0) return LEG_RUNNING;
      a.punchesUsed++;
      serialWrite(F("SNAG "));
      serialWrite(a.name);
      serialWrite(F(": punching through at "));
      serialWrite(SNAG_PUNCH_DPS, 0);
      serialWrite(F(" dps (attempt "));
      serialWrite(a.punchesUsed);
      serialWrite('/');
      serialWrite(punchTries);
      serialWriteLine(')');
      a.legPhase = LEG_PUNCH;
      a.punchStartDeg = encDeg(a);   // progress baseline for the retry rule
      axisStartMoveDeg(a, a.legTargetDeg - encDeg(a),
                       SNAG_PUNCH_DPS, a.legAccel, true, false);
      return LEG_RUNNING;

    case LEG_REST_SLOW:                             // settle, then slow rung
      if ((long)(millis() - a.legResumeMs) < 0) return LEG_RUNNING;
      a.slowUsed++;
      serialWrite(F("SNAG "));
      serialWrite(a.name);
      serialWrite(F(": slow retry at "));
      serialWrite(SNAG_SLOW_DPS, 0);
      serialWriteLine(F(" dps"));
      a.legPhase = LEG_SLOW;
      axisStartMoveDeg(a, a.legTargetDeg - encDeg(a),
                       SNAG_SLOW_DPS, a.legAccel, true, false);
      return LEG_RUNNING;

    default:
      return LEG_RUNNING;
  }
}

/* --------------------------- Fault latching ---------------------------- */
void latchFault(uint8_t state, uint8_t pyCode) {
  axisStop(m1);
  axisStop(m2);
  stopPending = false;
  m1.homed = m2.homed = false;   // faults invalidate home; re-CAL required
  calStage  = C_NONE;
  verStage  = V_IDLE;
  qcalActive = false;
  sunLegIdx = sunLegCount = 0;
  sysState  = state;
  faultCode = pyCode;
}


/* Proper snag with no recovery left -> cable-catch fault (53/54). */
void catchFault(Axis &a) {
  uint8_t c = (&a == &m1) ? PY_ERR_CATCH_M1 : PY_ERR_CATCH_M2;
  latchFault(ST_FAULT_HOME, c);
  serialWrite(F("ERR "));
  serialWrite(c);
  serialWrite(F(" CATCH "));
  serialWrite(a.name);
  serialWriteLine(F(": movement blocked — cable catch"));
}

/* ------------------------------ Homing --------------------------------- */
/* First-sweep direction for this axis (see M*_HOME_SWEEP_SIGN). "Forward"
 * (H_FWD) everywhere below means THIS direction, not encoder-positive. */
static inline float homeSweepSign(Axis &a) {
  return (&a == &m1) ? M1_HOME_SWEEP_SIGN : M2_HOME_SWEEP_SIGN;
}
/* Per-axis homing field of view / recovery unwind. EL is unrestricted (365
 * deg, a full rev plus margin); AZ is capped to its -140..+180 (320 deg) ROM. */
static inline float homeSweepTotal(Axis &a) {
  return (&a == &m1) ? M1_HOME_SWEEP_TOTAL_DEG : M2_HOME_SWEEP_TOTAL_DEG;
}
static inline float homeUnwindDeg(Axis &a) {
  return (&a == &m1) ? M1_UNWIND_DEG : M2_UNWIND_DEG;
}

/* Start (or re-issue) a homing SEARCH sweep at the standard homing rate,
 * remembering the phase and target so a gentle EL retry can rest, back off,
 * and re-attempt THIS exact leg. a.homePhase must already be the search phase
 * (H_FWD/H_REV/H_EXT) before this is called. */
static inline void searchLegStart(Axis &a, float targetDeg) {
  a.searchResumePhase  = a.homePhase;
  a.searchResumeTarget = targetDeg;
  legStart(a, targetDeg, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
}

void startSearchFwd(Axis &a) {
  float s = homeSweepSign(a);
  a.indexFound  = false;
  a.indexArmed  = true;
  a.searchRetry = 0;            // fresh retry budget for this new direction
  a.homePhase   = H_FWD;
  serialWrite(F("HOME "));
  serialWrite(a.name);
  serialWrite(F(": "));
  serialWrite(s > 0 ? '+' : '-');
  serialWriteLine(F("180 index search"));
  searchLegStart(a, encDeg(a) + s * HOME_SWEEP_FWD_DEG);
}

void homeBegin(Axis &a) {
  a.homed      = false;
  a.indexFound = false;
  if (a.zeroKnown) {
    // Re-CAL after a fault or a previous home: return to the old zero FIRST
    // so the sweep stays inside the proven cable envelope. Slow + watched.
    a.indexArmed = false;
    a.homePhase  = H_PREZERO;
    serialWrite(F("HOME "));
    serialWrite(a.name);
    serialWriteLine(F(": returning to previous zero before the sweep"));
    legStart(a, 0.0f, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
  } else {
    startSearchFwd(a);
  }
}

/* Begin the gentle search retry (caller already bumped a.searchRetry): a
 * full-stop REST so the stalled rotor re-syncs, then H_RETRY_BACK backs off
 * for a run-up and re-issues the SAME sweep (searchResume* captured at the
 * sweep's start). The rest is the actual fix for "EL won't move back after
 * catching": re-launching straight out of a stall shed rotor sync and stalled
 * again. searchResumePhase/Target must already describe the leg being retried. */
void enterSearchRest(Axis &a) {
  a.homePhase     = H_REST;
  a.homeRestUntil = millis() + SNAG_REST_MS;
  serialWrite(F("HOME "));
  serialWrite(a.name);
  serialWrite(F(": caught — resting then re-attempting ("));
  serialWrite(a.searchRetry);
  serialWrite('/');
  serialWrite(HOME_SEARCH_RETRY);
  serialWriteLine(F(") before this direction counts as blocked"));
}

/* Drives one axis' homing; latches faults on failure. The owner (CAL or
 * verify machine) watches for homePhase == H_DONE. */
void homeService(Axis &a) {
  if (a.homePhase == H_NONE || a.homePhase == H_DONE) return;

  /* Gentle search retry, stage 1 — rest: hold motionless until the rotor has
   * re-synced (no leg is running; the snag already halted motion). */
  if (a.homePhase == H_REST) {
    if ((long)(millis() - a.homeRestUntil) < 0) return;   // still resting
    a.homePhase = H_RETRY_BACK;
    serialWrite(F("HOME "));
    serialWrite(a.name);
    serialWriteLine(F(": rested — backing off for a run-up"));
    // Back off AWAY from the snag (opposite the last travel direction), then
    // H_RETRY_BACK re-issues the sweep from rest.
    legStart(a, encDeg(a) - (float)a.dir * SNAG_BACKOFF_DEG,
             HOME_SPEED_DPS, HOME_ACCEL_DPS2);
    return;
  }

  /* Index fired mid-search: that capture is smeared by sweep speed, so do
   * NOT zero on it. Ramp down smoothly (never dead-stop a moving stepper —
   * see axisRampDown), then overshoot IDX_OVERSHOOT_DEG past the CAPTURED
   * index count and creep back. The ramp-down travel (~2.6 deg from sweep
   * speed) costs nothing: the overshoot target is anchored to the captured
   * count, not to wherever the ramp ends. */
  if ((a.homePhase == H_FWD || a.homePhase == H_REV ||
       a.homePhase == H_EXT) && a.indexFound) {
    if (a.moving) {
      if (!a.decelOnly) {                  // initiate the ramp exactly once
        axisRampDown(a);
        serialWrite(F("IDX "));
        serialWrite(a.name);
        serialWriteLine(F(": index seen — overshooting 10 deg for slow re-capture"));
      }
      return;                              // wait for the ramp to finish
    }
    a.snagWatch   = false;                 // abandon the search leg cleanly
    a.snagged     = false;
    a.legPhase    = LEG_IDLE;
    a.indexArmed  = false;
    a.idxRoughEnc = a.encAtIndex;
    a.homePhase   = H_OVER;
    // ALWAYS overshoot to the POSITIVE side of the index, whichever sweep
    // direction first saw it: the slow re-capture then always approaches
    // moving negative, hitting the SAME edge of the index pulse every run.
    // (The pulse has physical width — capturing opposite edges on different
    // runs would shift home by that width. Repeatability requires one
    // canonical approach direction.)
    float overDeg = (float)a.idxRoughEnc * (360.0f / (float)ENC_CPR)
                  + IDX_OVERSHOOT_DEG;
    legStart(a, overDeg, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
    return;
  }

  /* Index re-fired during the slow creep: THAT capture is the zero. Same
   * ramp-then-act pattern (from 2 dps the ramp is a few milliseconds). */
  if (a.homePhase == H_CREEP && a.indexFound) {
    if (a.moving) {
      if (!a.decelOnly) axisRampDown(a);
      return;
    }
    a.snagWatch  = false;
    a.snagged    = false;
    a.legPhase   = LEG_IDLE;
    a.indexArmed = false;
    noInterrupts();
    a.enc -= a.encAtIndex;                 // refined index point = 0 deg
    interrupts();
    // This axis's frame is now the INDEX frame: sun geometry must shift.
    if (&a == &m1) frameElOff = HOME_EL_OFFSET_DEG;
    else           frameAzOff = HOME_AZ_OFFSET_DEG;
    serialWrite(F("IDX "));
    serialWrite(a.name);
    serialWriteLine(F(": zero refined on slow re-approach"));
    a.homePhase = H_RETURN;
    legStart(a, 0.0f, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
    return;
  }

  uint8_t r = legService(a);
  if (r == LEG_RUNNING) return;

  switch (a.homePhase) {
    case H_PREZERO:
      // The return-to-old-zero drive-back is only an optimization (keeps the
      // sweep inside the proven cable envelope). A snag here USED to be an
      // instant ERR 53 with no recovery. Instead, abandon the drive-back and
      // search from HERE: the axis is already halted (snagService cleared
      // a.moving), and H_FWD -> H_REV(365) -> H_EXT(365) supplies the reverse/
      // 365 recovery that H_PREZERO never had.
      if (r == LEG_SNAGGED) {
        serialWrite(F("HOME "));
        serialWrite(a.name);
        serialWriteLine(F(": prezero return blocked — searching from here"));
      }
      startSearchFwd(a);
      break;

    case H_FWD:
      // Reached the 180 mark OR properly snagged. On a snag, EL first rests +
      // backs off + retries the SAME direction (user spec: "move until it
      // snags twice"); AZ already gets a leg-level punch retry. Only once the
      // retries are spent is this direction BLOCKED — then reverse far enough
      // that total coverage spans 365 deg (reverse target = current - sign*365,
      // which brackets the 1/rev index).
      if (r == LEG_SNAGGED) {
        if (&a == &m1 && a.searchRetry < HOME_SEARCH_RETRY) {
          a.searchRetry++;
          enterSearchRest(a);
          return;
        }
        serialWrite(F("HOME "));
        serialWrite(a.name);
        serialWriteLine(F(": forward sweep blocked both attempts — reversing"));
      }
      a.searchRetry = 0;                     // fresh budget for the reverse
      a.homePhase   = H_REV;
      serialWrite(F("HOME "));
      serialWrite(a.name);
      serialWrite(F(": reverse sweep ("));
      serialWrite((int)homeSweepTotal(a));
      serialWriteLine(F(" deg total coverage)"));
      searchLegStart(a, encDeg(a) - homeSweepSign(a) * homeSweepTotal(a));
      break;

    case H_REV:
      if (r == LEG_SNAGGED) {
        // EL: rest + retry the reverse before it counts as blocked. This also
        // recovers a post-stall sync stumble on the reverse (the reverse is
        // launched right after the forward snag) instead of mistaking it for
        // a real range limit.
        if (&a == &m1 && a.searchRetry < HOME_SEARCH_RETRY) {
          a.searchRetry++;
          enterSearchRest(a);
          return;
        }
        // BOTH directions are blocked (two catches). Rotate the OTHER way
        // (away from this reverse catch) far enough for full coverage (365
        // deg EL / 320 deg AZ), watching the index the whole way (user spec)
        // — then home + validate if the index is crossed.
        a.searchRetry = 0;
        a.homePhase   = H_EXT;
        serialWrite(F("HOME "));
        serialWrite(a.name);
        serialWrite(F(": caught both ways — rotating the other way "));
        serialWrite((int)homeSweepTotal(a));
        serialWriteLine(F(" deg"));
        searchLegStart(a, encDeg(a) + homeSweepSign(a) * homeSweepTotal(a));
      } else {
        // Full CLEAN coverage and the index never fired: the
        // index channel (wiring/encoder) is the problem, not a cable.
        uint8_t hc = (&a == &m1) ? PY_ERR_HOME_M1 : PY_ERR_HOME_M2;
        latchFault(ST_FAULT_HOME, hc);
        serialWrite(F("ERR "));
        serialWrite(hc);
        serialWrite(F(" HOME FAIL "));
        serialWrite(a.name);
        serialWriteLine(F(": index not found in full-coverage sweep"));
      }
      break;

    case H_EXT:
      // The other-way 365 deg leg (both directions were already blocked).
      if (r == LEG_SNAGGED) {
        if (&a == &m1 && a.searchRetry < HOME_SEARCH_RETRY) {
          a.searchRetry++;                   // one gentle retry here too
          enterSearchRest(a);
          return;
        }
        catchFault(a);                       // genuinely blocked every way
        return;
      }
      // Completed with no index: the entire reachable range has now been
      // covered both ways, so the index channel is broken — a homing fault.
      {
        uint8_t hc = (&a == &m1) ? PY_ERR_HOME_M1 : PY_ERR_HOME_M2;
        latchFault(ST_FAULT_HOME, hc);
        serialWrite(F("ERR "));
        serialWrite(hc);
        serialWrite(F(" HOME FAIL "));
        serialWrite(a.name);
        serialWriteLine(F(": index not found across the full reachable range"));
      }
      break;

    case H_RETRY_BACK:
      // Backoff run-up done (a snag on a short backoff is ignored — just
      // proceed). Re-issue the SAME sweep, now starting cleanly from rest.
      a.homePhase = a.searchResumePhase;
      serialWrite(F("HOME "));
      serialWrite(a.name);
      serialWriteLine(F(": retrying the sweep"));
      searchLegStart(a, a.searchResumeTarget);
      break;

    case H_OVER:
      if (r == LEG_SNAGGED) { if (qcalHandleSnag(a)) return; catchFault(a); return; }
      a.indexFound = false;
      a.indexArmed = true;
      a.homePhase  = H_CREEP;
      // Canonical approach: always creep NEGATIVE onto the index edge.
      legStart(a, encDeg(a) - IDX_RESWEEP_DEG, IDX_CREEP_DPS, HOME_ACCEL_DPS2);
      break;

    case H_CREEP:
      if (r == LEG_SNAGGED) { if (qcalHandleSnag(a)) return; catchFault(a); return; }
      if (qcalActive) {
        // Quick-recal window swept with NO index pulse: the hint was wrong
        // (the mount moved while the link was dead, or the frame had
        // drifted). NEVER zero on an unverified hint — report the miss and
        // let the CAL owner fall back to the full homing sequence.
        a.indexArmed = false;
        a.homePhase  = H_DONE;             // a.homed stays false = miss
        serialWrite(F("QCAL "));
        serialWrite(a.name);
        serialWriteLine(F(": index NOT in the hint window"));
        return;
      }
      // Creep finished without a re-capture (shouldn't happen — the pulse
      // was bracketed). Fall back to the fast-sweep capture, don't fault.
      a.indexArmed = false;
      noInterrupts();
      a.enc -= a.idxRoughEnc;
      interrupts();
      if (&a == &m1) frameElOff = HOME_EL_OFFSET_DEG;   // index frame now
      else           frameAzOff = HOME_AZ_OFFSET_DEG;
      serialWrite(F("IDX "));
      serialWrite(a.name);
      serialWriteLine(F(": slow re-capture missed, using fast-sweep zero"));
      a.homePhase = H_RETURN;
      legStart(a, 0.0f, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
      break;

    case H_RETURN:
      if (r == LEG_SNAGGED) { if (qcalHandleSnag(a)) return; catchFault(a); return; }
      a.homePhase = H_DONE;
      a.homed     = true;
      a.zeroKnown = true;
      serialWrite(F("HOME "));
      serialWrite(a.name);
      serialWriteLine(F(": zeroed on index and back at 0"));
      break;

    default:
      break;
  }
}

/* ------------------- Post-home +/-180 verification ---------------------- *
 * NOTE hard-limit bypass: the +/-180 watchdog only runs while ST_MOVING /
 * ST_DANCING. All verify motion (including the 365 deg recovery unwind)
 * runs under ST_HOMING, so it is legally allowed past +/-180.
 * A leg that completes IS the range proof: the continuous 10 deg snag watch
 * guarantees the encoder tracked the command the entire way, so no separate
 * end-of-leg position check is needed (v3.x's settle-and-compare is gone). */

Axis *verStageAxis(uint8_t s) {
  switch (s) {
    case V_EL_FWD: case V_EL_BACK: case V_EL_RET: case V_EL_PARK: return &m1;
    case V_AZ_FWD: case V_AZ_BACK: case V_AZ_RET:                 return &m2;
    default:                                                      return verFailAxis;
  }
}

/* EL park elevation for the AZ validation sweep: the configured value, but
 * never above the sun window's lower edge minus clearance — removes the
 * assumption that -30 is safe for every possible sun elevation. */
float elAzValPark() {
  // Frame-aware: during CAL this runs in the INDEX frame, where the sun's
  // elevation is sunEl + HOME_EL_OFFSET. The park just has to sit OUTSIDE
  // the sun's elevation band — either side is equally safe — so keep the
  // configured park if it's already clear, else move to the NEAREST band
  // edge (with a sun at index el -110, parking below would mean -132 while
  // the configured -30 is already 58 deg above the band).
  float lo = sunElF() - sunHalfEl() - SUN_CLEAR_DEG;
  float hi = sunElF() + sunHalfEl() + SUN_CLEAR_DEG;
  float p  = EL_AZVAL_PARK_DEG;
  if (p > lo && p < hi)
    p = (p - lo <= hi - p) ? lo : hi;      // nearest safe side
  if (p >  LIMIT_DEG) p =  LIMIT_DEG;
  if (p < -LIMIT_DEG) p = -LIMIT_DEG;
  return p;
}

/* All verification-machine motion runs at VERIFY_SPEED_DPS — standard
 * open-loop profile legs with the stall watchdog armed, at under half the
 * normal movement speed (user spec). */
void verEnter(uint8_t s) {
  verStage = s;
  switch (s) {
    case V_EL_FWD:
      serialWriteLine(F("VERIFY: EL +/-180 sweep (AZ parked 90 deg off-sun)"));
      legStart(m1, +LIMIT_DEG, VERIFY_SPEED_DPS, HOME_ACCEL_DPS2);
      break;
    case V_EL_BACK: legStart(m1, -LIMIT_DEG, VERIFY_SPEED_DPS, HOME_ACCEL_DPS2); break;
    case V_EL_RET:  legStart(m1,       0.0f, VERIFY_SPEED_DPS, HOME_ACCEL_DPS2); break;
    case V_EL_PARK:
      serialWrite(F("VERIFY: EL OK; parking EL at "));
      serialWrite(elAzValPark(), 1);
      serialWriteLine(F(" deg for the AZ return home"));
      legStart(m1, elAzValPark(), VERIFY_SPEED_DPS, HOME_ACCEL_DPS2);
      break;
    /* AZ range verification is intentionally REMOVED (user spec): the AZ
     * axis is not swept over its range. These sweep stages are retained but
     * no longer entered by the sequence (see verService); if ever reached
     * they stay inside the -140..+180 ROM. Only V_AZ_RET still runs — it
     * simply returns AZ to home so the mount ends CAL at a known (EL park,0).*/
    case V_AZ_FWD:
      serialWriteLine(F("VERIFY: AZ sweep (disabled — within -140..+180 ROM)"));
      legStart(m2, +M2_MAX_DEG, VERIFY_SPEED_DPS, HOME_ACCEL_DPS2);
      break;
    case V_AZ_BACK: legStart(m2, M2_MIN_DEG, VERIFY_SPEED_DPS, HOME_ACCEL_DPS2); break;
    case V_AZ_RET:
      serialWriteLine(F("VERIFY: AZ verification skipped — returning AZ to home"));
      legStart(m2, 0.0f, VERIFY_SPEED_DPS, HOME_ACCEL_DPS2);
      break;
    case V_REC_RET:
      serialWrite(F("VERIFY "));
      serialWrite(verFailAxis->name);
      serialWrite(F(": returning to zero for a "));
      serialWrite((int)homeUnwindDeg(*verFailAxis));
      serialWriteLine(F(" deg recovery unwind"));
      legStart(*verFailAxis, 0.0f, VERIFY_SPEED_DPS, HOME_ACCEL_DPS2);
      break;
    case V_UNWIND:
      serialWrite(F("VERIFY "));
      serialWrite(verFailAxis->name);
      serialWrite(F(": unwinding "));
      serialWrite(-verFailDir * (int)homeUnwindDeg(*verFailAxis));
      serialWriteLine(F(" deg from zero (away from the snag, homing limit bypassed)"));
      legStart(*verFailAxis, -(float)verFailDir * homeUnwindDeg(*verFailAxis),
               VERIFY_SPEED_DPS, HOME_ACCEL_DPS2);
      break;
    default:
      break;
  }
}

/* Verification passed on both axes: homing is officially complete. The
 * mount rests at (EL_AZVAL_PARK_DEG, 0); the POS line below tells the host
 * exactly where it is, and its post-CAL offset MOVE plans from there. */
/* Shared homing completion (full CAL after validation, or quick re-cal
 * with validation skipped): anchor the hold, announce HOMED, and walk the
 * host's expected status sequence (13 then 0). */
void homingFinish() {
  verStage = V_IDLE;
  calStage = C_NONE;
  qcalActive = false;
  calSkipVerify = false;           // consumed; next full CAL verifies again
  m1.targetDeg = encDeg(m1);       // anchor the active hold HERE, so the
  m2.targetDeg = encDeg(m2);       // station keeper never chases a stale
  reqEl = m1.targetDeg;            // pre-CAL target
  reqAz = m2.targetDeg;

  // Arm continuous index re-zero: homing left each index at enc 0, and that
  // is where every future pass through home must latch. Re-arm the ISR so the
  // 1 kHz service sees the next crossing.
  movesSinceRecal = 0;
  armIndexRezero(m1);
  armIndexRezero(m2);

  serialWriteLine(F("HOMED"));
  printPos();
  pushStatus(PY_HOMING_DONE);      // host's CAL loop waits for this (13)...
  sysState = ST_IDLE;
  pushStatus(PY_IDLE);             // ...then for IDLE (0)
}

void verifyFinish() {
  serialWriteLine(F("VERIFY: EL +/-180 range confirmed (AZ verification removed)"));
  homingFinish();
}

void verService() {
  if (verStage == V_REHOME) {
    homeService(*verFailAxis);
    if (isFault()) return;
    if (verFailAxis->homePhase == H_DONE) {
      serialWrite(F("VERIFY "));
      serialWrite(verFailAxis->name);
      serialWriteLine(F(": re-homed after unwind — retrying the sweep"));
      // EL failure: redo the EL sweep from the top (its park + AZ return
      // follow naturally). AZ failure: AZ range verification is removed, so
      // just return AZ home (EL is already parked below the sun window).
      verEnter((verFailAxis == &m2) ? V_AZ_RET : V_EL_FWD);
    }
    return;
  }

  Axis *a = verStageAxis(verStage);
  if (a == NULL) return;
  uint8_t r = legService(*a);
  if (r == LEG_RUNNING) return;
  // End-of-leg guard: a RANGE-PROOF sweep or the sun-safety park that
  // "completed" far from its target (axis advancing too slowly to
  // stall-trip) has NOT proven anything — route it into the snag recovery.
  // RETURN legs are exempt along with the recovery legs: they are mere
  // repositioning, every subsequent step is encoder-anchored, and (bench
  // 2026-07-15) a return ending 15.8 deg short churned the recovery ladder
  // pointlessly against a sticky spot when nothing was actually wrong.
  if (r == LEG_DONE &&
      verStage != V_REC_RET && verStage != V_UNWIND &&
      verStage != V_EL_RET  && verStage != V_AZ_RET &&
      legShort(*a)) {
    serialWrite(F("VERIFY "));
    serialWrite(a->name);
    serialWrite(F(": leg ended "));
    serialWrite(fabsf(encDeg(*a) - a->legTargetDeg), 1);
    serialWriteLine(F(" deg short of target — treating as snag"));
    r = LEG_SNAGGED;
  }

  if (r == LEG_DONE) {
    switch (verStage) {
      case V_EL_FWD:  verEnter(V_EL_BACK); break;
      case V_EL_BACK: verEnter(V_EL_RET);  break;
      case V_EL_RET:  verEnter(V_EL_PARK); break;
      // AZ range verification removed (user spec): skip the AZ +/- sweep and
      // go straight to returning AZ home, then finish.
      case V_EL_PARK: verEnter(V_AZ_RET);  break;
      case V_AZ_FWD:  verEnter(V_AZ_BACK); break;   // (no longer reached)
      case V_AZ_BACK: verEnter(V_AZ_RET);  break;   // (no longer reached)
      case V_AZ_RET:  verifyFinish();      break;
      case V_REC_RET: verEnter(V_UNWIND);  break;
      case V_UNWIND:
        // A full turn away from the snag: the wrap is released HERE.
        // Re-anchor and re-home from this position (the index is 1/rev, so
        // it is reachable from anywhere) instead of driving back into the
        // old zero, which would just re-wrap the cable.
        noInterrupts();
        verFailAxis->enc = 0;
        interrupts();
        verFailAxis->zeroKnown = false;    // home from HERE, no drive-back
        serialWrite(F("VERIFY "));
        serialWrite(verFailAxis->name);
        serialWriteLine(F(": unwind complete, re-running homing"));
        verStage = V_REHOME;
        homeBegin(*verFailAxis);
        break;
      default:
        break;
    }
    return;
  }

  /* LEG_SNAGGED */
  if (verStage == V_REC_RET || verStage == V_UNWIND) {
    catchFault(*a);                        // recovery path itself is blocked
    return;
  }
  // Sweep leg (or the EL park) properly snagged: recovery cycle.
  verFailAxis = a;
  verFailDir  = a->dir;
  a->verRetry++;
  if (a->verRetry > VERIFY_MAX_RETRY) { catchFault(*a); return; }
  serialWrite(F("VERIFY "));
  serialWrite(a->name);
  serialWrite(F(": range blocked at "));
  serialWrite(encDeg(*a), 1);
  serialWrite(F(" deg (recovery attempt "));
  serialWrite(a->verRetry);
  serialWrite('/');
  serialWrite(VERIFY_MAX_RETRY);
  serialWriteLine(')');
  verEnter(V_REC_RET);
}

/* --------------------------- CAL sequencing ---------------------------- */
/* ---------------------- Quick re-calibration (CALQ) ---------------------- *
 * After a link-death reset the controller loses its frame but the MOUNT
 * HASN'T MOVED — the host still knows, from the TGT telemetry it received
 * until the moment of death, where the mount physically sits and therefore
 * where each index pulse must be. "CALQ <d_el> <d_az>" hands those
 * expectations over (deg relative to the current position) and the
 * firmware, instead of blind 365-deg sweeps plus two minutes of
 * validation:
 *   1. runs to hint+QCAL_WIN_DEG and creeps back across the +/-5 deg
 *      window with the index armed — the SAME canonical negative-edge
 *      capture a full CAL uses, so zero quality is identical;
 *   2. AZ first, then EL (full-CAL ordering);
 *   3. SKIPS the +/-180 validation (user spec): the last full CAL proved
 *      the range and nothing has physically changed — finding both
 *      indexes inside their windows is itself the frame check;
 *   4. on ANY miss or snag falls back to the full CAL sequence
 *      automatically, so the host flow is identical either way.
 * Quick path: ~15 s instead of ~2 min of blind sweeps. */
bool qcalHandleSnag(Axis &a) {
  if (!qcalActive) return false;
  a.homePhase = H_DONE;                    // a.homed stays false = miss
  serialWrite(F("QCAL "));
  serialWrite(a.name);
  serialWriteLine(F(": snag inside the quick window — full CAL instead"));
  return true;
}

void qcalBegin(Axis &a, float deltaDeg) {
  a.homed      = false;
  a.indexFound = false;
  a.indexArmed = false;
  float hintDeg = encDeg(a) + deltaDeg;
  // Pretend the fast sweep already captured the index AT THE HINT: homing
  // then runs its normal H_OVER -> H_CREEP slow negative re-capture, i.e.
  // the identical canonical-edge zero a full CAL produces.
  a.idxRoughEnc = lround((double)hintDeg / 360.0 * (double)ENC_CPR);
  a.homePhase   = H_OVER;
  serialWrite(F("QCAL "));
  serialWrite(a.name);
  serialWrite(F(": expecting index at "));
  serialWrite(hintDeg, 1);
  serialWriteLine(F(" deg — sweeping the +/-5 deg window"));
  legStart(a, hintDeg + QCAL_WIN_DEG, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
}

/* Start the FULL homing sequence (shared by the CAL command, a CALQ with
 * bad hints, and every quick-recal fallback). */
void calBeginFull() {
  qcalActive  = false;
  verStage    = V_IDLE;
  verFailAxis = NULL;
  m1.verRetry = m2.verRetry = 0;
  m1.idxRezeroArmed = m2.idxRezeroArmed = false;  // homing owns the index now
  calStage       = C_EL_PARK;
  calElStartDeg  = encDeg(m1);
  calElParkRetry = false;
  serialWriteLine(F("CAL: EL backing off 45 deg (blind) before AZ homing"));
  legStart(m1, calElStartDeg + EL_BLIND_PARK_DEG, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
  sysState = ST_HOMING;
  pushStatus(PY_HOMING_M2);
}

void qcalFallback() {
  // Educated re-cal missed the index: fall back to a full homing sweep, but
  // SKIP the +/-180 validation spins (user spec) — the last full CAL already
  // proved the range and nothing has physically changed.
  calSkipVerify = true;
  serialWriteLine(F("QCAL: index missed — full homing sweep (validation skipped)"));
  calBeginFull();
}

/* Educated (quick) re-calibration, fired autonomously every MOVES_PER_RECAL
 * moves. Two phases so the whole thing presents to the host as one homing
 * cycle (STATUS 12 ... 13 0), which the bridge already holds moves behind:
 *   Phase 1 (beginEducatedRecal): a SUN-SAFE slew back to 0,0 — the trip
 *     home from an arbitrary target must be routed, exactly like a MOVE.
 *   Phase 2 (beginRecalSweep, fired when phase 1 completes): the +/-5 deg
 *     index sweep around home, re-zeroing on the pulse. Validation skipped;
 *     a miss falls back to a full sweep (also unvalidated) via qcalFallback. */
void beginEducatedRecal() {
  sunLegIdx = 0;
  uint8_t nw = planSunRoute(encDeg(m1), encDeg(m2), 0.0f, 0.0f);
  if (nw == 255) {
    serialWriteLine(F("RECAL: no sun-safe route to 0,0 — skipping educated re-cal"));
    return;                          // try again after the next MOVES_PER_RECAL
  }
  sunLegCount       = nw;
  reqEl = reqAz     = 0.0f;
  recalSweepPending = true;
  serialWriteLine(F("RECAL: educated re-cal — slewing to 0,0 before the "
                    "+/-5 deg index sweep"));
  pushStatus(PY_HOMING_M2);          // whole cycle reads as homing to the host
  startSlewTo(0.0f, 0.0f);           // sun-routed; pushes SLEWING after
}

/* Phase 2: at 0,0 now, sweep the +/-QCAL_WIN_DEG window for the index. */
void beginRecalSweep() {
  qcalActive        = true;
  calSkipVerify     = true;
  m1.idxRezeroArmed = m2.idxRezeroArmed = false;  // homing owns the index now
  // Hint = distance from HERE back to enc 0 (~0, we're at home). AZ runs
  // first; EL does not move during the AZ window, so its hint stays valid.
  qcalElDelta = -encDeg(m1);
  calStage    = C_Q_AZ;
  sysState    = ST_HOMING;
  serialWriteLine(F("RECAL: sweeping +/-5 deg for the index"));
  pushStatus(PY_HOMING_M2);
  qcalBegin(m2, -encDeg(m2));
}

void calService() {
  switch (calStage) {
    case C_EL_PARK: {                      // EL backing off 45 deg (blind)
      uint8_t r = legService(m1);
      if (r == LEG_RUNNING) return;
      if (r == LEG_DONE && legShort(m1)) r = LEG_SNAGGED;  // ended far short
      if (r == LEG_SNAGGED) {
        if (!calElParkRetry) {
          // The -45 direction is properly blocked (an EL snag is immediate,
          // no punch): go +315 from the ORIGINAL rest position instead —
          // the same physical park, reached the long way around. This is
          // the LAST resort before a fault, so unlike every other EL leg it
          // gets one backoff+punch attempt at a jam. Only a snag that
          // survives the punch (= both directions blocked) is a fault.
          calElParkRetry = true;
          serialWriteLine(F("CAL: EL -45 park blocked — going +315 from start instead"));
          legStart(m1, calElStartDeg + EL_BLIND_PARK_RETRY_DEG,
                   HOME_SPEED_DPS, HOME_ACCEL_DPS2);
          m1.legPunchTries = 1;            // one push-through allowed here
          return;
        }
        catchFault(m1);                    // blocked both ways
        return;
      }
      serialWriteLine(F("CAL: EL backed off; homing AZ"));
      calStage = C_AZ_HOME;
      homeBegin(m2);
      break;
    }
    case C_AZ_HOME:
      homeService(m2);
      if (isFault()) return;
      if (m2.homePhase == H_DONE) {
        calStage = C_AZ_PARK;
        // Compute the sun-safe park NOW, in the frame AZ was just zeroed
        // into (index frame). "90 deg off-sun" has TWO solutions — take
        // whichever is CLOSER to where AZ sits (shorter drive, same
        // safety), keep the other as the snag fallback.
        float c1 = wrap180(sunAzF() + CAL_SUN_OFF_DEG);
        float c2 = wrap180(sunAzF() - CAL_SUN_OFF_DEG);
        float here = encDeg(m2);
        if (fabsf(c1 - here) <= fabsf(c2 - here)) { calSafeAz = c1; calAzParkAlt = c2; }
        else                                      { calSafeAz = c2; calAzParkAlt = c1; }
        calAzParkRetry = false;
        serialWrite(F("CAL: AZ homed, parking at "));
        serialWrite(calSafeAz, 1);
        serialWriteLine(F(" deg (sun-safe) before EL homing"));
        // HOME_SPEED (now = slew rate): fast enough to carry momentum
        // through the bearing stiction that made slow CAL legs stutter.
        legStart(m2, calSafeAz, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
      }
      break;
    case C_AZ_PARK: {                      // AZ slewing 90 deg off the sun
      uint8_t r = legService(m2);
      if (r == LEG_RUNNING) return;
      if (r == LEG_DONE && legShort(m2)) r = LEG_SNAGGED;  // ended far short
      if (r == LEG_SNAGGED) {
        if (!calAzParkRetry) {
          // The way to the nearer off-sun point is blocked: mirror to the
          // OTHER 90-deg-off-sun azimuth — opposite direction of travel,
          // equally sun-safe. Only both blocked is a fault.
          calAzParkRetry = true;
          calSafeAz = calAzParkAlt;
          serialWrite(F("CAL: AZ park blocked — trying the other off-sun point at "));
          serialWrite(calSafeAz, 1);
          serialWriteLine(F(" deg"));
          legStart(m2, calSafeAz, HOME_SPEED_DPS, HOME_ACCEL_DPS2);
          return;
        }
        catchFault(m2);
        return;
      }
      calStage = C_EL_HOME;
      pushStatus(PY_HOMING_M1);
      serialWriteLine(F("CAL: AZ sun-safe; homing EL"));
      homeBegin(m1);                       // homes fine from the backed-off spot
      break;
    }
    case C_EL_HOME:
      homeService(m1);
      if (isFault()) return;
      if (m1.homePhase == H_DONE) {
        if (calSkipVerify) {               // educated-recal fallback: range
          serialWriteLine(F("CAL: both axes homed — validation skipped"));
          homingFinish();                  // already proven; don't re-spin
        } else {
          calStage = C_VERIFY;
          serialWriteLine(F("CAL: both axes homed — verifying EL +/-180 range (AZ verify removed)"));
          verEnter(V_EL_FWD);              // EL validates; AZ then just returns home
        }
      }
      break;
    case C_VERIFY:
      verService();
      break;
    case C_Q_AZ:                           // quick re-cal: AZ window
      homeService(m2);
      if (isFault()) return;
      if (m2.homePhase == H_DONE) {
        if (!m2.homed) { qcalFallback(); return; }
        calStage = C_Q_EL;
        pushStatus(PY_HOMING_M1);
        serialWriteLine(F("QCAL: AZ re-zeroed on index; quick-homing EL"));
        qcalBegin(m1, qcalElDelta);        // EL hasn't moved since CALQ
      }                                    // arrived, so its delta holds
      break;
    case C_Q_EL:                           // quick re-cal: EL window
      homeService(m1);
      if (isFault()) return;
      if (m1.homePhase == H_DONE) {
        if (!m1.homed) { qcalFallback(); return; }
        serialWriteLine(F("QCAL: both axes re-zeroed — +/-180 validation skipped"));
        homingFinish();
      }
      break;
    default:
      break;
  }
}

/* --------------------------- Sun keep-out ------------------------------- */
static inline float sunHalfEl() { return SUN_WIN_EL_DEG * 0.5f + CAM_FOV_DEG * 0.5f; }
static inline float sunHalfAz() { return SUN_WIN_AZ_DEG * 0.5f + CAM_FOV_DEG * 0.5f; }

/* Is (el,az) inside the FOV-inflated sun window (plus pad)? Frame-aware:
 * compares against the sun's position in the CURRENT frame. */
bool sunBlocked(float el, float az, float pad) {
  return fabsf(el - sunElF()) < sunHalfEl() + pad &&
         fabsf(wrap180(az - sunAzF())) < sunHalfAz() + pad;
}

/* The over-the-top equivalent attitude: (el, az) and (flipEl(el), az+180)
 * aim the boresight at the same sky direction. The flip relation is a
 * property of the PHYSICAL mount geometry (el=0 = physical home), so it is
 * computed in the physical frame and mapped back through the current
 * frame offset — correct both mid-CAL (index frame) and after SETZERO. */
static inline float flipEl(float el) {
  float p = el - frameElOff;                       // to physical
  p = (p >= 0.0f) ? 180.0f - p : -180.0f - p;      // flip there
  return p + frameElOff;                           // back to current frame
}

/* Sky-aware sun check: is the DIRECTION (el,az) points at too close to the
 * sun, in EITHER of its two mount representations? A flipped attitude can
 * stare straight at the sun while its raw coordinates are nowhere near the
 * direct sun box, so every target and path sample must use this. */
bool sunBlockedSky(float el, float az, float pad) {
  return sunBlocked(el, az, pad) ||
         sunBlocked(flipEl(el), wrap180(az + 180.0f), pad);
}

/* Would the CONCURRENT-axis path from (e0,a0) to (e1,a1) clip the window?
 * Both axes slew at the same deg/s, so the path is a 45 deg diagonal until
 * the shorter axis arrives, then a straight run. Sampled every 5 deg —
 * far finer than the >=20 deg window half-widths. */
bool sunPathBlocked(float e0, float a0, float e1, float a1) {
  float de = e1 - e0, da = a1 - a0;
  float ade = fabsf(de), ada = fabsf(da);
  float span = (ade > ada) ? ade : ada;
  int n = (int)(span / 5.0f) + 1;
  for (int i = 0; i <= n; i++) {
    float trav = span * (float)i / (float)n;
    float e = e0 + ((trav < ade) ? (de > 0 ? trav : -trav) : de);
    float a = a0 + ((trav < ada) ? (da > 0 ? trav : -trav) : da);
    if (sunBlockedSky(e, a, 0.0f)) return true;   // both representations
  }
  return false;
}

/* Staged sun-safe route (per host guidance): EL moves as close to the final
 * elevation as possible but STOPS outside the box's elevation band; AZ then
 * clears the box's azimuth band; only then does EL finish. Every generated
 * leg is single-axis, so the concurrent diagonal can't cut a corner into
 * the window, and every leg (including the final implicit one to the true
 * target) is re-verified against the sampled path model.
 * Returns: 0 = direct path already clear, 1..4 = waypoint count,
 *          255 = no safe route (caller must refuse the MOVE with ERR 57). */
uint8_t planSunRoute(float e0, float a0, float e1, float a1) {
  if (!sunPathBlocked(e0, a0, e1, a1)) return 0;
  // The sun appears at TWO places in command space: the direct box around
  // (sunEl, sunAz) and its over-the-top image around (flipEl(sunEl),
  // sunAz+180). Route around whichever image this move actually operates
  // near (flipped hemisphere when |el| > 90); the sampled re-verification
  // below checks BOTH images, so a wrong pick refuses rather than risks.
  float bcEl = sunElF(), bcAz = sunAzF();
  if (fmaxf(fabsf(e0 - frameElOff), fabsf(e1 - frameElOff)) > 90.0f) {
    bcEl = flipEl(sunElF());                 // flipped image, current frame
    bcAz = wrap180(sunAzF() + 180.0f);
  }
  float elLo = bcEl - sunHalfEl() - SUN_CLEAR_DEG;
  float elHi = bcEl + sunHalfEl() + SUN_CLEAR_DEG;
  float azLo = bcAz - sunHalfAz() - SUN_CLEAR_DEG;
  float azHi = bcAz + sunHalfAz() + SUN_CLEAR_DEG;
  if (elHi >  LIMIT_DEG) elHi =  LIMIT_DEG;    // hold points stay commandable
  if (elLo < -LIMIT_DEG) elLo = -LIMIT_DEG;

  uint8_t n  = 0;
  float   ce = e0;                             // elevation as the plan builds
  bool escaped = false;                        // leg 0 is an unavoidable exit

  if (sunBlockedSky(e0, a0, SUN_CLEAR_DEG)) {
    // (a0) EMERGENCY ESCAPE: the boresight is already staring into the sun
    //      window (sun drifted onto a parked mount, or a bare-SUN test
    //      point). Exit through the NEAREST elevation edge of the active
    //      box image — shortest sun exposure — before any other planning.
    //      This leg unavoidably crosses part of the box, so it is exempted
    //      from the path re-verification below.
    float edge = (fabsf(elHi - ce) <= fabsf(ce - elLo)) ? elHi : elLo;
    sunWpEl[n] = edge; sunWpAz[n] = a0; n++;
    ce = edge;
    escaped = true;
  } else if (ce > elLo && ce < elHi) {
    // (a) Starting inside the el band (legal only when az is clear of the
    //     az band): pop EL out to an edge BEFORE any azimuth motion,
    //     preferring the side the target elevation is on so no crossing is
    //     needed later.
    float edge;
    if      (e1 >= elHi) edge = elHi;
    else if (e1 <= elLo) edge = elLo;
    else                 edge = (ce >= bcEl) ? elHi : elLo;
    sunWpEl[n] = edge; sunWpAz[n] = a0; n++;
    ce = edge;
  }

  // (b) EL as close to the target as possible while staying on THIS side of
  //     the band (this is the "stop outside the elevation of the bounding
  //     box" step — safe even if the current azimuth is inside the band).
  float eHold = (ce <= elLo) ? fminf(e1, elLo) : fmaxf(e1, elHi);
  if (fabsf(eHold - ce) > 0.01f) {
    sunWpEl[n] = eHold; sunWpAz[n] = a0; n++;
    ce = eHold;
  }

  // (c) If the target elevation is across/inside the band, EL may only
  //     finish once AZ has cleared the box's azimuth band.
  bool needCross = fabsf(eHold - e1) > 0.01f;
  if (needCross) {
    bool a1InBand = (a1 > azLo && a1 < azHi);
    if (a1InBand) {
      // Target azimuth sits inside the band: cross to the band edge nearest
      // the target with EL held clear, finish EL there, then the implicit
      // final leg slides AZ in at the (now safe) target elevation.
      float edge = (a1 <= bcAz) ? azLo : azHi;
      sunWpEl[n] = ce; sunWpAz[n] = edge; n++;
      sunWpEl[n] = e1; sunWpAz[n] = edge; n++;
    } else {
      // Target azimuth is clear of the band: take AZ all the way there with
      // EL held, then the implicit final leg is EL-only (az outside band).
      sunWpEl[n] = ce; sunWpAz[n] = a1; n++;
    }
  }
  // else: EL business is done (eHold == e1, outside the band) and the
  // implicit final leg is AZ-only at a safe elevation.

  // Belt-and-braces: re-verify every leg, including the final one, against
  // the sampled concurrent-path model. Refuse the move if anything clips.
  // (An emergency-escape first leg is exempt — it starts inside the box.)
  float pe = e0, pa = a0;
  for (uint8_t i = 0; i <= n; i++) {
    float we = (i < n) ? sunWpEl[i] : e1;
    float wa = (i < n) ? sunWpAz[i] : a1;
    if (!(escaped && i == 0) && sunPathBlocked(pe, pa, we, wa)) return 255;
    pe = we; pa = wa;
  }
  return n;
}

/* ------------------------ Host protocol output -------------------------- */
const __FlashStringHelper *pyName(uint8_t c) {
  switch (c) {
    case PY_IDLE:         return F("IDLE");
    case PY_HOMING_M1:    return F("HOMING_M1");
    case PY_HOMING_M2:    return F("HOMING_M2");
    case PY_HOMING_DONE:  return F("HOMING_COMPLETE");
    case PY_SLEWING:      return F("SLEWING");
    case PY_ERR_HOME_M1:  return F("ERROR_HOMING_M1");
    case PY_ERR_HOME_M2:  return F("ERROR_HOMING_M2");
    case PY_ERR_CATCH_M1: return F("ERROR_CATCH_M1");
    case PY_ERR_CATCH_M2: return F("ERROR_CATCH_M2");
    case PY_ERR_LIMIT_M1: return F("ERROR_M1_STEP_LIMIT");
    case PY_ERR_LIMIT_M2: return F("ERROR_M2_STEP_LIMIT");
    case PY_ERR_SUN:      return F("ERROR_SUN_FACING");
    case PY_ERR_CMD:      return F("ERROR_COMMAND_UNREC");
    case PY_ERR_ANGLE:    return F("ERROR_INVALID_ANGLE");
    case PY_ERR_UNDERFLOW:return F("ERROR_BYTE_UNDERFLOW");
    default:              return F("ERROR_GENERAL");
  }
}

uint8_t pyStatusCode() {
  switch (sysState) {
    case ST_MOVING:
    case ST_DANCING:     return PY_SLEWING;
    case ST_HOMING:      return (calStage == C_EL_PARK ||
                                 calStage == C_AZ_HOME ||
                                 calStage == C_AZ_PARK ||
                                 calStage == C_Q_AZ)   ? PY_HOMING_M2
                                                       : PY_HOMING_M1;
    case ST_FAULT_HOME:
    case ST_FAULT_LIMIT: return faultCode;
    default:             return PY_IDLE;   // IDLE and NOT_HOMED both report 0
  }
}

void pushStatus(uint8_t c) {
  serialWrite(F("STATUS "));
  serialWrite(c);
  serialWrite(' ');
  serialWriteLine(pyName(c));
}

void pushErrMsg(uint8_t c, const __FlashStringHelper *msg) {
  serialWrite(F("ERR "));
  serialWrite(c);
  serialWrite(' ');
  serialWriteLine(msg);
}

/* POS reports the true BORESIGHT pointing direction relative to mechanical
 * zero — in whichever of its two representations is closer to the LAST
 * HOST-REQUESTED target (reqEl/reqAz). So whether you command (30, 170) or
 * its flip (150, -10), POS reads back in the same form you used and error
 * readouts are accurate, regardless of which attitude the mount actually
 * took. Raw attitude stays visible via the FLIP log line, HOME_LOG, and
 * the DONE residuals (servo error in the executed frame). Note: streamed mid-move, a
 * zenith crossing makes az step 180 deg in one sample — that IS what the
 * boresight azimuth does through zenith. */
void printPos() {
  float el = encDeg(m1), az = encDeg(m2);
  float fe = flipEl(el), fa = wrap180(az + 180.0f);
  // BOTH axes are angles: distances to the request must be measured ON THE
  // CIRCLE (-179.57 is 0.43 deg from +180, not 359 deg; an elevation of
  // -179.68 is 0.32 deg from a requested +180 the same way — comparing el
  // linearly picked the wrong representation after seam-straddling flips
  // and produced a nonsense az readback of -358 on the bench).
  float dRaw  = fmaxf(fabsf(wrap180(el - reqEl)), fabsf(wrap180(az - reqAz)));
  float dFlip = fmaxf(fabsf(wrap180(fe - reqEl)), fabsf(wrap180(fa - reqAz)));
  if (dFlip < dRaw) { el = fe; az = fa; }
  // Seam continuity on BOTH axes: report the alias numerically adjacent to
  // the request, so a commanded (180, -180) reads back as e.g.
  // (180.32, -178.74) and host-side (got - commanded) stays truthful.
  el = reqEl + wrap180(el - reqEl);
  az = reqAz + wrap180(az - reqAz);
  serialWrite(F("POS "));
  serialWrite(el, 2);
  serialWrite(' ');
  serialWriteLine(az, 2);
}

/* TGT is a pure debug/telemetry line for the host's motor CSV: the position
 * each axis is aiming for AT THIS MOMENT, in the RAW encoder frame
 * (m.targetDeg — during a sun-detour or snag recovery this is the current
 * waypoint, not the final request), plus the raw encoder position in the
 * same frame so target-vs-actual graphs line up without any flip/frame
 * bookkeeping. Every host parser ignores unknown line types by design, so
 * this is protocol-compatible by construction. Format:
 *   TGT <el_target> <az_target> <el_raw> <az_raw> */
void printTgt() {
  serialWrite(F("TGT "));
  serialWrite(m1.targetDeg, 2);
  serialWrite(' ');
  serialWrite(m2.targetDeg, 2);
  serialWrite(' ');
  serialWrite(encDeg(m1), 2);
  serialWrite(' ');
  serialWriteLine(encDeg(m2), 2);
}

void printHelp() {
  serialWriteLine(F("Commands (one per line, case-insensitive):"));
  serialWriteLine(F("  MOVE <m1> <m2>   slew to angles (m1 = elevation, m2 = azimuth)"));
  serialWriteLine(F("  STOP             halt all motion (works mid-slew)"));
  serialWriteLine(F("  CAL  (or HOME)   home both motors"));
  serialWriteLine(F("  SETZERO          reset encoder zero to current position"));
  serialWriteLine(F("  ACK              clear a latched error"));
  serialWriteLine(F("  STATUS           report status code"));
  serialWriteLine(F("  POS              report position -> POS <el> <az>"));
  serialWriteLine(F("  ID               report firmware id"));
  serialWriteLine(F("  DANCE            perform a little routine (requires CAL)"));
  serialWriteLine(F("  RAW <1|2|both> <cw|c> <dps> <sec>  BENCH: open-loop run, no checks; reports encoder change"));
  serialWriteLine(F("  SUN [<el> <az>]  set sun keep-out center (current frame); bare = TEST point at it"));
  serialWriteLine(F("  MOTOROFF         de-energize the drivers"));
  serialWriteLine(F("  HELP             reprint this list"));
  serialWriteLine(F("Every line needs the trailer #<seq>*<cs>  (cs = XOR of all bytes before '*', 2 hex digits)"));
  serialWriteLine(F("  e.g.  POS #7*78   -> reply ACK 7 then POS ...;  bad checksum -> NAK <seq> checksum"));
}

/* HOME_LOG row: t_ms m1_phase m1_deg m1_retry m1_idx m2_phase m2_deg
 * m2_retry m2_idx — exactly the 9 fields airis_mount.py writes to its
 * homing debug CSV. Streamed only during CAL. */
void pushHomeLog() {
  serialWrite(F("HOME_LOG "));
  serialWrite(millis());          serialWrite(' ');
  serialWrite(m1.homePhase);      serialWrite(' ');
  serialWrite(encDeg(m1), 2);     serialWrite(' ');
  serialWrite(m1.verRetry);       serialWrite(' ');
  serialWrite(m1.indexFound ? 1 : 0); serialWrite(' ');
  serialWrite(m2.homePhase);      serialWrite(' ');
  serialWrite(encDeg(m2), 2);     serialWrite(' ');
  serialWrite(m2.verRetry);       serialWrite(' ');
  serialWriteLine(m2.indexFound ? 1 : 0);
}

/* --------------------------- Serial protocol ---------------------------- */
char    lineBuf[64];
uint8_t lineLen = 0;
unsigned long lastRxByteMs = 0;  // age of the newest byte in lineBuf
                                 // (stale-partial purge, see LINE_STALE_MS)

/* May an UNSOLICITED telemetry burst start right now? RX has absolute
 * priority (rule 1): nothing waiting, no line mid-assembly, and no byte
 * received within the last RX_QUIET_MS (v5 — the host is mid-transaction;
 * our ACK/NAK/reply must not be interleaved with a POS line). The whole
 * burst must also fit the TX ring so the print can never block the loop or
 * overfill the bridge (rule 2). Reply-class output never calls this. */
bool telemetryClear() {
  bool rxQuiet = (long)(millis() - rxQuietUntil) >= 0;
  return rxQuiet && lineLen == 0 && Serial.available() == 0 &&
         Serial.availableForWrite() >= TELEM_TX_MIN_FREE;
}

void startSlewTo(float t1, float t2) {
  m1.targetDeg = t1;
  m2.targetDeg = t2;
  settlePasses  = 0;
  settleWaitEnd = 0;
  float fe = sunLegCount ? sunWpEl[0] : t1;
  float fa = sunLegCount ? sunWpAz[0] : t2;
  // Crawl-land only on the leg that actually arrives at the target; detour
  // corner waypoints stop plainly (their accuracy doesn't matter).
  // Use DANCE_SPEED/ACCEL: same crawl-landing profile that DANCE uses,
  // which is measurably smoother and more repeatable than the higher SLEW
  // accel. The settle loop below still provides closed-loop correction.
  // v5: snag watch ON for pointing legs — a stall halts within ~200 ms and
  // is reported (the bounded settle then ends in ERR 58) instead of being
  // discovered, and worked around, at settle time.
  bool land = (sunLegCount == 0);
  axisStartMoveDeg(m1, fe - encDeg(m1), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, true, land);
  axisStartMoveDeg(m2, fa - encDeg(m2), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, true, land);
  sysState = ST_MOVING;
  pushStatus(PY_SLEWING);
}

/* Validate the v5 trailer "#<seq>*<cs>" (cs = XOR of every byte before
 * '*', two hex digits), strip it from the line, return the seq. looseSeq is
 * a best-effort seq read even from a damaged line so the NAK can name it. */
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
  lastCmdMs = millis();             // any command defers the idle sleep

  if (holdTrimming) {               // station-keeping trims yield instantly
    axisStop(m1);                   // to any real command (they only ever
    axisStop(m2);                   // run in ST_IDLE, so this is safe)
    holdTrimming = false;
  }
  serialWrite(F("RX: "));          // human echo; the host ignores it
  serialWriteLine(line);

  /* v5 framing: checksum, then de-dup, then ACK — BEFORE any execution. */
  uint8_t seq; int loose;
  if (!parseSeqChecksum(line, seq, loose)) {
    serialWrite(F("NAK "));
    serialWrite(loose);
    serialWriteLine(F(" checksum"));
    return;                                        // NEVER execute a damaged line
  }
  if (seq == lastSeq && !strcmp(line, lastLine)) { // resend after a lost ACK
    serialWrite(F("ACK "));
    serialWriteLine(seq);
    return;                                        // already executed: no ghost move
  }
  lastSeq = seq;
  strncpy(lastLine, line, sizeof(lastLine) - 1);
  lastLine[sizeof(lastLine) - 1] = '\0';
  serialWrite(F("ACK "));
  serialWriteLine(seq);

  char *cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "MOVE")) {
    if (isFault()) { pushErrMsg(PY_ERR_FAULT, F("fault latched (send ACK)")); return; }
    if (sysState == ST_HOMING || sysState == ST_MOVING ||
        sysState == ST_DANCING || m1.moving || m2.moving) {
      pushErrMsg(PY_ERR_BUSY, F("busy")); return;
    }
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");
    if (!a1 || !a2) { pushErrMsg(PY_ERR_UNDERFLOW, F("usage: MOVE <el> <az>")); return; }
    float t1 = atof(a1), t2 = atof(a2);

    // Normalise az: 180 and -180 name the same direction. Pick whichever
    // equivalent value is shortest-path from the current az position so that
    // MOVE 0 -180 from az=180 is treated as "no az travel needed", not 360°.
    // If the shortest-path equivalent lands OUTSIDE the -140..+180 ROM, keep
    // the canonical direction — it is in the "outlawed" band (below -140) and
    // gets reached via the elevation flip below, not by driving az there.
    {
      float ca = encDeg(m2);
      float t2n = ca + wrap180(t2 - ca);
      if (t2n >= M2_MIN_DEG && t2n <= M2_MAX_DEG) t2 = t2n;  // in-ROM, shortest
      else                                        t2 = wrap180(t2);  // outlawed dir
    }

    // Only ELEVATION is limited in ROM here (full +/-180). Azimuth is never
    // rejected for range: anything in the outlawed -180..-140 band is made
    // reachable by the camera flip (see below), so only |az| > 180 (impossible
    // after wrap180) or a bad elevation is an error.
    if (t1 < M1_MIN_DEG || t1 > M1_MAX_DEG) {
      pushErrMsg(PY_ERR_ANGLE, F("target elevation outside soft limits"));
      return;
    }
    if (!(m1.homed && m2.homed)) {
      pushErrMsg(PY_ERR_NOT_HOMED, F("not homed (send CAL)"));
      return;
    }
    if (fabsf(encDeg(m1)) > LIMIT_DEG + LIMIT_GUARD_DEG ||
        fabsf(encDeg(m2)) > LIMIT_DEG + LIMIT_GUARD_DEG) {
      // Never start a slew from outside the verified envelope (only possible
      // after an abnormal abort) — it would trip the watchdog instantly.
      pushErrMsg(PY_ERR_NOT_HOMED, F("position outside +/-180 - send CAL first"));
      return;
    }
    if (sunBlockedSky(t1, t2, 0.0f)) {     // either representation = same sky
      pushErrMsg(PY_ERR_SUN, F("target is sun-facing (inside sun keep-out)"));
      return;
    }

    /* CAMERA-FLIP: a sky target (el >= 0) is reachable directly at (t1,t2)
     * or over the top at (180-t1, t2+180) — same boresight direction. Rank
     * the two by worst-axis travel (both axes run the same deg/s, so time
     * ~= max axis delta) and take the first that yields a sun-safe route.
     * Below-horizon targets are frame-positioning moves and never flip. */
    reqEl = t1;                            // remember the REQUESTED frame so
    reqAz = t2;                            // POS reads back in it (see printPos)
    float candE[2] = { t1, t1 }, candA[2] = { t2, t2 };
    uint8_t nCand = 1;
    if (t2 > M2_MAX_DEG || t2 < M2_MIN_DEG) {
      /* OUTLAWED azimuth (below -140, i.e. -180..-140): the azimuth ROM cannot
       * reach this direction, so it is MANDATORY (independent of flipEnable) to
       * point at the same sky direction over the top — (el,az) == (flipEl(el),
       * az+180) — which reflects the azimuth back inside the ROM. Elevation
       * keeps its full range. reqEl/reqAz stay the original request so POS reads
       * back in the commanded frame. No direct candidate exists (unreachable).*/
      float fa  = wrap180(t2 + 180.0f);            // flipped az (lands in ROM)
      float ca2 = encDeg(m2);
      candE[0]  = flipEl(t1);
      candA[0]  = ca2 + wrap180(fa - ca2);         // nearest in-ROM equivalent
      nCand     = 1;
      serialWrite(F("AZ-LIMIT: az "));
      serialWrite(t2, 1);
      serialWriteLine(F(" outside the -140..+180 ROM — flipping elevation to reach the same sky point"));
    } else if (flipEnable && t1 - frameElOff >= 0.0f) {  // "sky target" in PHYSICAL terms
      float f1 = flipEl(t1), f2 = wrap180(t2 + 180.0f);
      float cur_el = encDeg(m1), cur_az = encDeg(m2);
      float del_az_d = fabsf(t2 - cur_az), del_az_f = fabsf(f2 - cur_az);
      float cd = fmaxf(fabsf(t1 - cur_el), del_az_d);
      float cf = fmaxf(fabsf(f1 - cur_el), del_az_f);
      // Az travel in 120-160° penalised 100% (counts double): bias toward the
      // solution that avoids that mid-range sweep.
      if (del_az_d >= 120.0f && del_az_d <= 160.0f) cd += del_az_d;
      if (del_az_f >= 120.0f && del_az_f <= 160.0f) cf += del_az_f;
      nCand = 2;
      if (cf + FLIP_MARGIN_DEG < cd) {     // flip clearly cheaper: try first
        candE[0] = f1; candA[0] = f2;
        candE[1] = t1; candA[1] = t2;
      } else {                             // direct first, flip as fallback
        candE[1] = f1; candA[1] = f2;
      }
    }
    float u1 = t1, u2 = t2;
    uint8_t nw = 255;
    for (uint8_t i = 0; i < nCand && nw == 255; i++) {
      nw = planSunRoute(encDeg(m1), encDeg(m2), candE[i], candA[i]);
      if (nw != 255) { u1 = candE[i]; u2 = candA[i]; }
    }
    if (nw == 255) {
      pushErrMsg(PY_ERR_SUN, F("no sun-safe route to target"));
      return;
    }
    if (u1 != t1 || u2 != t2) {
      serialWrite(F("FLIP: pointing over the top instead — el "));
      serialWrite(u1, 1);
      serialWrite(F(" az "));
      serialWrite(u2, 1);
      serialWriteLine(F(" (same sky direction, shorter move)"));
    }
    sunLegCount = nw;
    sunLegIdx   = 0;
    if (nw) {
      serialWrite(F("SUN ROUTE: "));
      serialWrite(nw);
      serialWriteLine(F(" waypoint(s) — EL held clear of the sun window until AZ passes"));
    }
    startSlewTo(u1, u2);

  } else if (!strcasecmp(cmd, "STOP")) {
    bool wasHoming = (sysState == ST_HOMING);
    axisStopRamp(m1);
    axisStopRamp(m2);
    calStage  = C_NONE;
    verStage  = V_IDLE;
    sunLegIdx = sunLegCount = 0;
    recalSweepPending = false;             // cancel a pending educated re-cal
    if (wasHoming) {
      // CAL/verify aborted mid-sequence: axes may legitimately be far
      // outside +/-180 and the range is unverified. Force a fresh CAL.
      m1.homed = m2.homed = false;
      serialWriteLine(F("STOP: homing/verify aborted — re-CAL required"));
    }
    if (!isFault()) sysState = (m1.homed && m2.homed) ? ST_IDLE : ST_NOT_HOMED;
    if (m1.moving || m2.moving) {
      stopPending = true;
      serialWriteLine(F("STOPPING"));
    } else {
      m1.targetDeg = encDeg(m1);
      m2.targetDeg = encDeg(m2);
      reqEl = m1.targetDeg;
      reqAz = m2.targetDeg;
      serialWriteLine(F("STOPPED"));
      motorsDisable();                     // STOP releases drivers immediately
      pushStatus(pyStatusCode());          // host waits for STATUS 0
    }

  } else if (!strcasecmp(cmd, "CAL") || !strcasecmp(cmd, "HOME")) {
    if (isFault()) { pushErrMsg(PY_ERR_FAULT, F("fault latched (send ACK)")); return; }
    if (sysState == ST_MOVING || sysState == ST_HOMING ||
        sysState == ST_DANCING || m1.moving || m2.moving) {
      pushErrMsg(PY_ERR_BUSY, F("busy")); return;
    }
    // The sun-safe park azimuth is computed LATER, at the C_AZ_PARK
    // transition — in the index frame AZ has just been zeroed into, from
    // the firmware's own sun center. A numeric argument is still ACCEPTED
    // for protocol compatibility — airis_mount's calibrate_motors() sends
    // "CAL 0.0" by default — but IGNORED: honoring it would park AZ at 0,
    // wherever the sun happens to be.
    if (strtok(NULL, " \t")) {
      serialWriteLine(F("CAL: host az argument ignored — sun avoidance is firmware-owned"));
    }
    calSkipVerify = false;   // an explicit CAL always runs the +/-180 validation
    calBeginFull();

  } else if (!strcasecmp(cmd, "SETZERO")) {
    // Anchor the PHYSICAL-HOME frame — by ARITHMETIC, not by position.
    // The host calls this after the post-CAL offset move. The mount only
    // ever reaches that offset target within the settle tolerance, so
    // "zero = wherever I stopped" would shift home by up to +/-1.5 deg on
    // EVERY calibration. Instead, zero goes exactly where the INDEX says
    // it belongs: current coordinates are index-frame, so shifting the
    // counts by HOME_*_OFFSET puts encoder (0,0) at index + offset — the
    // SAME physical point every run, regardless of parking error. The
    // mount's true residual then reads back honestly on POS.
    if (m1.moving || m2.moving) {
      pushErrMsg(PY_ERR_BUSY, F("busy")); return;
    }
    noInterrupts();
    m1.enc -= lround((double)HOME_EL_OFFSET_DEG / 360.0 * (double)ENC_CPR);
    m2.enc -= lround((double)HOME_AZ_OFFSET_DEG / 360.0 * (double)ENC_CPR);
    interrupts();
    frameElOff = 0.0f;                 // encoder frame == physical frame now
    frameAzOff = 0.0f;
    m1.targetDeg = encDeg(m1);         // hold wherever we physically are
    m2.targetDeg = encDeg(m2);         // (the small residual, near 0,0)
    reqEl = m1.targetDeg;
    reqAz = m2.targetDeg;
    serialWriteLine(F("ZERO SET"));
    pushStatus(PY_IDLE);

  } else if (!strcasecmp(cmd, "ACK")) {
    if (isFault()) {
      sysState    = ST_NOT_HOMED;     // faults invalidate home; re-CAL required
      faultCode   = 0;
      stopPending = false;
      serialWriteLine(F("OK"));
      pushStatus(PY_IDLE);
    } else {
      serialWriteLine(F("OK NO FAULT"));
    }

  } else if (!strcasecmp(cmd, "STATUS")) {
    pushStatus(pyStatusCode());

  } else if (!strcasecmp(cmd, "POS")) {
    printPos();

  } else if (!strcasecmp(cmd, "POSRAW")) {
    // Honest mechanical position: raw encoder, no flip/frame remap (printPos
    // remaps toward the last request and can MASK a flipped/drifted pose).
    // Use this, not POS, when verifying that a command landed where expected.
    serialWrite(F("POSRAW "));
    serialWrite(encDeg(m1), 2);
    serialWrite(' ');
    serialWriteLine(encDeg(m2), 2);

  } else if (!strcasecmp(cmd, "FLIP")) {
    strtok(NULL, " \t");
    serialWriteLine(F("FLIP OFF (disabled)"));

  } else if (!strcasecmp(cmd, "DANCE")) {
    if (isFault()) { pushErrMsg(PY_ERR_FAULT, F("fault latched (send ACK)")); return; }
    if (sysState == ST_MOVING || sysState == ST_HOMING ||
        sysState == ST_DANCING || m1.moving || m2.moving) {
      pushErrMsg(PY_ERR_BUSY, F("busy")); return;
    }
    if (!(m1.homed && m2.homed)) { pushErrMsg(PY_ERR_NOT_HOMED, F("not homed (send CAL)")); return; }
    danceIdx = 0;
    danceDwellStart = 0;
    sysState = ST_DANCING;
    serialWriteLine(F("OK LET'S DANCE"));
    pushStatus(PY_SLEWING);

  } else if (!strcasecmp(cmd, "MOTOROFF")) {
    // Host-requested driver release (sent after its own idle timeout).
    // Must never produce ERR 60 — that desyncs the host's serial parsing.
    if (m1.moving || m2.moving) { pushErrMsg(PY_ERR_BUSY, F("busy")); return; }
    motorsDisable();                   // prints "MOTORS OFF" (silent if off)

  } else if (!strcasecmp(cmd, "RAW")) {
    // Open-loop bench run — see rawRun(). The only guard is "busy": never
    // clobber an active profile or state machine. Everything else (fault
    // latch, homed, limits, sun) is deliberately bypassed.
    if (sysState == ST_HOMING || sysState == ST_MOVING ||
        sysState == ST_DANCING || m1.moving || m2.moving) {
      pushErrMsg(PY_ERR_BUSY, F("busy")); return;
    }
    char *a1 = strtok(NULL, " \t");   // motor: 1 | 2 | 12/both
    char *a2 = strtok(NULL, " \t");   // direction: cw | c
    char *a3 = strtok(NULL, " \t");   // speed, deg/s
    char *a4 = strtok(NULL, " \t");   // time, s
    if (!a1 || !a2 || !a3 || !a4) {
      pushErrMsg(PY_ERR_UNDERFLOW, F("usage: RAW <1|2|both> <cw|c> <deg_per_s> <seconds>"));
      return;
    }
    uint8_t mask;
    if      (!strcmp(a1, "1"))  mask = 1;
    else if (!strcmp(a1, "2"))  mask = 2;
    else if (!strcmp(a1, "12") || !strcasecmp(a1, "BOTH")) mask = 3;
    else { pushErrMsg(PY_ERR_CMD, F("RAW motor must be 1, 2 or both")); return; }
    bool cw;
    if      (!strcasecmp(a2, "CW")) cw = true;
    else if (!strcasecmp(a2, "C"))  cw = false;
    else { pushErrMsg(PY_ERR_CMD, F("RAW direction must be cw or c")); return; }
    float dps  = atof(a3);
    float secs = atof(a4);
    if (dps <= 0.0f || secs <= 0.0f) {
      pushErrMsg(PY_ERR_ANGLE, F("RAW speed and time must be > 0"));
      return;
    }
    // Caps: the run BLOCKS the whole loop (no STOP, no heartbeat), so keep
    // it short; the speed cap keeps the step interval well above the
    // blocking generator's worst-case pass time.
    if (secs > RAW_MAX_S)   { secs = RAW_MAX_S;   serialWriteLine(F("RAW: time capped at 30 s")); }
    if (dps  > RAW_MAX_DPS) { dps  = RAW_MAX_DPS; serialWriteLine(F("RAW: speed capped at 360 dps")); }
    serialWrite(F("RAW RUN: M"));
    if (mask & 1) serialWrite('1');
    if (mask & 2) serialWrite('2');
    serialWrite(' ');
    serialWrite(cw ? F("cw") : F("c"));
    serialWrite(F(" at "));
    serialWrite(dps, 1);
    serialWrite(F(" dps for "));
    serialWrite(secs, 1);
    serialWriteLine(F(" s — open loop, all checks bypassed, blocking (STOP won't interrupt)"));
    rawRun(mask, cw, dps, secs);

  } else if (!strcasecmp(cmd, "SUN")) {
    // "SUN <el> <az>": update the keep-out center (CURRENT encoder frame).
    // The host pushes this every 60 s (GPS + UTC + heading). Bare "SUN":
    // TEST ONLY — slew DIRECTLY at the sun center, bypassing the keep-out.
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");
    if (a1 && a2) {
      // v5 LIVE SUN CENTER: values arrive in the CURRENT encoder frame and
      // are stored in the PHYSICAL frame, so they survive SETZERO and every
      // downstream user (sunElF/sunAzF, the box, the detour planner, the
      // flip check) already reads them back through the frame offsets.
      float el = atof(a1), az = atof(a2);
      sunEl = el - frameElOff;
      sunAz = wrap180(az - frameAzOff);
      serialWrite(F("SUN OK "));
      serialWrite(el, 3);
      serialWrite(' ');
      serialWriteLine(az, 3);
      return;
    }
    if (isFault()) { pushErrMsg(PY_ERR_FAULT, F("fault latched (send ACK)")); return; }
    if (sysState == ST_HOMING || sysState == ST_MOVING ||
        sysState == ST_DANCING || m1.moving || m2.moving) {
      pushErrMsg(PY_ERR_BUSY, F("busy")); return;
    }
    if (!(m1.homed && m2.homed)) {
      pushErrMsg(PY_ERR_NOT_HOMED, F("not homed (send CAL)")); return;
    }
    serialWriteLine(F("SUN POINT (TEST): slewing directly at the sun center — keep-out bypassed"));
    sunLegIdx = sunLegCount = 0;           // direct — no detour legs
    reqEl = sunElF();                      // sun in the CURRENT frame
    reqAz = sunAzF();
    startSlewTo(reqEl, reqAz);

  } else if (!strcasecmp(cmd, "ID")) {
    serialWriteLine(F(FW_ID));

  } else if (!strcasecmp(cmd, "HELP")) {
    printHelp();

  } else {
    pushErrMsg(PY_ERR_CMD, F("unknown command (HELP for list)"));
  }
}

void pollSerial() {
  /* Stale-partial purge: a fragment sitting in lineBuf past LINE_STALE_MS
   * lost its terminator on the bus (protocol lines arrive whole in <5 ms).
   * Left alone it prepends itself to the NEXT command — the persistent
   * 'MOVE'->'MO' truncation. Discard it and say so, so a resend lands on a
   * clean buffer instead of inheriting the desync. (With the v5 checksum a
   * poisoned line would be NAK'd anyway; this just heals faster.) */
  if (lineLen > 0 && millis() - lastRxByteMs > LINE_STALE_MS) {
    lineBuf[lineLen] = '\0';
    serialWrite(F("RXHEAL: discarded stale partial line \""));
    serialWrite(lineBuf);
    serialWriteLine(F("\" (terminator lost on the bus)"));
    lineLen = 0;
  }
  while (Serial.available()) {
    rxQuietUntil = millis() + RX_QUIET_MS;         // host is talking: hold our telemetry
    uint8_t c = (uint8_t)Serial.read();
    lastRxByteMs = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) handleLine(lineBuf);            // checksum/seq judged inside
      lineLen = 0;
      continue;
    }
    // USB-wake garbage filter: non-printable bytes are junk (a CDC link
    // waking from autosuspend garbles the first bytes of a line). Kept in
    // v5 — it is what rescues a line after a wake; the checksum then
    // decides whether what survived is trustworthy.
    if (c < 0x20 || c > 0x7E) continue;
    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = (char)c;
    } else {
      lineLen = 0;
      pushErrMsg(PY_ERR_LINE_LONG, F("line too long"));
    }
  }
}

/* ------------------------------- Setup ---------------------------------- */
void axisInit(Axis &a, const char *name, uint8_t pulse, uint8_t dir,
              uint8_t encA, uint8_t encB, uint8_t encZ,
              bool invert, bool invertEnc, unsigned long stepsPerRev) {
  a.name = name;
  a.pulsePin = pulse; a.dirPin = dir; a.invertDir = invert;
  a.encSign = invertEnc ? -1 : +1;
  a.stepsPerRev = stepsPerRev;
  a.enc = 0; a.indexArmed = false; a.indexFound = false; a.encAtIndex = 0;
  a.idxRezeroArmed = false; a.indexRefEnc = 0; a.encPrevSvc = 0;
  a.moving = false; a.dir = 1; a.totalSteps = 0; a.stepsDone = 0;
  a.decelOnly = false;
  a.pulseHigh = false; a.pulseHighUs = 0;
  a.snagWatch = false; a.snagged = false; a.moveStartEnc = 0; a.lastSnagChkMs = 0;
  a.punchProgressRetries = 0; a.punchStartDeg = 0.0f;
  a.legPhase = LEG_IDLE; a.legTargetDeg = 0; a.legDps = 0; a.legAccel = 0;
  a.punchesUsed = 0;
  a.homePhase = H_NONE; a.homed = false; a.zeroKnown = false;
  a.idxRoughEnc = 0; a.verRetry = 0;
  a.searchRetry = 0; a.homeRestUntil = 0;
  a.searchResumePhase = H_NONE; a.searchResumeTarget = 0.0f;
  a.targetDeg = 0.0f;

  pinMode(pulse, OUTPUT); digitalWrite(pulse, LOW);
  pinMode(dir,   OUTPUT); digitalWrite(dir,   LOW);
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encZ, INPUT_PULLUP);
  a.qPrev = (digitalRead(encA) << 1) | digitalRead(encB);
}

void setup() {
  Serial.begin(115200);

  // Drivers start de-energized; any motion command enables them on demand.
  pinMode(M1_ENA_PIN, OUTPUT); digitalWrite(M1_ENA_PIN, ENA_DISABLE_LEVEL);
  pinMode(M2_ENA_PIN, OUTPUT); digitalWrite(M2_ENA_PIN, ENA_DISABLE_LEVEL);
  motorsEnabled = false;

  axisInit(m1, "M1 (EL)", M1_PULSE_PIN, M1_DIR_PIN, M1_ENC_A_PIN, M1_ENC_B_PIN,
           M1_ENC_INDEX_PIN, M1_INVERT_DIR, M1_INVERT_ENC, M1_STEPS_PER_REV);
  axisInit(m2, "M2 (AZ)", M2_PULSE_PIN, M2_DIR_PIN, M2_ENC_A_PIN, M2_ENC_B_PIN,
           M2_ENC_INDEX_PIN, M2_INVERT_DIR, M2_INVERT_ENC, M2_STEPS_PER_REV);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN),     isrM1Quad,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B_PIN),     isrM1Quad,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_INDEX_PIN), isrM1Index, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A_PIN),     isrM2Quad,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B_PIN),     isrM2Quad,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_INDEX_PIN), isrM2Index, RISING);

  serialWriteLine(F(FW_ID));
  printHelp();
  serialWriteLine(F("MOTORS OFF"));
}

/* -------------------------------- Loop ---------------------------------- */
void loop() {
  pollSerial();

  /* Step generation first, every pass — everything else is rate-limited so
   * the step timing stays clean. */
  axisService(m1);
  axisService(m2);
  snagService(m1);
  snagService(m2);

  /* 1 kHz encoder service: samples both encoders and applies the continuous
   * index re-zero (drift control). Cheap; gated internally on ENC_SERVICE_US. */
  encService();

  /* COMMAND-LOSS KILL TIMER: motion in progress but the host has been silent
   * for KILL_TIMER_MS -> presume the host is dead, ramp everything to a stop
   * and let the idle sleep below power the drivers down. Same semantics as a
   * host STOP (an aborted CAL would demand a re-CAL, but CAL is exempt: it
   * is autonomous by design and covered by HOMING_SLEEP_MS instead). */
  if (sysState != ST_HOMING && !stopPending &&
      (m1.moving || m2.moving) &&
      millis() - lastCmdMs >= KILL_TIMER_MS) {
    serialWriteLine(F("KILL TIMER: no commands for 60 s — stopping motion, powering down"));
    axisStopRamp(m1);
    axisStopRamp(m2);
    sunLegIdx = sunLegCount = 0;
    if (!isFault()) sysState = (m1.homed && m2.homed) ? ST_IDLE : ST_NOT_HOMED;
    stopPending = true;                    // prints STOPPED + STATUS when done;
  }                                        // idle sleep then releases drivers

  /* IDLE SLEEP: de-energize the drivers if no serial command has arrived
   * for IDLE_SLEEP_MS and nothing is in motion — EXCEPT while homed and
   * on target (ST_IDLE): then the drivers stay energized, ACTIVELY HOLDING
   * the pointing (the host's MOTOROFF is still honored). While homing the
   * much longer HOMING_SLEEP_MS applies (CAL runs minutes with no host
   * commands). The encoders keep counting while asleep, so any parked
   * drift is seen and corrected on the next move. */
  if (motorsEnabled && !m1.moving && !m2.moving && !stopPending &&
      sysState != ST_MOVING && sysState != ST_DANCING &&
      millis() - lastCmdMs >=
        ((sysState == ST_HOMING) ? HOMING_SLEEP_MS :
         (sysState == ST_IDLE)   ? HOLD_SILENCE_MS   // active hold: only the
                                 : IDLE_SLEEP_MS)) { // dead-link failsafe
    serialWriteLine(sysState == ST_IDLE
        ? F("HOLD TIMEOUT: no commands for 30 min - link/host presumed dead, drivers released")
        : F("IDLE SLEEP: no commands, drivers released"));
    motorsDisable();
  }

  /* ACTIVE HOLD: while homed, idle, and energized, watch the encoders and
   * quietly re-trim any drift beyond HOLD_TRIM_DEG (wind loading, bumps,
   * balance sag). Yields instantly to any incoming command (handleLine
   * cancels a hold trim before processing). Never runs with the drivers
   * released — a host MOTOROFF means "let it rest". */
  if (sysState == ST_IDLE && motorsEnabled && !stopPending &&
      m1.homed && m2.homed && !m1.moving && !m2.moving &&
      millis() - lastHoldChkMs >= HOLD_CHECK_MS) {
    lastHoldChkMs = millis();
    float h1 = m1.targetDeg - encDeg(m1);
    float h2 = m2.targetDeg - encDeg(m2);
    if (fabsf(h1) > HOLD_TRIM_DEG || fabsf(h2) > HOLD_TRIM_DEG) {
      holdTrimming = true;
      serialWriteLine(F("HOLD: drift detected — re-trimming to target"));
      if (fabsf(h1) > SETTLE_TOL_DEG)
        axisStartMoveDeg(m1, h1, TRIM_SPEED_DPS, TRIM_ACCEL_DPS2, false, false);
      if (fabsf(h2) > SETTLE_TOL_DEG)
        axisStartMoveDeg(m2, h2, TRIM_SPEED_DPS, TRIM_ACCEL_DPS2, false, false);
    }
  }
  if (holdTrimming && !m1.moving && !m2.moving) holdTrimming = false;

  if (stopPending && !m1.moving && !m2.moving) {
    stopPending = false;
    m1.targetDeg = encDeg(m1);
    m2.targetDeg = encDeg(m2);
    reqEl = m1.targetDeg;
    reqAz = m2.targetDeg;
    serialWriteLine(F("STOPPED"));
    printPos();
    motorsDisable();                       // STOP releases drivers after ramp-down
    pushStatus(pyStatusCode());            // host waits for STATUS 0
  }

  /* IDLE HEARTBEAT: keepalive POS while stationary (see IDLE_HEARTBEAT_MS).
   * Skipped while homing/moving — those states already stream (HOME_LOG /
   * POS) so the bus is never silent there anyway. telemetryClear() also
   * enforces the v5 RX_QUIET_MS hold after any received byte. */
  if (sysState != ST_HOMING && sysState != ST_MOVING &&
      sysState != ST_DANCING && !m1.moving && !m2.moving &&
      millis() - lastHeartbeatMs >= IDLE_HEARTBEAT_MS &&
      telemetryClear()) {
    lastHeartbeatMs = millis();
    printPos();
    printTgt();
  }

  if (sysState == ST_HOMING) {
    calService();
    if (sysState == ST_HOMING &&
        millis() - lastPosPushMs >= POS_STREAM_MS &&
        telemetryClear()) {
      lastPosPushMs = millis();
      printTgt();                          // full-rate raw positions during
                                           // CAL too: the host CSV keeps a
                                           // continuous position/velocity
                                           // data path through homing
    }
    if (sysState == ST_HOMING &&           // not faulted / finished by service
        millis() - lastHomeLogMs >= HOME_LOG_MS &&
        telemetryClear()) {
      lastHomeLogMs = millis();
      pushHomeLog();
    }

  } else if (sysState == ST_MOVING || sysState == ST_DANCING) {
    if (millis() - lastPosPushMs >= POS_STREAM_MS &&
        telemetryClear()) {
      lastPosPushMs = millis();
      printPos();                          // host motion logger samples these
      printTgt();                          // current aim point for the CSV
    }

    /* Hard-limit watchdog on the REAL encoder position. Only here: homing
     * legitimately exceeds +/-180 (365 sweeps, recovery unwinds). */
    bool m1over = fabsf(encDeg(m1)) > LIMIT_DEG + LIMIT_GUARD_DEG;
    bool m2over = fabsf(encDeg(m2)) > LIMIT_DEG + LIMIT_GUARD_DEG;
    if (m1over || m2over) {
      uint8_t lc = m1over ? PY_ERR_LIMIT_M1 : PY_ERR_LIMIT_M2;
      latchFault(ST_FAULT_LIMIT, lc);
      serialWrite(F("ERR "));
      serialWrite(lc);
      serialWriteLine(F(" LIMIT TRIP: +/-180 deg exceeded, motion halted"));

    } else if (!m1.moving && !m2.moving) {
      if (sysState == ST_MOVING && sunLegIdx < sunLegCount) {
        // Arrived at a sun-detour waypoint: fire the next leg immediately.
        // No settle pass between legs — accuracy only matters at the final
        // target; corners just need to stay outside the sun window.
        sunLegIdx++;
        bool fin = (sunLegIdx >= sunLegCount);   // final leg -> real target:
        float ne = fin ? m1.targetDeg : sunWpEl[sunLegIdx];
        float na = fin ? m2.targetDeg : sunWpAz[sunLegIdx];
        axisStartMoveDeg(m1, ne - encDeg(m1), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, false, fin);
        axisStartMoveDeg(m2, na - encDeg(m2), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, false, fin);

      } else if (sysState == ST_MOVING) {
        /* Settle (v5, BOUNDED): wait SETTLE_WAIT_MS for residual frame flex
         * to damp, read the encoder, and trim closed-loop until inside
         * SETTLE_TOL_DEG — at most SETTLE_MAX_PASSES times. Success reports
         * "DONE <el_err> <az_err>"; running out of passes reports ERR 58 with
         * the same residuals and goes IDLE honestly — the host decides. No
         * AZ nudge to free a "stuck" EL any more: a stall is reported, not
         * worked around (the snag watch on the slew halts it within ~200 ms). */
        if (settleWaitEnd == 0) {
          settleWaitEnd = millis() + SETTLE_WAIT_MS;
          if (settleWaitEnd == 0) settleWaitEnd = 1;
        } else if ((long)(millis() - settleWaitEnd) >= 0) {
          float e1 = m1.targetDeg - encDeg(m1);
          float e2 = m2.targetDeg - encDeg(m2);
          bool ok = fabsf(e1) <= SETTLE_TOL_DEG && fabsf(e2) <= SETTLE_TOL_DEG;
          if (ok && recalSweepPending) {
            // Phase 1 (sun-safe slew to 0,0) finished — go straight into the
            // index sweep. No DONE/STATUS 0 here: the whole educated re-cal
            // reads to the host as one homing cycle (STATUS 12 ... 13 0).
            settleWaitEnd = 0;
            recalSweepPending = false;
            beginRecalSweep();
          } else if (ok) {
            settleWaitEnd = 0;
            sysState = ST_IDLE;
            serialWrite(F("DONE "));
            serialWrite(e1, 3);
            serialWrite(' ');
            serialWriteLine(e2, 3);
            printPos();
            pushStatus(PY_IDLE);   // STATUS 0 IDLE — the host's move
                                   // confirmation waits on this
            // Educated re-cal cadence: the move is fully reported (host has
            // its "ok") BEFORE this fires, so the autonomous re-cal shows up
            // to the host as a normal homing cycle (STATUS 12 -> 13/0), which
            // the bridge already holds moves behind.
            if (m1.homed && m2.homed && ++movesSinceRecal >= MOVES_PER_RECAL) {
              movesSinceRecal = 0;
              beginEducatedRecal();
            }
          } else if (++settlePasses > SETTLE_MAX_PASSES) {
            settleWaitEnd = 0;
            sysState = ST_IDLE;              // give up honestly; the host decides
            recalSweepPending = false;       // a failed re-cal approach is abandoned
            serialWrite(F("ERR "));
            serialWrite(PY_ERR_SETTLE);
            serialWrite(F(" settle "));
            serialWrite(e1, 3);
            serialWrite(' ');
            serialWriteLine(e2, 3);
            pushStatus(PY_IDLE);
          } else {
            settleWaitEnd = 0;               // one more trim, SAME targets, no other axis
            if (fabsf(e1) > SETTLE_TOL_DEG)
              axisStartMoveDeg(m1, e1, TRIM_SPEED_DPS, TRIM_ACCEL_DPS2, false, false);
            if (fabsf(e2) > SETTLE_TOL_DEG)
              axisStartMoveDeg(m2, e2, TRIM_SPEED_DPS, TRIM_ACCEL_DPS2, false, false);
          }
        }

      } else {                             // ST_DANCING: next waypoint
        if (danceDwellStart == 0) {
          danceDwellStart = millis();      // brief rest between figures
        } else if (millis() - danceDwellStart >= DANCE_DWELL_MS) {
          danceDwellStart = 0;
          if (danceIdx < DANCE_LEN) {
            m1.targetDeg = DANCE_SEQ[danceIdx][0];
            m2.targetDeg = DANCE_SEQ[danceIdx][1];
            reqEl = m1.targetDeg;
            reqAz = m2.targetDeg;
            axisStartMoveDeg(m1, m1.targetDeg - encDeg(m1),
                             DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, false, true);
            axisStartMoveDeg(m2, m2.targetDeg - encDeg(m2),
                             DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, false, true);
            danceIdx++;
          } else {
            sysState = ST_IDLE;
            serialWriteLine(F("DANCE COMPLETE *bows*"));
            printPos();
            pushStatus(PY_IDLE);
          }
        }
      }
    }
  }
}

/* ------------------------------- End of file --------------------------- */