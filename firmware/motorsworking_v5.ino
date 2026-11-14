/* AIRIS Mount Controller — Arduino Mega 2560, two-axis stepper mount */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#define FW_ID "MEGA2560-2AXIS-STEP v5.1 (AIRIS: seq+CRC16 ACK/NAK, physical-frame SUN, boot ERR 67, auto-recal off, detour arrival check, dedup, distinct err codes, bounded settle, DONE residuals, live SUN, rx-quiet telemetry, STOP releases motors, 30-min hold, az-norm, az-120-160-bias, az-ROM-140-to-180, az-320-homing, az-verify-removed, outlaw-az-flip)"

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
#define PY_ERR_RESET      67   // "RESET: ..." pushed once from setup() after every boot
#define SETTLE_MAX_PASSES 3    // closed-loop trim passes before ERR 58
#define RX_QUIET_MS       50   // pause POS/TGT/HOME_LOG this long after any received byte

#define BENCH_COMMANDS    1    // RAW and bare SUN enabled. SET TO 0 FOR THE FLIGHT BUILD.

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
#define M1_INVERT_ENC   false    // flip if the encoder counts opposite to the motor
#define M2_INVERT_ENC   false

#define PULSE_WIDTH_US  4        // DM556T needs >= 2.5 us
#define DIR_SETUP_US    10       // dir-to-pulse setup time

#define SLEW_SPEED_DPS    67.5f  // half the proven 135 dps (MOVE itself runs at DANCE_SPEED_DPS)
#define SLEW_ACCEL_DPS2  112.5f
#define DECEL_RATIO      0.75f   // decel peak = this fraction of accel peak.

#define LAND_ZONE_DEG     3.0f   // "super slow" final approach zone
#define LAND_SPEED_DPS    4.0f   // crawl speed through the zone
#define LAND_ACCEL_DPS2  15.0f   // gentleness of the very last ramp to zero

#define HOME_SPEED_DPS   200.0f   // homing/verify sweeps and CAL parks.
#define VERIFY_SPEED_DPS 90.0f
#define HOME_ACCEL_DPS2  200.0f  // homing peak accel.

#define IDX_OVERSHOOT_DEG 10.0f  // continue this far past the index pulse...
#define IDX_RESWEEP_DEG   12.0f
#define QCAL_WIN_DEG       5.0f  // quick re-cal (CALQ): half-width of the index search window around the host's hint.
#define IDX_CREEP_DPS      2.0f  // re-capture creep rate: slow enough that
                                 // ISR latency (~us) is < 0.01 deg of error

#define ENC_SERVICE_US    1000UL // 1 ms (1 kHz) encoder service period
#define INDEX_REZERO_TOL_DEG 3.0f
#define MOVES_PER_RECAL      0   // v5.1: DISABLED (0 = never).

#define SNAG_CHECK_MS    25      // rate-sample cadence while a watched leg runs
#define SNAG_STALL_DPS    3.0f   // encoder below this = essentially stopped
#define SNAG_CMD_MIN_DPS 10.0f   // only judge while commanding at least this
                                 // (creep moves are never judged — too slow)
#define SNAG_STALL_CHECKS 8      // 8 x 25 ms = 200 ms of stall to trip
#define VERIFY_END_TOL_DEG 15.0f // parks & verify sweeps: leg completed but
                                 // axis this far from target = treat as snag
#define SNAG_REST_MS   400UL
#define SNAG_BACKOFF_DEG 10.0f   // back off this far before the punch (user spec)
#define SNAG_PUNCH_DPS   90.0f
#define SNAG_SLOW_DPS    40.0f
#define M1_SNAG_SLOW_TRIES 0     // EL: no slow retry — if it snags it snags
#define M2_SNAG_SLOW_TRIES 1     // AZ: one slow attempt after the punch
#define PUNCH_PROGRESS_DEG 1.0f
#define PUNCH_PROGRESS_RETRIES_MAX 2
#define M1_SNAG_PUNCH_TRIES 0    // EL: no punch-through — if it snags it snags
#define M2_SNAG_PUNCH_TRIES 1    // AZ: one backoff + punch attempt

/* ---- Homing sweep geometry ---- */
#define HOME_SWEEP_FWD_DEG   180.0f // first search leg (user spec)
#define M1_HOME_SWEEP_SIGN   1.0f   // EL: first search leg runs positive
#define M2_HOME_SWEEP_SIGN  -1.0f   // AZ: first search leg runs negative
#define M1_HOME_SWEEP_TOTAL_DEG 365.0f // EL: full-rev sweep (>=360 + margin)
#define M2_HOME_SWEEP_TOTAL_DEG 320.0f // AZ: limited to the -140..+180 ROM
#define M1_UNWIND_DEG           365.0f // EL verify-failure recovery rotation
#define M2_UNWIND_DEG           320.0f // AZ: capped to the 320 deg ROM
#define VERIFY_MAX_RETRY     1      // unwind/re-home attempts per axis per CAL
#define HOME_SEARCH_RETRY    1

#define EL_BLIND_PARK_DEG  -270.0f
#define EL_BLIND_PARK_RETRY_DEG 90.0f
#define EL_AZVAL_PARK_DEG 180.0f // CAL step 6: EL held here during the AZ validation sweep.
#define CAL_SUN_OFF_DEG   90.0f

/* ---- Soft/hard limits (all measured from encoder zero = physical home
 *      after SETZERO) ---- */
#define M1_MIN_DEG     -180.0f
#define M1_MAX_DEG      180.0f
#define M2_MIN_DEG     -140.0f    // AZ range is -140..+180; below -140 is reached by the elevation flip
#define M2_MAX_DEG      180.0f
#define LIMIT_DEG       180.0f   // hard encoder watchdog, +/- from zero
#define LIMIT_GUARD_DEG  10.0f   // watchdog margin (tolerates brief overshoot on full-range pivots).

/* ---- Settle / trim (post-slew accuracy) ---- */
#define SETTLE_TOL_DEG   1.5f    // final pointing tolerance
#define SETTLE_WAIT_MS   200UL

#define HOLD_TRIM_DEG    2.0f    // re-trim when drift while holding exceeds this
#define HOLD_CHECK_MS    500UL   // drift check cadence while holding
#define TRIM_SPEED_DPS   13.5f   // slow closed-loop trim rate (proven EL value)
#define TRIM_ACCEL_DPS2  67.5f

/* ---- Dance ---- */
#define DANCE_SPEED_DPS  50.0f
#define DANCE_ACCEL_DPS2 45.0f
#define DANCE_DWELL_MS   250UL

#define FLIP_MARGIN_DEG  10.0f   // flip only if it saves more than this
                                 // (hysteresis: prefer the direct solution)

/* ---- Sun keep-out ---- */
#define SUN_CENTER_EL_DEG 85.0f // boot default sun center, physical-home frame;
#define SUN_CENTER_AZ_DEG -68.0f // the host overwrites it with SUN <el> <az>

#define HOME_EL_OFFSET_DEG -55.0f
#define HOME_AZ_OFFSET_DEG -35.0f
#define SUN_WIN_EL_DEG  25.0f    // full elevation span of the sun window
#define SUN_WIN_AZ_DEG  35.0f    // full azimuth span of the sun window
#define CAM_FOV_DEG     15.0f    // camera full FOV (inflates the box)
#define SUN_CLEAR_DEG    2.0f    // extra clearance for detour waypoints
#define SUN_WP_TOL_DEG   3.0f    // a detour leg must end this close to its waypoint (bench-tune)

/* ---- Housekeeping ---- */
#define KILL_TIMER_MS    60000UL
#define IDLE_SLEEP_MS    20000UL // de-energize drivers after this long with no
                                 // serial commands while idle (stationary)
#define HOLD_SILENCE_MS 1800000UL
#define HOMING_SLEEP_MS 120000UL // same, but while a CAL runs (CAL has long
                                 // command-free stretches by design)
#define POS_STREAM_MS      200UL
#define IDLE_HEARTBEAT_MS 1000UL // keepalive POS while stationary.
#define HOME_LOG_MS        200UL // HOME_LOG push cadence during CAL (host
                                 // captures these rows to a debug CSV)
#define TELEM_TX_MIN_FREE   60
#define LINE_STALE_MS      250UL // discard a PARTIAL command line older than this.
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
float   calSafeAz = 36.0f;
float   calElStartDeg  = 0.0f;   // EL rest position when CAL began (anchor
                                 // for the mirrored +315 blind-park retry)
bool    calElParkRetry = false;  // blind park already retried mirrored?
float   calAzParkAlt   = 0.0f;   // the OTHER 90-deg-off-sun azimuth (fallback
                                 // if the park toward the nearer one snags)
bool    calAzParkRetry = false;  // AZ park already retried mirrored?
bool    qcalActive     = false;
float   qcalElDelta    = 0.0f;   // EL index hint (deg from position at CALQ
                                 // receipt; EL hasn't moved when it's used)
bool    calSkipVerify  = false;
uint8_t movesSinceRecal = 0;     // completed moves since the last (educated or
                                 // full) re-cal; triggers one at MOVES_PER_RECAL
bool    recalSweepPending = false; // educated re-cal phase 1 (slew to 0,0) is
                                 // running; its completion starts the sweep
unsigned long lastEncSvcUs = 0;  // micros() of the last 1 kHz encoder service

enum : uint8_t { H_NONE = 0, H_PREZERO, H_FWD, H_REV,
                 H_OVER, H_CREEP, H_RETURN, H_DONE, H_EXT,
                 H_REST, H_RETRY_BACK };

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

float sunEl = SUN_CENTER_EL_DEG;
float sunAz = SUN_CENTER_AZ_DEG;

float frameElOff = HOME_EL_OFFSET_DEG;   // boot guess: index-like frame
float frameAzOff = HOME_AZ_OFFSET_DEG;
uint8_t sunLegIdx = 0, sunLegCount = 0;  // pending detour waypoints for MOVE
float sunWpEl[4], sunWpAz[4];

float reqEl = 0.0f, reqAz = 0.0f;

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
static unsigned long rxQuietUntil = 0; // hold unsolicited telemetry until this time

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
  uint8_t legPunchTries;

  /* homing / verification */
  uint8_t homePhase;
  bool    homed;
  bool    zeroKnown;             // homed at least once; zero survives faults
  long    idxRoughEnc;           // fast-sweep index capture (fallback zero)
  uint8_t verRetry;              // unwind/re-home attempts used this CAL

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

static inline float sunElF() { return sunEl + frameElOff; }
static inline float sunAzF() { return wrap180(sunAz + frameAzOff); }

static inline float rampV(float x) {
  return x * x * x * (10.0f - 15.0f * x + 6.0f * x * x);
}
static inline float rampQ(float x) {
  float x2 = x * x;
  return x2 * x2 * (2.5f - 3.0f * x + x2);     // 2.5x^4 - 3x^5 + x^6
}

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
unsigned long motorsReadyUs = 0;

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
    long settle = (long)(motorsReadyUs - a.t0us);
    if (settle > 0) a.t0us += (unsigned long)settle;
  }
  a.moving = true;
}

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

void snagService(Axis &a) {
  if (!a.snagWatch || !a.moving) return;
  unsigned long now = millis();
  if (now - a.lastSnagChkMs < SNAG_CHECK_MS) return;
  a.lastSnagChkMs = now;
  long encNow   = encRead(a);
  long stepsNow = a.stepsDone;
  float cmdD = (float)(stepsNow - a.lastChkSteps) * (360.0f / (float)a.stepsPerRev);
  float encD = (float)(encNow - a.lastChkEnc) * (float)a.dir * (360.0f / (float)ENC_CPR);
  a.lastChkEnc   = encNow;
  a.lastChkSteps = stepsNow;
  float dt = (float)SNAG_CHECK_MS * 1e-3f;         // nominal window (loop jitter only lengthens it, which biases AGAINST a false trip)
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

bool legShort(Axis &a) {
  return fabsf(encDeg(a) - a.legTargetDeg) > VERIFY_END_TOL_DEG;
}

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
      if (r == LEG_SNAGGED) {
        serialWrite(F("HOME "));
        serialWrite(a.name);
        serialWriteLine(F(": prezero return blocked — searching from here"));
      }
      startSearchFwd(a);
      break;

    case H_FWD:
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
        if (&a == &m1 && a.searchRetry < HOME_SEARCH_RETRY) {
          a.searchRetry++;
          enterSearchRest(a);
          return;
        }
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

Axis *verStageAxis(uint8_t s) {
  switch (s) {
    case V_EL_FWD: case V_EL_BACK: case V_EL_RET: case V_EL_PARK: return &m1;
    case V_AZ_FWD: case V_AZ_BACK: case V_AZ_RET:                 return &m2;
    default:                                                      return verFailAxis;
  }
}

float elAzValPark() {
  float lo = sunElF() - sunHalfEl() - SUN_CLEAR_DEG;
  float hi = sunElF() + sunHalfEl() + SUN_CLEAR_DEG;
  float p  = EL_AZVAL_PARK_DEG;
  if (p > lo && p < hi)
    p = (p - lo <= hi - p) ? lo : hi;      // nearest safe side
  if (p >  LIMIT_DEG) p =  LIMIT_DEG;
  if (p < -LIMIT_DEG) p = -LIMIT_DEG;
  return p;
}

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

void homingFinish() {
  verStage = V_IDLE;
  calStage = C_NONE;
  qcalActive = false;
  calSkipVerify = false;           // consumed; next full CAL verifies again
  m1.targetDeg = encDeg(m1);       // anchor the active hold HERE, so the
  m2.targetDeg = encDeg(m2);       // station keeper never chases a stale
  reqEl = m1.targetDeg;            // pre-CAL target
  reqAz = m2.targetDeg;

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
      verEnter((verFailAxis == &m2) ? V_AZ_RET : V_EL_FWD);
    }
    return;
  }

  Axis *a = verStageAxis(verStage);
  if (a == NULL) return;
  uint8_t r = legService(*a);
  if (r == LEG_RUNNING) return;
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
  calSkipVerify = true;
  serialWriteLine(F("QCAL: index missed — full homing sweep (validation skipped)"));
  calBeginFull();
}

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
  startSlewTo(0.0f, 0.0f);           // sun-routed; pushes SLEWING
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

static inline float flipEl(float el) {
  float p = el - frameElOff;                       // to physical
  p = (p >= 0.0f) ? 180.0f - p : -180.0f - p;      // flip there
  return p + frameElOff;                           // back to current frame
}

bool sunBlockedSky(float el, float az, float pad) {
  return sunBlocked(el, az, pad) ||
         sunBlocked(flipEl(el), wrap180(az + 180.0f), pad);
}

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

uint8_t planSunRoute(float e0, float a0, float e1, float a1) {
  if (!sunPathBlocked(e0, a0, e1, a1)) return 0;
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
    float edge = (fabsf(elHi - ce) <= fabsf(ce - elLo)) ? elHi : elLo;
    sunWpEl[n] = edge; sunWpAz[n] = a0; n++;
    ce = edge;
    escaped = true;
  } else if (ce > elLo && ce < elHi) {
    float edge;
    if      (e1 >= elHi) edge = elHi;
    else if (e1 <= elLo) edge = elLo;
    else                 edge = (ce >= bcEl) ? elHi : elLo;
    sunWpEl[n] = edge; sunWpAz[n] = a0; n++;
    ce = edge;
  }

  float eHold = (ce <= elLo) ? fminf(e1, elLo) : fmaxf(e1, elHi);
  if (fabsf(eHold - ce) > 0.01f) {
    sunWpEl[n] = eHold; sunWpAz[n] = a0; n++;
    ce = eHold;
  }

  // If the target elevation is across/inside the band, EL may only
  // finish once AZ has cleared the box's azimuth band.
  bool needCross = fabsf(eHold - e1) > 0.01f;
  if (needCross) {
    bool a1InBand = (a1 > azLo && a1 < azHi);
    if (a1InBand) {
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

void printPos() {
  float el = encDeg(m1), az = encDeg(m2);
  float fe = flipEl(el), fa = wrap180(az + 180.0f);
  float dRaw  = fmaxf(fabsf(wrap180(el - reqEl)), fabsf(wrap180(az - reqAz)));
  float dFlip = fmaxf(fabsf(wrap180(fe - reqEl)), fabsf(wrap180(fa - reqAz)));
  if (dFlip < dRaw) { el = fe; az = fa; }
  el = reqEl + wrap180(el - reqEl);
  az = reqAz + wrap180(az - reqAz);
  serialWrite(F("POS "));
  serialWrite(el, 2);
  serialWrite(' ');
  serialWriteLine(az, 2);
}

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
  serialWriteLine(F("Every line needs the trailer #<seq>*<cs>  (cs = CRC-16/CCITT-FALSE of all bytes before '*', 4 hex digits)"));
  serialWriteLine(F("  e.g.  POS #7*7735   -> reply ACK 7 then POS ...;  bad checksum -> NAK <seq> checksum"));
}

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
  bool land = (sunLegCount == 0);
  axisStartMoveDeg(m1, fe - encDeg(m1), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, true, land);
  axisStartMoveDeg(m2, fa - encDeg(m2), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, true, land);
  sysState = ST_MOVING;
  pushStatus(PY_SLEWING);
}

static uint16_t crc16(const char *p, const char *end) {
  uint16_t c = 0xFFFF;
  for (; p < end; ++p) {
    c ^= (uint16_t)((uint8_t)*p) << 8;
    for (uint8_t i = 0; i < 8; i++)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

static bool parseSeqChecksum(char *line, uint8_t &seq, int &looseSeq) {
  looseSeq = -1;
  char *star = strrchr(line, '*');
  char *hash = strrchr(line, '#');
  if (hash) looseSeq = atoi(hash + 1);
  if (!star || !hash || hash > star) return false;
  uint16_t cs = crc16(line, star);
  if (strlen(star + 1) != 4) return false;
  for (uint8_t i = 1; i <= 4; i++)
    if (!isxdigit((unsigned char)star[i])) return false;
  if (cs != (uint16_t)strtoul(star + 1, NULL, 16)) return false;
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

    {
      float ca = encDeg(m2);
      float t2n = ca + wrap180(t2 - ca);
      if (t2n >= M2_MIN_DEG && t2n <= M2_MAX_DEG) t2 = t2n;  // in-ROM, shortest
      else                                        t2 = wrap180(t2);  // outlawed dir
    }

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

    reqEl = t1;                            // remember the REQUESTED frame so
    reqAz = t2;                            // POS reads back in it (see printPos)
    float candE[2] = { t1, t1 }, candA[2] = { t2, t2 };
    uint8_t nCand = 1;
    if (t2 > M2_MAX_DEG || t2 < M2_MIN_DEG) {
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
    if (strtok(NULL, " \t")) {
      serialWriteLine(F("CAL: host az argument ignored — sun avoidance is firmware-owned"));
    }
    calSkipVerify = false;   // an explicit CAL always runs the +/-180 validation
    calBeginFull();

  } else if (!strcasecmp(cmd, "SETZERO")) {
    if (m1.moving || m2.moving) {
      pushErrMsg(PY_ERR_BUSY, F("busy")); return;
    }
    if (frameElOff == 0.0f && frameAzOff == 0.0f) {
      serialWriteLine(F("ZERO SET"));
      pushStatus(pyStatusCode());
      return;
    }
    long zs1 = lround((double)HOME_EL_OFFSET_DEG / 360.0 * (double)ENC_CPR);
    long zs2 = lround((double)HOME_AZ_OFFSET_DEG / 360.0 * (double)ENC_CPR);
    noInterrupts();
    m1.enc -= zs1;
    m2.enc -= zs2;
    interrupts();
    m1.indexRefEnc -= zs1;
    m2.indexRefEnc -= zs2;
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
#if !BENCH_COMMANDS
    pushErrMsg(PY_ERR_CMD, F("RAW disabled (flight build)")); return;
#endif
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
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");
    if (a1 && a2) {
      float el = atof(a1), az = atof(a2);
      // Written as a positive range test so NaN is rejected too: a NaN center
      // makes every box comparison false, i.e. NO keep-out at all.
      if (!(el >= -180.0f && el <= 180.0f && az >= -360.0f && az <= 360.0f)) {
        pushErrMsg(PY_ERR_ANGLE, F("SUN center out of range - keep-out unchanged"));
        return;
      }
      sunEl = el;
      sunAz = wrap180(az);
      serialWrite(F("SUN OK "));
      serialWrite(el, 3);
      serialWrite(' ');
      serialWriteLine(az, 3);
      return;
    }
#if !BENCH_COMMANDS
    pushErrMsg(PY_ERR_CMD, F("bare SUN disabled (flight build)")); return;
#endif
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
  pushErrMsg(PY_ERR_RESET, F("RESET: controller (re)booted - not homed, sun center at default (send SUN, then CAL)"));
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
      printTgt();
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
        float w1 = sunWpEl[sunLegIdx] - encDeg(m1);
        float w2 = sunWpAz[sunLegIdx] - encDeg(m2);
        if (fabsf(w1) > SUN_WP_TOL_DEG || fabsf(w2) > SUN_WP_TOL_DEG) {
          sunLegIdx = sunLegCount = 0;
          recalSweepPending = false;
          settleWaitEnd = 0;
          sysState = ST_IDLE;
          m1.targetDeg = encDeg(m1);       // hold HERE: the active hold must not
          m2.targetDeg = encDeg(m2);       // trim straight toward the old target
          reqEl = m1.targetDeg;
          reqAz = m2.targetDeg;
          serialWrite(F("ERR "));
          serialWrite(PY_ERR_SETTLE);
          serialWrite(F(" settle "));
          serialWrite(w1, 3);
          serialWrite(' ');
          serialWrite(w2, 3);
          serialWriteLine(F(" sun-detour waypoint not reached - move aborted"));
          printPos();
          pushStatus(PY_IDLE);
          return;
        }
        sunLegIdx++;
        bool fin = (sunLegIdx >= sunLegCount);   // final leg -> real target
        float ne = fin ? m1.targetDeg : sunWpEl[sunLegIdx];
        float na = fin ? m2.targetDeg : sunWpAz[sunLegIdx];
        axisStartMoveDeg(m1, ne - encDeg(m1), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, true, fin);
        axisStartMoveDeg(m2, na - encDeg(m2), DANCE_SPEED_DPS, DANCE_ACCEL_DPS2, true, fin);

      } else if (sysState == ST_MOVING) {
        if (settleWaitEnd == 0) {
          settleWaitEnd = millis() + SETTLE_WAIT_MS;
          if (settleWaitEnd == 0) settleWaitEnd = 1;
        } else if ((long)(millis() - settleWaitEnd) >= 0) {
          float e1 = m1.targetDeg - encDeg(m1);
          float e2 = m2.targetDeg - encDeg(m2);
          bool ok = fabsf(e1) <= SETTLE_TOL_DEG && fabsf(e2) <= SETTLE_TOL_DEG;
          if (ok && recalSweepPending) {
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
            pushStatus(PY_IDLE);
            if (MOVES_PER_RECAL > 0 && m1.homed && m2.homed &&
                ++movesSinceRecal >= MOVES_PER_RECAL) {
              movesSinceRecal = 0;
              beginEducatedRecal();
            }
          } else if (++settlePasses > SETTLE_MAX_PASSES) {
            settleWaitEnd = 0;
            sysState = ST_IDLE;              // give up honestly; the host decides
            recalSweepPending = false;       // a failed re-cal approach is abandoned
            m1.targetDeg = encDeg(m1);
            m2.targetDeg = encDeg(m2);
            reqEl = m1.targetDeg;
            reqAz = m2.targetDeg;
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