"""Every tunable in one place."""
from dataclasses import dataclass, field


# ---- Site defaults (used before the first GPS fix) --------------------------
DEFAULT_LAT_DEG = -77.85       # McMurdo (launch site); replace with the surveyed pad
DEFAULT_LON_DEG = 166.67       # east-positive
DEFAULT_HEIGHT_M = 30.0

# ---- Frames (see pointing.py docstring) ------------------------------------
EL_OFFSET_DEG = 0.0            # encoder-home-to-sky, measured by plate solve
AZ_OFFSET_DEG = 0.0
MOUNT_TO_GONDOLA_AZ_DEG = 0.0  # mount az zero relative to the HDT baseline
HEADING_CORRECTION_ENABLED = False  # flip to True in the SAME change for pointing and sun

HOME_EL_OFFSET_DEG = -55.0
HOME_AZ_OFFSET_DEG = -35.0

ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG = 180.0
MIN_ELEVATION_DEG = 0.0

# ---- Sun keep-out ----------------------------------------------------------
SUN_WIN_EL_DEG = 25.0          # full span of the window
SUN_WIN_AZ_DEG = 35.0
FW_SUN_FOV_DEG = 15.0          # firmware CAM_FOV_DEG: what it inflates the window by
CAM_FOV_W_DEG = 10.3           # the real camera footprint (scheduler coverage)
CAM_FOV_H_DEG = 6.9
SUN_CLEAR_DEG = 2.0
SUN_PUSH_PERIOD_S = 60.0
SUN_TILE_MARGIN_DEG = 5.0      # scheduler never queues a tile within this of the box edge
HEADING_STALE_S = 5.0

# ---- Mount motion model (from firmware constants; calibrate from moves.csv) --
SLEW_SPEED_DPS = 50.0
SLEW_ACCEL_DPS2 = 45.0
DECEL_RATIO = 0.75
LAND_ZONE_DEG = 3.0
LAND_SPEED_DPS = 4.0
LAND_ACCEL_DPS2 = 15.0
SETTLE_TOL_DEG = 1.5
FLIP_MARGIN_DEG = 10.0         # take the over-the-top attitude only if it saves more than this
SETTLE_MAX_PASSES = 3
HOLD_SILENCE_S = 1800.0        # firmware HOLD_SILENCE_MS: drivers released after this without a command

# ---- Serial link (bridge <-> firmware) --------------------------------------
SERIAL_BAUD = 115200
SERIAL_CANDIDATES = ["/dev/airis_mount", "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
ACK_TIMEOUT_S = 0.25
ACK_RETRIES = 3
MOVE_START_TIMEOUT_S = 2.0     # STATUS 15 is pushed synchronously; 2 s is generous
MOVE_MAX_S = 45.0              # flat DONE wait.
CAL_START_TIMEOUT_S = 5.0
CAL_FINISH_TIMEOUT_S = 180.0
CAL_KEEPALIVE_PERIOD_S = 30.0  # STATUS during CAL so the firmware's 120 s homing sleep never fires
FAULT_AUTO_CLEAR_MAX = 3       # automatic ACK + CAL attempts on a latched fault...
FAULT_AUTO_CLEAR_WINDOW_S = 1800.0  # ...per this window (an explicit `cal` request is never limited)
LINK_SILENCE_S = 10.0          # no line at all for this long = link dead
KEEPALIVE_PERIOD_S = 120.0     # STATUS while idle so the hold never lapses

# ---- Bridge socket -----------------------------------------------------------
BRIDGE_SOCKET_PATH = "/tmp/airis_bridge.sock"
BRIDGE_CLIENT_TIMEOUT_S = 300.0  # must cover CAL, and a move that triggers an automatic re-CAL

# ---- ADAPT link --------------------------------------------------------------
ADAPT_HOST = "0.0.0.0"
ADAPT_PORT_CMD = 5005
ADAPT_PORT_NAV = 5006
ADAPT_SYNC_WORD = bytes.fromhex("ADA72026ADA72026")
ADAPT_MAX_ENVELOPE_CMD = 64 * 1024 * 1024
ADAPT_MAX_ENVELOPE_NAV = 64 * 1024
ADAPT_RECV_TIMEOUT_CMD_S = 30.0
ADAPT_RECV_TIMEOUT_NAV_S = 10.0
ADAPT_KEEPALIVE_IDLE_S = 30     # TCP keepalive: first probe after this much silence,
ADAPT_KEEPALIVE_INTVL_S = 10    # then one every this long,
ADAPT_KEEPALIVE_CNT = 3         # and the link is dead after this many unanswered

# ---- airis-comms server (comms/cppImplementation, see comms_reference.md) ----
# One serialized MessageEnvelope per UDP datagram, no framing.
COMMS_HOST = "127.0.0.1"
COMMS_PORT_COMMANDS = 9001      # comms -> controls: pointing_cmd, safe_mode_exit_cmd
COMMS_PORT_DOWNLINK = 9003      # controls -> comms: events, images, beacons

# ---- Nav ---------------------------------------------------------------------
TRIMBLE_UDP_PORT = 31250
TRIMBLE_STALE_S = 10.0
NAV_PUSH_MIN_PERIOD_S = 5.0
ACCEPTED_GGA_QUALITIES = {1, 2, 4, 5}

# ---- Orchestrator ------------------------------------------------------------
OVERHEAT_C = 60.0              # camera sensor temperature that enters safe mode
SLEW_RETRY_BACKOFF_S = 5.0     # wait after a failed slew before trying again
SLEW_MAX_RETRIES = 3           # tile-specific failures (e.g. settle) before the tile is retired
LOCK_ON_REVISITS = 3           # PLACEHOLDER: times a hit tile is re-imaged before the queue resumes
SAFE_PARK_EL_DEG = 0.0         # PLACEHOLDER: where the mount parks on entering safe mode
SAFE_PARK_AZ_DEG = 0.0         # (physical home; the firmware still refuses it if sun-facing)
LOOP_ERROR_BACKOFF_S = 1.0     # pause after an unexpected exception in a worker loop
IDLE_RESCORE_PERIOD_S = 30.0   # with nothing queued, rebuild the queue this often
GPS_CLOCK_WARN_S = 2.0         # log when the system clock and GPS UTC differ by more than this

# ---- Scheduler ---------------------------------------------------------------
HEALPIX_DTYPE = "<f8"
HEALPIX_NESTED = False
SCHED_CANDIDATE_PIXELS = 600   # top-N pixels tried as FOV centers
EXPOSURE_S = 8.0
PROC_S = 4.0

# ---- Detection ---------------------------------------------------------------
PSF_SIGMA_PX = 1.2
DETECT_K_SIGMA = 5.0
CONFIDENCE_THRESHOLD = 0.8
MIN_PEAK_AREA_PX = 3

# ---- Downlink ----------------------------------------------------------------
CHUNK_BYTES = 60000            # one image_data per UDP datagram (65507-byte limit, minus the envelope)
CUTOUT_PX = 128
THUMB_SCALE = 8
DOWNLINK_QUEUE_DIR = "downlink_queue"

# ---- Thermal -----------------------------------------------------------------
THERMAL_POLL_S = 20.0
JETSON_SHUTDOWN_C = 88.0
SENSOR_HEAT_ON_C = -15.0
SENSOR_HEAT_OFF_C = -12.0      # hysteresis: once on, stay on until the sensor is this warm
SENSOR_MIN_C = -25.0

# ---- Paths -------------------------------------------------------------------
IMG_SAVES = "img_saves"
MOVES_CSV = "moves.csv"
CANDIDATES_CSV = "candidates.csv"


@dataclass
class SunBoxSpec:
    """Half-sizes of the firmware's box: sunHalfEl() / sunHalfAz() in the .ino (20 x 25 deg)."""
    half_el: float = SUN_WIN_EL_DEG / 2 + FW_SUN_FOV_DEG / 2
    half_az: float = SUN_WIN_AZ_DEG / 2 + FW_SUN_FOV_DEG / 2
