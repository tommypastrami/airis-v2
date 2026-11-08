"""Every tunable in one place. Units in the name or the comment; degrees unless stated.

Anything that used to be duplicated across files (sun center, frame offsets,
angle limits, timeouts) lives here once. The firmware's copies of the frame
offsets and the sun window MUST match these (see firmware/FIRMWARE_V5_CHANGES.md).
"""
from dataclasses import dataclass, field


# ---- Site defaults (used before the first GPS fix) --------------------------
DEFAULT_LAT_DEG = 31.7793      # NASA CSBF, Palestine TX
DEFAULT_LON_DEG = -95.6311     # east-positive
DEFAULT_HEIGHT_M = 159.0

# ---- Frames (see pointing.py docstring) ------------------------------------
EL_OFFSET_DEG = 0.0            # encoder-home-to-sky, measured by plate solve
AZ_OFFSET_DEG = 0.0
MOUNT_TO_GONDOLA_AZ_DEG = 0.0  # mount az zero relative to the HDT baseline
HEADING_CORRECTION_ENABLED = False  # flip to True in the SAME change for pointing and sun

# Index-pulse-to-physical-home offsets. After CAL the bridge drives here and
# sends SETZERO so that encoder (0, 0) is the physical home. Must equal the
# firmware's HOME_EL_OFFSET_DEG / HOME_AZ_OFFSET_DEG.
HOME_EL_OFFSET_DEG = -55.0
HOME_AZ_OFFSET_DEG = -35.0

ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG = 180.0
MIN_ELEVATION_DEG = 0.0

# ---- Sun keep-out ----------------------------------------------------------
SUN_WIN_EL_DEG = 25.0          # full span of the window
SUN_WIN_AZ_DEG = 35.0
CAM_FOV_W_DEG = 10.3
CAM_FOV_H_DEG = 6.9
SUN_CLEAR_DEG = 2.0
SUN_PUSH_PERIOD_S = 60.0
SUN_TILE_EXCLUSION_DEG = 30.0  # scheduler never queues a tile this close to the sun
HEADING_STALE_S = 5.0

# ---- Mount motion model (from firmware constants; calibrate from moves.csv) --
SLEW_SPEED_DPS = 67.5
SLEW_ACCEL_DPS2 = 112.5
DECEL_RATIO = 0.75
LAND_ZONE_DEG = 3.0
LAND_SPEED_DPS = 4.0
SETTLE_TOL_DEG = 1.5
FLIP_MARGIN_DEG = 10.0         # take the over-the-top attitude only if it saves more than this
SETTLE_MAX_PASSES = 3
HOLD_SILENCE_S = 300.0         # firmware releases drivers after this without a command

# ---- Serial link (bridge <-> firmware) --------------------------------------
SERIAL_BAUD = 115200
SERIAL_CANDIDATES = ["/dev/airis_mount", "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
ACK_TIMEOUT_S = 0.25
ACK_RETRIES = 3
MOVE_START_TIMEOUT_S = 2.0     # STATUS 15 is pushed synchronously; 2 s is generous
MOVE_MARGIN_S = 5.0            # added to the model slew time for the DONE wait
MOVE_MAX_S = 30.0
CAL_START_TIMEOUT_S = 5.0
CAL_FINISH_TIMEOUT_S = 180.0
LINK_SILENCE_S = 10.0          # no line at all for this long = link dead
KEEPALIVE_PERIOD_S = 120.0     # STATUS while idle so the hold never lapses

# ---- Bridge socket -----------------------------------------------------------
BRIDGE_SOCKET_PATH = "/tmp/airis_bridge.sock"
BRIDGE_CLIENT_TIMEOUT_S = 60.0

# ---- ADAPT link --------------------------------------------------------------
ADAPT_HOST = "0.0.0.0"
ADAPT_PORT_CMD = 5005
ADAPT_PORT_NAV = 5006
ADAPT_SYNC_WORD = bytes.fromhex("ADA72026ADA72026")
ADAPT_MAX_ENVELOPE_CMD = 64 * 1024 * 1024
ADAPT_MAX_ENVELOPE_NAV = 64 * 1024
ADAPT_RECV_TIMEOUT_CMD_S = 30.0
ADAPT_RECV_TIMEOUT_NAV_S = 10.0
ADAPT_DEAD_AFTER_TIMEOUTS = 10  # consecutive recv timeouts on 5005 before dropping the link

# ---- Nav ---------------------------------------------------------------------
TRIMBLE_UDP_PORT = 31250
TRIMBLE_STALE_S = 10.0
NAV_PUSH_MIN_PERIOD_S = 5.0
ACCEPTED_GGA_QUALITIES = {1, 2, 4, 5}

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
CHUNK_BYTES = 64 * 1024
CUTOUT_PX = 128
THUMB_SCALE = 8
DOWNLINK_QUEUE_DIR = "downlink_queue"

# ---- Thermal -----------------------------------------------------------------
THERMAL_POLL_S = 20.0
JETSON_SHUTDOWN_C = 88.0
SENSOR_HEAT_ON_C = -15.0
SENSOR_MIN_C = -25.0

# ---- Paths -------------------------------------------------------------------
IMG_SAVES = "img_saves"
MOVES_CSV = "moves.csv"
CANDIDATES_CSV = "candidates.csv"


@dataclass
class SunBoxSpec:
    half_el: float = SUN_WIN_EL_DEG / 2 + CAM_FOV_H_DEG / 2 + SUN_CLEAR_DEG
    half_az: float = SUN_WIN_AZ_DEG / 2 + CAM_FOV_W_DEG / 2 + SUN_CLEAR_DEG
