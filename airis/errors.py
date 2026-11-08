"""One error scheme for the whole stack.

A failure is a code with a subsystem prefix, a short name, a human message and
optional data. It prints as ``MNT-63 BUSY: mount is executing a move (seq=17)``
and the code is the first token on every log line and CSV row that carries it,
so a grep for the code finds every occurrence across every file.

Prefix  Owner
LNK     airis_bridge (the serial link)
MNT     firmware codes, mirrored one-to-one
PTG     pointing
NAV     nav
DET     detect
DLK     downlink
CAM     camera module
SCH     scheduler
"""
from dataclasses import dataclass, field
from enum import Enum


class Code(str, Enum):
    # link
    LNK_NAK = "LNK-01"
    LNK_NO_ACK = "LNK-02"
    LNK_PORT_LOST = "LNK-03"
    LNK_TIMEOUT = "LNK-04"
    LNK_UNEXPECTED = "LNK-05"
    # mount (firmware codes)
    MNT_GENERAL = "MNT-50"
    MNT_HOME_M1 = "MNT-51"
    MNT_HOME_M2 = "MNT-52"
    MNT_CATCH_M1 = "MNT-53"
    MNT_CATCH_M2 = "MNT-54"
    MNT_LIMIT_M1 = "MNT-55"
    MNT_LIMIT_M2 = "MNT-56"
    MNT_SUN = "MNT-57"
    MNT_SETTLE = "MNT-58"
    MNT_SUN_UNKNOWN = "MNT-59"
    MNT_CMD = "MNT-60"
    MNT_ANGLE = "MNT-61"
    MNT_UNDERFLOW = "MNT-62"
    MNT_BUSY = "MNT-63"
    MNT_NOT_HOMED = "MNT-64"
    MNT_FAULT_LATCHED = "MNT-65"
    MNT_LINE_TOO_LONG = "MNT-66"
    MNT_RESET = "MNT-67"
    MNT_STOPPED = "MNT-68"
    # pointing
    PTG_BELOW_HORIZON = "PTG-01"
    PTG_SUN_RADIUS = "PTG-02"
    PTG_HEADING_STALE = "PTG-03"
    PTG_RANGE = "PTG-04"
    # nav
    NAV_TRIMBLE = "NAV-00"        # not an error: position is from the Trimble
    NAV_NO_FIX = "NAV-01"
    NAV_STALE_USING_ADAPT = "NAV-02"
    NAV_USING_DEFAULTS = "NAV-03"
    NAV_BAD_SENTENCE = "NAV-04"
    # detect
    DET_REGISTRATION = "DET-01"
    DET_SUBTRACTION = "DET-02"
    DET_REFERENCE_EMPTY = "DET-03"
    # downlink
    DLK_NO_SENDER = "DLK-01"
    DLK_CHUNK_REJECTED = "DLK-02"
    DLK_QUEUED = "DLK-03"
    # camera
    CAM_OVERHEAT = "CAM-451"
    CAM_NO_FRAME = "CAM-01"
    # scheduler
    SCH_EMPTY = "SCH-01"
    SCH_BAD_MAP = "SCH-02"


FIRMWARE_CODE_TO_ERR = {int(c.value.split("-")[1]): c for c in Code if c.value.startswith("MNT-")}


@dataclass
class Fault(Exception):
    code: Code
    msg: str = ""
    data: dict = field(default_factory=dict)

    def __str__(self) -> str:
        extra = " ".join(f"{k}={v}" for k, v in self.data.items())
        name = self.code.name.split("_", 1)[1]
        return f"{self.code.value} {name}: {self.msg}" + (f" ({extra})" if extra else "")


def fw_err(code: int, msg: str = "") -> Fault:
    """Map a firmware ``ERR <code>`` line to a Fault."""
    return Fault(FIRMWARE_CODE_TO_ERR.get(code, Code.MNT_GENERAL), msg, {"fw_code": code})
