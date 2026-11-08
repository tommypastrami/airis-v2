"""AIRIS controls v2 — one job per file, one frame, one error scheme.

Modules
    config        every tunable in one place
    errors        the LNK/MNT/PTG/NAV/DET/DLK/CAM code scheme
    protocol      serial line protocol v5 (sequence + checksum, ACK/NAK)
    pointing      the only module that converts between coordinate frames
    nav           Trimble NMEA assembly + ADAPT fallback
    adapt_link    sync+length framing, listener, sender, dispatch
    bridge_client Unix-socket client of the bridge
    airis_bridge  sole owner of the Arduino link
    scheduler     priority heap of tiles (FOV-integrated probability / time)
    detect        register, clean, subtract, detect, measure, vet
    downlink      products sized for the link, chunking, comms modes
    payload_hw    RTDs, heater policy, thermal watchdog
    airis_main    the orchestrator: acquire thread + analyze thread
"""
__version__ = "2.0.0-draft"
