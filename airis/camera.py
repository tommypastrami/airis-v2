"""The flight camera: a XIMEA behind the CameraLike seam in airis_main (Jetson only).

Ported from the old airis_photo_capture.py (camera_fixed_setting_initialization,
camera_customizable_setting_initialization, set_bit_depths, capture_one_exposure).
Settings come from config.py; vars.yaml is retired. Frames arrive binned 3x3 by the
camera, so the frame shape is whatever the array says. No WCS is attached: registration
does not exist yet, so every frame reports DET-01 downstream.
"""
from __future__ import annotations

import logging
import threading
from datetime import datetime, timezone
from typing import Optional

from . import config
from .detect import Frame
from .errors import Code, Fault

log = logging.getLogger("airis.camera")


class XimeaCamera:  # pragma: no cover - hardware only
    """One open device, configured once. capture() and temperature_c() share a lock because
    the thermal watchdog reads the temperature from another thread."""

    def __init__(self):
        from ximea import xiapi            # exists only on the Jetson
        self._xiapi = xiapi
        self._lock = threading.Lock()
        self._cam = xiapi.Camera()
        self._cam.open_device()
        cam = self._cam
        # 1. fixed settings
        cam.set_shutter_type('XI_SHUTTER_ROLLING')
        cam.disable_aeag()
        cam.enable_new_process_chain_enable()
        # 2. flat-field correction, only with both calibration files
        if config.CAM_FLAT_FILE and config.CAM_DARK_FILE:
            cam.set_ffc_flat_field_file_name(config.CAM_FLAT_FILE)
            cam.set_ffc_dark_field_file_name(config.CAM_DARK_FILE)
            cam.enable_ffc()
        else:
            log.warning("flat-field correction is OFF: CAM_FLAT_FILE and CAM_DARK_FILE are not both set")
        # 3. binning and data format
        cam.set_downsampling_type('XI_BINNING')
        cam.set_downsampling(config.CAM_BINNING)
        cam.set_imgdataformat(config.CAM_IMG_FORMAT)
        cam.disable_output_bit_packing()
        # 4. bit depths (12-bit sensor readout is the most the link carries when binning)
        cam.set_sensor_bit_depth(config.CAM_SENSOR_BIT_DEPTH)
        cam.set_output_bit_depth(config.CAM_OUTPUT_BIT_DEPTH)
        cam.set_image_data_bit_depth(config.CAM_IMAGE_BIT_DEPTH)
        # 5. gain
        cam.set_gain_selector('XI_GAIN_SELECTOR_ANALOG_ALL')
        cam.set_gain(config.CAM_GAIN_DB)

    def capture(self, tile_ra: float, tile_dec: float, exposure_s: float) -> Frame:
        xiapi = self._xiapi
        with self._lock:
            try:
                self._cam.set_exposure(int(exposure_s * 1e6))          # xiAPI exposure is microseconds
                self._cam.start_acquisition()
                try:
                    utc = datetime.now(timezone.utc)
                    img = xiapi.Image()
                    # UNVERIFIED on the Jetson's xiAPI: the `timeout` keyword (milliseconds)
                    self._cam.get_image(img, timeout=int(exposure_s * 1000) + 5000)
                    data = img.get_image_data_numpy()
                finally:
                    self._cam.stop_acquisition()
            except xiapi.Xi_error as e:
                raise Fault(Code.CAM_NO_FRAME, f"capture failed: {e}", {"exposure_s": exposure_s})
        return Frame(data, utc, exposure_s, tile_ra=tile_ra, tile_dec=tile_dec)    # wcs stays None (D4)

    def temperature_c(self) -> Optional[float]:
        with self._lock:
            try:
                return float(self._cam.get_sensor_board_temp())
            except self._xiapi.Xi_error as e:
                log.warning("sensor temperature read failed: %s", e)
                return None

    def close(self) -> None:
        with self._lock:
            self._cam.close_device()
