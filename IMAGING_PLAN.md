# AIRIS v2: imaging integration plan

Goal: bring the old flight code's camera capture and SFFT image subtraction into `airis-v2`,
behind the seams v2 already has. Registration (plate solving) is deferred; see "Deferred".

## Ground rules

- Work only in `airis-v2`. Do not edit `airis-comms`, `sim/`, `firmware/`, or any function not named below.
- Simplest thing that works. No new abstractions, config layers or options beyond what is listed.
- Files are CRLF. Keep the line endings.
- `ximea`, `sfft`, `cupy` and `spidev` exist only on the Jetson. Import them inside functions or
  constructors, never at module top level, and mark hardware-only code `# pragma: no cover`.
- `python -m pytest` currently passes 95 tests (about 2.5 min). It must still pass after every step.
- If anything below contradicts the code you find, stop and ask. Do not guess.

## Existing seams (do not change these signatures)

- `airis/airis_main.py`: `CameraLike` = `capture(tile_ra, tile_dec, exposure_s) -> Frame` and
  `temperature_c() -> Optional[float]`. `Orchestrator(mount, camera, catalog, detector, downlink, sender, nav)`
  where `catalog` is `Callable[[float, float], list[Star]]`.
- `airis/detect.py`: `Frame(data, utc, exposure_s, psf_sigma_px, wcs, tile_ra, tile_dec, path)`,
  `Star(ra, dec, flux)`, `Subtractor.subtract(science, reference, sci_sigma, ref_sigma) -> ndarray`,
  `Pipeline(registrar=None, subtractor=None, ...)`. `SFFTSubtractor.subtract` is a stub that raises.
- `main()` currently does `import airis_photo_capture as camera_mod` and uses `camera_mod.catalog`
  and `camera_mod.temperature_c`. That module does not exist in v2 and must not be ported as is.

## Decisions already made (defaults; flagged to Matt)

| # | Decision | Default in this plan |
| --- | --- | --- |
| D1 | Camera settings live in `config.py`; `vars.yaml` is retired | Values copied from the current `vars.yaml` |
| D2 | Exposure: `vars.yaml` says 1 s, v2 `EXPOSURE_S` is 8.0 (two 4 s halves), old reference model assumed 10 s | Camera obeys the `exposure_s` argument; `EXPOSURE_S` is NOT changed. Camera team to confirm |
| D3 | Overheat limit: `vars.yaml` 51 C, v2 `OVERHEAT_C` 60 C | Set `OVERHEAT_C = 51.0` |
| D4 | No registrar yet | Flight path reports `DET-01` on every frame until registration exists. No placeholder WCS in flight code; the bench tool (step 6) uses a nominal one |
| D5 | Frames arrive binned 3x3 by the camera (`XI_DWN_3x3`) | No software binning. Never hard-code the frame shape; read it from the array |

## Step 1: `airis/config.py`

Add a camera block and a catalog/SFFT block. Change `OVERHEAT_C` to 51.0 (D3). Touch nothing else.

```python
# ---- Camera (XIMEA; replaces vars.yaml) --------------------------------------
CAM_BINNING = "XI_DWN_3x3"
CAM_IMG_FORMAT = "XI_MONO16"
CAM_SENSOR_BIT_DEPTH = "XI_BPP_12"
CAM_OUTPUT_BIT_DEPTH = "XI_BPP_16"
CAM_IMAGE_BIT_DEPTH = "XI_BPP_16"
CAM_GAIN_DB = 6
CAM_FLAT_FILE = None           # flat-field correction is OFF until both files are set
CAM_DARK_FILE = None

# ---- Reference catalog and SFFT ----------------------------------------------
GAIA_CHUNK_DIR = "/home/wusat/airis/airis_gaia_chunks"
GAIA_ZERO_POINT = 18.0
SFFT_SATURATE = 51000          # old value; 12-bit summed 3x3 would top out at 36855 - bench-check
SFFT_GAIN = 0                  # old value, copied as is
```

## Step 2: `airis/camera.py` (new)

One class, `XimeaCamera`, satisfying `CameraLike`. Source: old `airis_photo_capture.py`
(`camera_fixed_setting_initialization`, `camera_customizable_setting_initialization`,
`set_bit_depths`, `capture_one_exposure`). Do NOT port `process_handler`,
`photo_capture_sequence`, `save_img_as_np_array`, the multiprocessing, the yaml loader,
or the `image_subtraction` import.

- `__init__`: `from ximea import xiapi`; open the device ONCE; apply settings in this order:
  1. `set_shutter_type('XI_SHUTTER_ROLLING')`, `disable_aeag()`, `enable_new_process_chain_enable()`
  2. Only if both `CAM_FLAT_FILE` and `CAM_DARK_FILE` are set: `set_ffc_flat_field_file_name`,
     `set_ffc_dark_field_file_name`, `enable_ffc()`. Otherwise log a WARNING that FFC is off.
     No bare `except` (the old code hid a missing-key error this way).
  3. `set_downsampling_type('XI_BINNING')`, `set_downsampling(CAM_BINNING)`,
     `set_imgdataformat(CAM_IMG_FORMAT)`, `disable_output_bit_packing()`
  4. `set_sensor_bit_depth`, `set_output_bit_depth`, `set_image_data_bit_depth`
  5. `set_gain_selector('XI_GAIN_SELECTOR_ANALOG_ALL')`, `set_gain(CAM_GAIN_DB)`
- `capture(tile_ra, tile_dec, exposure_s) -> Frame`: under a `threading.Lock`:
  `set_exposure(int(exposure_s * 1e6))` (xiAPI exposure is microseconds), `start_acquisition()`,
  take `utc = datetime.now(timezone.utc)`, `get_image(img, timeout=int(exposure_s * 1000) + 5000)`,
  `img.get_image_data_numpy()`, and `stop_acquisition()` in a `finally`.
  Return `Frame(data, utc, exposure_s, tile_ra=tile_ra, tile_dec=tile_dec)` with `wcs` left `None` (D4).
  Any `xiapi.Xi_error` becomes `Fault(Code.CAM_NO_FRAME, ...)`.
- `temperature_c()`: `get_sensor_board_temp()` under the same lock (the thermal watchdog calls it
  from another thread); return `None` on error.
- `close()`: `close_device()`.

Uncertain, verify against the xiAPI installed on the Jetson: the `timeout` keyword of `get_image`.
Not included (old code never called it): sensor defect correction. Ask the camera team.

No unit test (hardware only).

## Step 3: `airis/catalog.py` (new)

One class, `GaiaCatalog(chunk_dir)`, with `stars(ra, dec) -> list[Star]`.
Source: the query half of old `generateReferenceImage` (table read, box cut, magnitude to flux).
Do NOT port the image rendering, `add_psf`, binning or `.npy` saving: `render_reference` in
`detect.py` already does that.

- Chunk files are named like `airis_gaia_dr3_G8_G18_dec_-80_to_-75_ruwe1.4.fits`. Parse the band
  with the regex `dec_(-?\d+)_to_(-?\d+)`; scan `chunk_dir` once in `__init__`.
  The 5-degree band layout is inferred from ONE filename and may change (someone else owns the catalog).
- Columns used: `ra`, `dec`, `phot_g_mean_mag` (same as the old code).
- Load a band on first use (`astropy.table.Table.read(path, memmap=True)`), keep only those three
  columns as numpy arrays, and cache only the bands used by the most recent call (drop the rest).
- Selection radius `r = 1.2 * hypot(CAM_FOV_W_DEG, CAM_FOV_H_DEG) / 2` (the field roll is unknown
  on an el/az mount, so cut on the diagonal). Keep stars with `|dec - dec0| <= r` and
  `|dRA wrapped to +-180| <= r / cos(radians(|dec0| + r))`; if `|dec0| + r >= 90`, take every RA.
  The old code used a fixed RA half-width with no cos(dec) term, which loses most of the field
  near the pole. Do not copy that.
- `flux = 10 ** (-0.4 * (mag - GAIA_ZERO_POINT))`; drop non-finite magnitudes.
- No band covering the field, or no chunk directory: `Fault(Code.DET_REFERENCE_EMPTY, ...)`.

Tests (`tests/test_catalog.py`): build small FITS chunks in `tmp_path` with astropy. Cover a field
at dec -78 (RA window widens), a field straddling RA 0/360, a field spanning two bands,
and a missing band raising the Fault.

## Step 4: `airis/detect.py`, `SFFTSubtractor.subtract` only

Replace the stub body. Touch no other function in the file.

1. In a `tempfile.TemporaryDirectory()`, write `science` and `reference` as float32 FITS
   (`fits.PrimaryHDU(arr)`), each with header `SATURATE = config.SFFT_SATURATE`, `GAIN = config.SFFT_GAIN`.
2. Before writing the reference, add the noise the old code added (SExtractor inside SFFT
   probably needs a non-zero background RMS; unverified, so keep the old behaviour):
   `rng.poisson(0.56 * 9 * 10, shape)` plus `rng.normal(1, 1.5, shape).astype(int)`,
   then clip to `[0, SFFT_SATURATE]`. Use a fixed seed.
3. Call, with exactly the old parameters:

```python
Easy_CrowdedPacket.ECP(FITS_REF=ref_path, FITS_SCI=sci_path, FITS_DIFF=diff_path, FITS_Solution=None,
    ForceConv='AUTO', GKerHW=None, KerHWRatio=2.0, KerHWLimit=(2, 15), KerPolyOrder=2,
    BGPolyOrder=2, ConstPhotRatio=True, MaskSatContam=False, GAIN_KEY='GAIN',
    SATUR_KEY='SATURATE', BACK_TYPE='AUTO', BACK_VALUE=0.0, BACK_SIZE=64, BACK_FILTERSIZE=3,
    DETECT_THRESH=5.0, DETECT_MINAREA=5, DETECT_MAXAREA=0, DEBLEND_MINCONT=0.005, BACKPHOTO_TYPE='LOCAL',
    ONLY_FLAGS=None, BoundarySIZE=0.0, BACK_SIZE_SUPER=128, StarExt_iter=2, PriorBanMask=None,
    BACKEND_4SUBTRACT=self.backend, CUDA_DEVICE_4SUBTRACT=self.device, VERBOSE_LEVEL=2)
```

4. Read the difference with `fits.getdata`, `np.nan_to_num` it, check its shape equals
   `science.shape`, return it as float.
5. Any exception becomes `Fault(Code.DET_SUBTRACTION, ...)`. Keep the sfft import inside the method.

`sci_sigma` and `ref_sigma` are unused (SFFT fits its own kernel). Do NOT port `noiseMask`,
`transientFound`, `fitsToPNG`, `doOutliersExist` or `_ensure_cuda_ld_path`: `Pipeline.run`
already does thresholded detection, and the re-exec would restart the orchestrator.

Test: one test that monkeypatches a fake `sfft.EasyCrowdedPacket` module whose `ECP` writes
`sci - ref` to `FITS_DIFF`, and checks shape, finiteness and that a failing `ECP` raises the Fault.

## Step 5: `airis/airis_main.py`, `main()` only

- Remove `import airis_photo_capture as camera_mod`.
- Build `camera = XimeaCamera()` and `gaia = GaiaCatalog(config.GAIA_CHUNK_DIR)`.
- `Orchestrator(MountClient(), camera, gaia.stars, Pipeline(subtractor=SFFTSubtractor()), Downlink(sender), sender, nav)`.
- `ThermalWatchdog(read_camera=camera.temperature_c, ...)`.
- Log one WARNING at startup: no registrar is configured, so every frame will report `DET-01` (D4).

Do not change `Orchestrator` or any other function.

## Step 6: `tools/imaging_bench.py` (new, Jetson only)

Replaces the old file-dialog test. Arguments: `--ra --dec --exposure-s --out DIR`.
Capture one frame, attach a NOMINAL WCS
(`LinearWCS(ra, dec, w / 2, h / 2, config.CAM_FOV_W_DEG / w)`), fetch stars, `render_reference`,
run `SFFTSubtractor`, save science, reference and difference as FITS, and print the frame shape,
min/max and the sensor temperature. The subtraction will be misaligned until registration exists;
the tool is for checking that the camera, catalog and SFFT run at all.

## Step 7: `README.md` and `requirements.txt`

- Run section: the CUDA library path is set in the environment, not by the program:
  `export LD_LIBRARY_PATH=/usr/local/cuda-13.2/lib64:/usr/local/cuda-13.2/targets/sbsa-linux/lib:$LD_LIBRARY_PATH`
- Requirements, Jetson only: `sfft`, `cupy`, and the XIMEA `ximea` Python package (vendor installer).
  Note from the old code: sfft needs scikit-image 0.18.3, which needs numpy below 2 (1.26.4 was used).
- Layout table: add `camera.py`, `catalog.py`, `tools/imaging_bench.py`.

## Step 8 (optional, not imaging): RTDs in `main()`

`payload_hw.MAX31865(bus, device)` exists but `main()` never builds one. The old `airis_mount.py`
gives the wiring: SPI bus 1, device 0 = sensor RTD, device 1 = OBC RTD, 430 ohm reference, PT100.
Pass a `read_rtd` to `ThermalWatchdog` that returns `(sensor_c, obc_c)`; first check how the
watchdog handles a read that raises. The heater GPIO pin was never assigned in the old code,
so `set_heater` stays a no-op. Remove the "WITHOUT the sensor RTD" warning only for the part that is wired.

## Deferred (do not start)

- Registration: a plate solver behind `Registrar`, and replacing `LinearWCS`'s single-cos(dec)
  approximation with a true tangent-plane projection (corner error is about 1.75 degrees at dec -78).
- Stars brighter than G = 8 are not in the Gaia chunks, so they will be missing from the reference
  and show up as candidates. For the catalog owner.
- Repeated camera failures only log and retry; there is no escalation to safe mode.

## Done when

- `python -m pytest` passes (95 old tests plus the new catalog and SFFT tests).
- `python -c "import airis.airis_main, airis.camera, airis.catalog"` works on a machine with no
  `ximea`, `sfft` or `cupy` installed.
- `git diff --stat` shows only: `airis/config.py`, `airis/camera.py`, `airis/catalog.py`,
  `airis/detect.py`, `airis/airis_main.py`, `tools/imaging_bench.py`, `tests/test_catalog.py`,
  `tests/test_detect.py`, `README.md`, `requirements.txt`.
