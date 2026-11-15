V2 control systems files for AIRIS

## Layout

| Path | What it is |
| --- | --- |
| `airis/` | Flight software: bridge, orchestrator, scheduler, pointing, nav, ADAPT/comms link, downlink, detection |
| `airis/camera.py` | The XIMEA camera behind the orchestrator's camera seam (Jetson only; settings in `config.py`) |
| `airis/catalog.py` | Gaia DR3 reference stars for a field, read from the chunk files in `GAIA_CHUNK_DIR` |
| `firmware/` | Mount controller for the Arduino Mega (`motorsworking_v5.ino`) and its change notes |
| `sim/`, `tests/` | Simulators and the test suite that runs against them |
| `tools/bridge_smoke.py` | Bench acceptance run against a real or simulated bridge |
| `tools/imaging_bench.py` | Jetson bench run of one frame through camera, catalog and SFFT (nominal WCS, no registration) |

## Install

```
pip install -r requirements.txt
```

The flight codec needs the generated ADAPT protobuf bindings (`messages/` and
`airis/telemetry/`) on `PYTHONPATH`. Generate them from the `proto` repository
the way `airis-comms/comms/sim/run_stack.sh` does (`protoc --python_out=...`).

Jetson only, for imaging: `sfft` (image subtraction), `cupy` (its GPU backend)
and the XIMEA `ximea` Python package, which comes from the vendor installer, not
PyPI. From the old flight code: `sfft` needs scikit-image 0.18.3, which needs
numpy below 2 (1.26.4 was used). The tests do not need any of these.

## Run

The firmware and the bridge must be flashed and deployed together: the serial
checksum is a 4-digit CRC-16 since firmware v5.1, and an older build on either
side rejects every line.

SFFT runs on the GPU through cupy. The CUDA library path is set in the
environment, not by the program:

```
export LD_LIBRARY_PATH=/usr/local/cuda-13.2/lib64:/usr/local/cuda-13.2/targets/sbsa-linux/lib:$LD_LIBRARY_PATH
```

```
python -m airis.airis_bridge            # owns the serial port; serves /tmp/airis_bridge.sock
python -m airis.airis_main              # orchestrator (use --codec json without the protobufs)
python -m tools.bridge_smoke            # bench check: sun, cal, move, busy
python -m tools.imaging_bench --ra RA --dec DEC --exposure-s 4 --out DIR   # Jetson: camera, catalog, SFFT
python -m pytest -q                     # tests (about 3 minutes)
```

Start both from the same working directory: `moves.csv`, `candidates.csv`,
`downlink_queue/` and `img_saves/` are written relative to it.

The bridge homes the mount only after it has received a sun position, which the
orchestrator pushes at startup and every 60 s.

## Links

| Port | Transport | Direction | Carries |
| --- | --- | --- | --- |
| 5005, 5006 | TCP, sync word + length | ADAPT to controls | alerts, maps, comms directives, navigation |
| 9001 | UDP, one envelope per datagram | comms server to controls | `pointing_cmd`, `safe_mode_exit_cmd` |
| 9003 | UDP, one envelope per datagram | controls to comms server | events, image chunks |
| 31250 | UDP | Trimble to controls | NMEA |
