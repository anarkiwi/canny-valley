# Implementation plan: Linux/USB/Python replacement for the QM-RDK software

Replaces the two vendor components that need Windows or MATLAB:

| Vendor component | Function | Replacement |
|------------------|----------|-------------|
| QM-RDK GUI (Windows, VISA) | configure sweep, RF on/off, single and continuous capture, save/load, raw / spectrum / range / Doppler / time-history plots, clutter cancellation, averaging | `qmrdk` CLI and live viewer |
| MATLAB scripts (VISA) | scripted capture and plotting, plotting of saved CSV | `qmrdk` Python API |

Specifications: [protocol.md](protocol.md),
[signal-processing.md](signal-processing.md). Beyond the vendor functions
the plan adds synthetic aperture imaging using the kit's motorised sled.

Bluetooth is out of scope: it shares the radar's 2.4 GHz band and
interferes with it. USB is the only control and data path.

## Constraints that shape the design

* The board is a USBTMC instrument speaking SCPI-style ASCII; no firmware
  change is needed and none is planned.
* Capture is frame based: at most 4096 samples (186 ms at the measured
  21 977 Hz) per frame, read back 31 samples per query, 0.73 s per frame end
  to end. There is no USB streaming, so time-history products are built from
  gapped frames.
* The firmware restarts the sweep for each capture, so USB frames are
  sweep-synchronous. Continuous recordings from the audio tap have no sync
  and use the estimator in the signal-processing spec.
* The vendor documentation is wrong in several places (sample rate, RF
  power-up state, error codes, accepted parameter forms). The protocol spec
  records what the board does; its remaining open questions are closed in
  phase 0 before any device-layer code is written.

## Dependencies

| Need | Library |
|------|---------|
| USBTMC transport | `pyvisa` + `pyvisa-py` (`pyusb`, libusb-1.0) |
| Arrays, FFT, windows, STFT | `numpy`, `scipy` |
| Compiled kernels where NumPy cannot vectorise (CFAR) | `numba` |
| Live display | `pyqtgraph` (Qt) |
| Static plots / export | `matplotlib` |
| CLI, progress | `argparse`, `tqdm` |
| Tests and checks | `pytest`, `pytest-xdist`, `pytest-cov`, `black`, `pylint` |

Supported platform: Linux, currently maintained CPython versions only.

## Package layout

```
qmrdk/
  transport.py   open/close, discovery, write/query/clear over PyVISA; the only module that touches USB
  device.py      typed command layer: one method per command in protocol §3, range validation, error-queue policy, RF-off-on-exit
  capture.py     frame acquisition state machine (protocol §3.4), hex decoding, recording loop
  recording.py   .npz recordings, vendor CSV import/export (protocol §5)
  sim.py         simulated device: SCPI state machine + synthetic IF source (signal-processing §12) behind the transport interface
  dsp/
    convert.py   codes → volts, level scaling
    segment.py   sweep segmentation
    range.py     range profile, vendor-equivalent spectrum
    clutter.py   cancellers, background subtraction
    doppler.py   CW spectrum, STFT, range–Doppler
    detect.py    CFAR, peak interpolation
    sar.py       phase history, backprojection
  sled.py        sled interface (`home`, `move_to`, `position`) and its simulated implementation
  scan.py        SAR scan sequencer: move, settle, capture, record position
  viewer.py      live and replay display
  cli.py         entry point
tools/
  probe.py       phase 0 bring-up probe (kept as the hardware diagnostic)
  udev/          udev rule
tests/
docker/
docs/
```

`transport.py` defines the interface (`write`, `query`, `clear`, `close`)
that both the PyVISA transport and `sim.py` implement; everything above it
is hardware independent.

## CLI

| Command | Purpose |
|---------|---------|
| `qmrdk list` | discover boards, print `*IDN?` |
| `qmrdk info` | all readable state: sweep settings, lock, RF, temperature, status, error queue |
| `qmrdk set` | start/stop frequency, ramp time, sweep type, reference divider (validated per protocol §3.3) |
| `qmrdk rf on\|off` | `SWEEP:START` / `SWEEP:STOP` |
| `qmrdk capture` | N captures at an interval into a recording, with progress bar |
| `qmrdk view` | live display: raw, spectrum, range, Doppler, range–time, Doppler–time; clutter and averaging toggles |
| `qmrdk replay` | same displays from a recording or vendor CSV |
| `qmrdk export` | recording → vendor CSV / PNG |
| `qmrdk sar scan` | step the sled across the aperture, one capture per position, into a recording |
| `qmrdk sar image` | form and display / export an image from a scan recording |
| `qmrdk scpi` | send a raw command or query (diagnostic; refuses undocumented commands, protocol §3.8) |

All commands accept `--sim` to run against the simulated device.

## Phases

Each phase is one PR, merged when CI is green. Work items inside a phase are
sized for independent execution.

### Phase 0 — project scaffold and hardware bring-up

Scaffold:

* `pyproject.toml`, package skeleton, `black` / `pylint` configuration.
* Docker image (multi-stage: system and Python dependencies in a cached base
  stage, source in the final stage) used for both development and CI.
* GitHub Actions workflow running format, lint and tests in that image;
  Dependabot for pip, Docker and Actions.
* udev rule and USB pass-through instructions.

Bring-up (`tools/probe.py`, needs the board attached):

* Dump USB descriptors.
* Run the connect sequence and record every documented read-only query and
  its raw response bytes.
* Run the timing and behaviour tests that answer protocol open questions
  1–11, including a static-scene capture set at several ramp times for the
  segmentation questions, and the IF step response at a turnaround for the
  guard interval `Ng`.
* Write a machine-readable report into gitignored `artifacts/`.

Exit: protocol.md updated with every A tag resolved; signal-processing.md
updated with the segmentation source and `Ng`.

The probe sends only documented commands, never writes memory location 0,
and leaves the RF off.

### Phase 1 — transport, device layer, simulator

* `transport.py`, `device.py` per the protocol spec.
* `sim.py` SCPI state machine: settings, ranges and defaults, error queue,
  status registers, sweep side effects, `Not Ready` timing, 31-sample
  paging. Its behaviour for formerly open questions follows the phase 0
  report.
* CLI: `list`, `info`, `set`, `rf`, `scpi`.

Exit: device-layer tests pass against the simulator; the same test module
passes against hardware when run with a hardware marker.

Status: delivered as `transport.py`, `device.py` and `sim/scpi.py`
([driver.md](driver.md)). The hardware run of `tests/test_hardware.py` is
pending.

### Phase 2 — capture and recordings

* `capture.py` state machine with validation, timeout and recovery.
* `recording.py` formats; round-trip with a vendor-GUI CSV fixture generated
  by the project (no vendor files in the repository).
* Synthetic IF source in `sim.py`.
* CLI: `capture`, `export`.

Exit: simulated multi-capture recording round-trips bit-exact; fault
injection tests (truncated chunk, non-hex data, `Not Ready` beyond deadline,
disconnect) recover or fail cleanly with RF off.

Status: capture (in `device.py`), the synthetic IF source and `capture` are
delivered and meet the exit criteria; vendor CSV and `export` are pending.

### Phase 3 — signal processing

* `dsp/` modules per the signal-processing spec, in dependency order:
  convert → segment → range → clutter → doppler → detect.
* Every row of the verification table in signal-processing §12 is a test.

Exit: all verification rows pass on synthetic data; segmentation and range
calibration confirmed on phase 0 hardware recordings.

### Phase 4 — viewer

* `viewer.py`: acquisition in a worker thread feeding a bounded queue; the
  display never blocks capture. Same pipeline objects for live and replay.
* Controls matching the vendor GUI functions (sweep settings, RF, sample
  count, single / continuous capture, clutter cancellation, averaging, axis
  selection range / frequency / velocity).
* CLI: `view`, `replay`.

Exit: live operation against the simulator in CI (offscreen Qt platform) and
against hardware manually.

### Phase 5 — SAR

* `dsp/sar.py` per signal-processing §11, verified on synthetic scans.
* `sled.py` interface with a simulated sled; the driver for the kit's custom
  sled controller is documented in `docs/sled.md`.
* `scan.py` sequencer and CLI `sar scan`, `sar image`; scan recordings add a
  per-capture `x_pos` field.

Exit: synthetic verification rows pass; a hardware scan of a single strong
reflector focuses at its surveyed position.

### Phase 6 — documentation and release

* README kept to a summary and usage; details in `docs/` (installation and
  USB access, CLI reference, API reference, hardware notes).
* Tagged release, wheel build in CI.

## Testing

* Unit and integration tests run entirely against `sim.py`; coverage above
  85 % of `qmrdk/`, measured in CI.
* Hardware tests carry a `hardware` marker, are skipped in CI and run from
  the development container with the board passed through.
* Full runs use `pytest -n auto` with BLAS thread counts pinned
  (`OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`).
* The simulator's synthetic source and the DSP are verified against closed
  forms, not against each other's outputs.

## Risks

| Risk | Mitigation |
|------|------------|
| Vendor documentation and scripts disagree (RF default state, capture trigger behaviour) | phase 0 gate; simulator written after the report |
| Capture is asynchronous to the sweep | segmentation estimator specified and testable without hardware |
| Low frame rate from 31-sample paging limits time-history products | measure in phase 0; sample count per capture is the user-facing trade-off |
| Transmitter left on after a crash | RF-off on every exit path in `device.py`; `qmrdk rf off` always available |
| Kernel `usbtmc` driver holds the interface | udev rule and documented unbind; confirmed in phase 0 |
| SAR needs ramp direction and sub-sample alignment across positions | firmware timing if phase 0 finds it; otherwise the sharpness test and fractional alignment in the spec |
| Board operates in the 2.4 GHz ISM band alongside Wi-Fi and Bluetooth | `info` reports lock state; ambient check procedure (receive with TX terminated) documented |
