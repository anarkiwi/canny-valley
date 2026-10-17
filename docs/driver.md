# USB driver

Host driver for the QM-RDK board over USB, implementing
[protocol.md](protocol.md).

## Layers

| Module | Role |
|--------|------|
| `qmrdk/transport.py` | The only module that opens USB resources. Discovery (vendor ID `0x2012`, any product ID, `*IDN?` of the form `Quonset Microwave,QM4004,<serial>,<firmware>`, optional serial number), `write` / `query` with LF termination and per-call timeout, the §3.7 error policy (`SYST:ERR?` drained after every setter, non-zero raises `ScpiError` with code and text, `-500 Power on` consumed), frame flush (`CAPT:FRAM?` until `Not Ready`, in place of device clear), reopen by serial number after re-enumeration. Refuses every command outside the documented set (protocol §3.8), including `CAPT:STRE`; the CV-only commands (protocol §6.5) are allowed only when `Idn.cv` (firmware field starting with `CV`). `query_block` reads one `#6dddddd` binary block (protocol §6.6). |
| `qmrdk/device.py` | `Device`: typed commands over a `Transport`. `identify`, `settings` (read-back `Sweep`, reference divider, RF, lock), `configure` with host-side validation, `start` / `stop` / `rf`, `temperature`, `status`, `errors`, `reset`, `save` / `recall` / `restore_factory`, `capture` / `capture_many`, guarded `scpi` passthrough. On CV firmware also `limits` (`Limits`, from `SWEEP:LIM?`), the exact `fs` and `timing()` (`Timing`). |
| `qmrdk/radar.py` | `UsbRadar`: the `Radar` protocol (`sweep`, `capture(n)`) over a `Device`, used by scans and calibration; `timing` is the device `Timing` of the configured sweep on CV firmware, None on stock. `ManualSled`: the `Sled` protocol by hand; each move prints the position from home (m, cm, mm) and the step on stderr and waits for Enter (`qmrdk sar scan --manual`). |
| `qmrdk/sled.py` | `HardwareSled`: the `Sled` protocol over the sled controller's serial line protocol ([sled.md](sled.md)). |
| `qmrdk/sim/scpi.py` | `SimBoard`, `SimResource`, `SimManager`: the firmware's SCPI state machine behind the PyVISA resource and resource-manager interfaces, so the layers above run unchanged against it. |

Errors derive from `DeviceError`: `ScpiError` (board error queue),
`ConfigError` (refused before sending), `LockError`, `FrameError`,
`DeviceTimeout`, `Disconnected`, `ForbiddenCommand`.

## Configuration

`configure(Sweep)` checks, before anything is sent: frequencies in
2.4–2.5 GHz, stop above start (except CW), an integer ramp time of
1–65536 ms, and ramp time within `T_max` (§3.3) at the smallest reference
divider that allows it, at most 256. `Sweep.kind` selects the type by number:
`ramp` 0, `tri` 1, `triangle` 2 (AUTO), `cw` 3. It then runs the §4 sequence,
writes the divider only when it changes, sends `SWEEP:START`, waits for
`FREQ:LOCK?` to read 1 and keeps the read-back sweep in `Device.sweep`.

On CV firmware (protocol §6) the checks use the board's `Limits`: frequencies
in `f_min`–`f_max` (2.25–2.5 GHz), stop above start (except CW), a ramp time
of at most 65536 ms and at least 4 samples (`round(T fs) >= 4`), a slope
`B fs / Nr` of at least `slope_min`, and `saw` (type 4) as a fifth kind.
`Sweep.dwell` (s) applies to `saw` only: at least 4 samples, or 0 to keep the
board's dwell; a dwell on any other kind, or `saw` on stock firmware, is a
`ConfigError`. The sequence is protocol §6.9: frequencies with 9 decimals,
the ramp time in decimal ms, `SWEEP:DWEL` for a sawtooth with a dwell, no
reference divider. Every `settings()` read refreshes `Device.fs` (exactly
`F_REF / R` from `CAPT:RATE?`) and `Device.limits`, and returns `Settings`
with the device timing `fs`, `n0`, `first_up` (`SWEEP:ORIG?`; None on stock
firmware, `Settings.timing` combines them) and `ref_div` 1.
`Device.timing()` reads the timing of the current sweep.

## Capture

`capture(n)` follows §3.4: `CAPT:FRAM n`, the first `CAPT:FRAM?` with a
timeout of the acquisition time plus a margin, every chunk validated (hex,
exactly 31 samples except the last, `Not Ready` before the end is an error).
A failed frame is flushed, the error queue drained and the frame retried. A
lost session is reopened and the sweep re-applied, since the board may have
rebooted. Capture is refused unless the sweep was started and locked in this
session. `capture_many(n, count)` returns the frames and the host time of
each `CAPT:FRAM`, with a progress bar; optionally frame starts at least
`interval` apart and the board temperature before each frame.

On CV firmware `capture(n)` is a stream (protocol §6.8) of any length
`n >= 1`: `CAPT:CONT n`, then `CAPT:BLOC?` until `n` samples, each block
checked by `decode_block` (sequence, first sample, count, OVERFLOW, END;
`FrameError` otherwise). The first read waits for the alignment latency plus
one block time plus the margin, later reads one block time plus the margin.
A failed stream is aborted (`CAPT:ABOR`), the error queue drained and the
stream retried. A tqdm bar shows streams longer than 8 blocks. On connect
`CAPT:ABOR` precedes the frame flush.

`Transport.query_block` writes the query, makes one `read_raw` and reads the
rest of the header and payload with `read_bytes(count,
break_on_termchar=False)`, so a payload byte equal to LF does not end the
read; pyvisa-py over USBTMC assembles a block spanning several transfers. A
header other than `#6dddddd`, a short block or a missing final LF raises
`FrameError`.

## Safety

The board transmits from power-up. RF is turned off (`SWEEP:STOP`, then
`POWE:RF?` must read 0) when:

* a `Device` closes, including on context-manager exit; a lost session is
  reopened first, because a board that rebooted comes back transmitting;
* configuration, lock wait or capture fails;
* SIGINT or SIGTERM arrives while a `Device` is open; the handler stops RF
  on every open board, then defers to the previous handler (SIGINT raises
  `KeyboardInterrupt`; a default SIGTERM exits with status 143);
* the interpreter exits with a board still open (`atexit`).

`Device.leave_rf_on = True` skips the stop on close only. `reset()` sends
`*RST`, waits for the session to drop, reopens the board and stops the RF it
powers up with. `*SAV 0` and `SYST:REST` need `force=True`; `*RST` goes
through `reset()`.

## Simulated board

`SimBoard(serial, firmware, source, reboot_s)` keeps the settings with their
ranges, power-up defaults and read-back formats, the 10-deep error queue
(`-500` first after power-up, `-350` on overflow), `*ESR?` / `*STB?` /
`*ESE` / `*SRE` / `*CLS`, `POWE:RF`, the lock rule
`slope >= f_ref^2 / (div * 2^25)`, the stop on `SWEEP:TYPE`, memory
locations, and `*RST` as a reboot that invalidates open resources until it
re-enumerates. A first `CAPT:FRAM?` whose timeout is shorter than the
acquisition time times out. Frames come from `source(n, sweep)`, by default
the board model of [simulation.md](simulation.md) on the built-in `single`
scene; types 0 and 1 hold the frequency after one ramp or triangle.

Faults: `board.inject(...)` applies one per subsequent `CAPT:FRAM`:
`nonhex`, `truncate` (one sample short), `not_ready`, `stall` (first read
times out), `disconnect` (reboot), `unplug` (never returns);
`board.stuck_unlocked` holds `FREQ:LOCK?` at 0. `temperature` is a constant
or a function of the number of frames captured; the default source
synthesises its `hardware` (§4.1 of simulation.md) at that temperature.

`SimBoard(firmware="CV1.0.0")` (any firmware starting with `CV`) emulates
protocol §6: RF off at power-up, the §6.2 ranges with error 201, the Q24
and whole-sample quantisation of §6.3 in every read-back, `SWEEP:START`
refused for a zero step, `SWEEP:ORIG?`, `SWEEP:LIM?`, `CAPT:RATE`,
`SWEEP:DWEL`, `POWE:RF:TIME`, `CAPT:CONT` / `CAPT:BLOC?` / `CAPT:ABOR` and
`CAPT:FRAM` with the alignment latency of §6.7, and none of the stock-only
commands. Its source is called `source(n, sweep, timing)` with the device
timing `(fs, n0, first_up)`; the default source runs the board model at
that timing (simulation.md §5). The `overflow` fault drops the block before
the second; `truncate` and `nonhex` corrupt the first block's count and
header. `SimResource` implements `read`, `read_raw` (up to `chunk_size`
bytes) and `read_bytes` (exactly `count` bytes, unmodified) as pyvisa-py over
USBTMC does.

Where the protocol leaves behaviour open the simulator assumes: `SYST:PRES`
restores memory location 0 without rebooting, `SYST:STAT?` reads
`0,"Operational"`, `SYST:MODNUM?` reads `QM4004`.

## CLI

| Command | Effect | RF at exit |
|---------|--------|------------|
| `qmrdk list` | resources and `*IDN?` | off |
| `qmrdk info` | identity, settings, lock, temperature, status, error queue (JSON) | off |
| `qmrdk set --f0 --f1 --ramp-time --type [--dwell MS]` | configure, start, print read-back | on, sweeping as set |
| `qmrdk rf on\|off` | start (locked) or stop the sweep | as requested |
| `qmrdk scpi CMD [--force]` | guarded raw command or query | off |
| `qmrdk capture --frames K --n N --out rec.npz [--interval S] [--temperature]` | configure (sweep options as `set`) and record | off |
| `qmrdk sar scan --out scan.npz [--sled URL] [--origin M] [--settle S] [--manual]` | step the sled across the aperture, one capture per position | off |
| `qmrdk sar image scan.npz --out img.png [--cal cal.json] [--extent X0 X1 Y0 Y1] [--min-range 3] [--aperture-window hann] [--background mean\|none] [--reference ref.npz]` | backprojection image (signal-processing §11); `--reference` subtracts a scan at the same positions (change image) | n/a |
| `qmrdk drift rec.npz --cal cal.json [--lines 3] [--out drift.png] [--report drift.json]` | phase drift of a static capture (signal-processing §9.1) | n/a |

Every command takes `--sim` (process-wide simulated board),
`--resource NAME` and `--serial N`. `--type saw` and `--dwell` (ms, 0 keeps
the board's) need CV firmware; `--f0`, `--f1` and `--ramp-time` are checked
against the board's limits there. `info` adds the device timing and limits
on CV firmware. `capture` writes a native recording
(§5) with `t_host` and, in `extra`, the `*IDN?` fields, resource, reference
divider and sample rate (the exact `fs` on CV firmware, plus `timing` =
`{fs, n0, first_up}`). `sar scan` records `fs` and `timing` the same way;
`sar image` and `drift` apply a recorded `timing` to the calibration
(`Calibration.with_timing`). `--interval` sets the minimum time between frame
starts; `--temperature` reads `SYST:TEMP?` before each frame into the
recording's `temperature`.

## Tests

Tests run against the simulated board. `tests/test_hardware.py` also runs
on the board with `python -m pytest -m hardware` (skipped when none is
attached; the default run deselects it) and writes results to
`artifacts/hardware/`. `test_type1_sweeps_down` answers protocol open
question 5: it passes if a type 1 frame mirrors about the end of its
up-ramp as the first triangle of an AUTO frame does.
