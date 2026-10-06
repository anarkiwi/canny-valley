# USB driver

Host driver for the QM-RDK board over USB, implementing
[protocol.md](protocol.md).

## Layers

| Module | Role |
|--------|------|
| `qmrdk/transport.py` | The only module that opens USB resources. Discovery (vendor ID `0x2012`, any product ID, `*IDN?` of the form `Quonset Microwave,QM4004,<serial>,<firmware>`, optional serial number), `write` / `query` with LF termination and per-call timeout, the §3.7 error policy (`SYST:ERR?` drained after every setter, non-zero raises `ScpiError` with code and text, `-500 Power on` consumed), frame flush (`CAPT:FRAM?` until `Not Ready`, in place of device clear), reopen by serial number after re-enumeration. Refuses every command outside the documented set (protocol §3.8), including `CAPT:STRE`. |
| `qmrdk/device.py` | `Device`: typed commands over a `Transport`. `identify`, `settings` (read-back `Sweep`, reference divider, RF, lock), `configure` with host-side validation, `start` / `stop` / `rf`, `temperature`, `status`, `errors`, `reset`, `save` / `recall` / `restore_factory`, `capture` / `capture_many`, guarded `scpi` passthrough. |
| `qmrdk/radar.py` | `UsbRadar`: the `Radar` protocol (`sweep`, `capture(n)`) over a `Device`, used by scans and calibration. |
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

## Capture

`capture(n)` follows §3.4: `CAPT:FRAM n`, the first `CAPT:FRAM?` with a
timeout of the acquisition time plus a margin, every chunk validated (hex,
exactly 31 samples except the last, `Not Ready` before the end is an error).
A failed frame is flushed, the error queue drained and the frame retried. A
lost session is reopened and the sweep re-applied, since the board may have
rebooted. Capture is refused unless the sweep was started and locked in this
session. `capture_many(n, count)` returns the frames and the host time of
each `CAPT:FRAM`, with a progress bar.

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
`board.stuck_unlocked` holds `FREQ:LOCK?` at 0.

Where the protocol leaves behaviour open the simulator assumes: `SYST:PRES`
restores memory location 0 without rebooting, `SYST:STAT?` reads
`0,"Operational"`, `SYST:MODNUM?` reads `QM4004`.

## CLI

| Command | Effect | RF at exit |
|---------|--------|------------|
| `qmrdk list` | resources and `*IDN?` | off |
| `qmrdk info` | identity, settings, lock, temperature, status, error queue (JSON) | off |
| `qmrdk set --f0 --f1 --ramp-time --type` | configure, start, print read-back | on, sweeping as set |
| `qmrdk rf on\|off` | start (locked) or stop the sweep | as requested |
| `qmrdk scpi CMD [--force]` | guarded raw command or query | off |
| `qmrdk capture --frames K --n N --out rec.npz` | configure (sweep options as `set`) and record | off |

Every command takes `--sim` (process-wide simulated board),
`--resource NAME` and `--serial N`. `capture` writes a native recording
(§5) with `t_host` and, in `extra`, the `*IDN?` fields, resource, reference
divider and sample rate.

## Tests

Tests run against the simulated board. `tests/test_hardware.py` also runs
on the board with `python -m pytest -m hardware` (skipped when none is
attached; the default run deselects it) and writes results to
`artifacts/hardware/`. `test_type1_sweeps_down` answers protocol open
question 5: it passes if a type 1 frame mirrors about the end of its
up-ramp as the first triangle of an AUTO frame does.
