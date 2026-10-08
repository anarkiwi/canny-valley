# Sled controller

Host driver `qmrdk/sled.py` (`HardwareSled`) for the rail sled: one stepper
axis moving the radar side to side along the rail, run by an Arduino
Leonardo with `radar_scan_platform.ino`.

## Hardware

| Item | Value |
|------|-------|
| Controller | Arduino Leonardo, USB id `2341:8036`, CDC serial at 9600 baud |
| Driver | A4988, full step (MS1–MS3 low), 200 steps/rev |
| Pulley | 49.5 mm diameter; step length `STEP = π · 49.5 / 200` mm (single precision) |
| Home switch | far end of the rail, pin 2 (falling edge) |
| End switch | pin 3, not wired |
| Coordinates | controller `x = 0` is 10 steps from the home switch trip; positive away from home |
| Travel | commands above 950 mm are refused |

## Line protocol

Lines are LF-terminated in both directions; replies end in CRLF.

| Event | Controller output |
|-------|-------------------|
| Port opened after reset | `boot`, `homing`, homing sequence, `ready` |
| Integer `N` in 1–950 | `moving to position N`, move, `ready` |
| `N` above 950 | `moving to position N`, `error`, `ready` |
| `0` | `moving to position 0`, homing sequence, `ready` |

The controller parses the line with `String.toInt()` and takes the absolute
value. Homing: if the switch is pressed, step off it and 100 steps further;
step toward home until the switch trips; step away until it releases and 10
steps further; approach again at 1/8 speed until it trips; call that −10 steps
and move to 0.

## Quantisation

A command `N` moves to `trunc(float32(N) / float32(STEP))` steps (AVR float
division, truncated to `long`). The step is shorter than 1 mm, so each integer
command reaches a distinct step but the position is quantised to whole mm.

`HardwareSled.move_to(x)` sends `N = round(1000 · (origin + x))` (refused before
sending outside 1–950), skips the command when it reaches the step already
held, and records the reached step. `position()` reports
`steps · STEP − origin` (m), the step actually reached, not the requested `x`;
it raises before the first move. Each command waits for the echo, then for
`ready` (or `error`, which raises `SledError`), then `settle` seconds.

## Origin

Rail position 0 is `origin` metres from the controller's `x = 0`, 50 mm by
default (`--origin`), so the whole scan stays clear of the switch. `home()`
moves to rail 0 without re-homing; `rehome()` sends `0` and runs the homing
sequence.

## Port

`HardwareSled(url, origin, settle)` opens `url` with
`serial.serial_for_url`; without one it uses `$QMRDK_SLED`, else the first
port with the Leonardo's USB id. On open it waits briefly for `homing` and,
if seen, for the `ready` that ends homing (60 s).

From a container without the device, bridge the port with socat in the
container's network namespace and point the driver at it:

```
docker run -d --name sled-bridge --network container:<id> --device /dev/ttyACMn alpine/socat \
  tcp-listen:7001,bind=127.0.0.1,reuseaddr,fork file:/dev/ttyACMn,raw,echo=0,b9600
export QMRDK_SLED=socket://127.0.0.1:7001
```

## Limitations of the sketch

* Limit switch trips during moves only set a flag; the move continues.
* Homing has no timeout: a missing or failed switch drives the sled forever.
* Non-numeric input parses as 0 and re-homes.
* There is no position query; the host tracks the last reached step.
