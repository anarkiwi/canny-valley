# Sled controller

Host driver `qmrdk/sled.py` (`HardwareSled`) for the rail sled: one stepper
axis moving the radar side to side along the rail, run by an Arduino
Leonardo. The controller runs either this repository's firmware
(`firmware/sled`) or the legacy `radar_scan_platform.ino` sketch; the driver
detects which on connect.

## Hardware

| Item | Value |
|------|-------|
| Controller | Arduino Leonardo (ATmega32u4), USB id `2341:8036`, CDC serial (opened at 9600 baud; the rate is ignored by USB CDC) |
| Driver | A4988: STEP 10, DIR 11 (low moves towards home), SLEEP 9, RESET 8, MS1–MS3 5–7, EN 4 (active low) |
| Motor | 200 full steps/rev |
| Pulley | 49.5 mm diameter; full step `π · 49.5 / 200` mm = 777544 nm |
| Home switch | far end of the rail, pin 2, active low |
| End switch | pin 3, active low, not wired |
| Coordinates | `x = 0` is 10 full steps from the home switch trip; positive away from home |
| Travel | 950 mm: 1221 full steps |

## Firmware

`firmware/sled/sled.ino` wires the pins, AccelStepper and USB serial to
`sled_core.h`, which holds the line parser and the motion and homing state
machine as Arduino-independent logic. The motor runs non-blocking
(`AccelStepper::run()` every loop), so every command is served during motion.
The firmware never moves on its own: there is no homing at boot.

### Protocol

ASCII lines terminated by LF (CR ignored, empty lines ignored, at most 24
characters); replies end in CRLF. Positions are in steps of the current
microstep setting, `x = 0` to `max = 1221 · microstep`. Speeds are
steps/s and accelerations steps/s², both of the current microstep setting.

| Command | Reply |
|---------|-------|
| (port opened: DTR rises) | banner `id qmrdk-sled <version> <full step nm> <microstep>` |
| `id?` | the banner line |
| `pos?` | `pos <steps>` |
| `status?` | `status homed=0\|1 moving=0\|1 homing=0\|1 pos=<steps> target=<steps> home=0\|1 end=0\|1 microstep=<n> speed=<steps/s> accel=<steps/s²> max=<steps>` |
| `home` | homing sequence, then `ok 0` |
| `move <steps>` | absolute move; `ok <steps>` when it completes |
| `stop` | decelerate to rest (aborts a move or homing), then `ok <steps>`; when idle, `ok <steps>` at once |
| `speed <1–4000>` | `speed <n>` |
| `accel <1–100000>` | `accel <n>` |
| `microstep <1\|2\|4\|8\|16>` | sets MS1–MS3 (A4988 table), rescales position (rounded) and soft limit; `microstep <n>` |

Errors, none of which moves the motor: `err unknown` (unknown command,
malformed or extra argument, overlong line), `err range` (argument outside
its range or `move` outside `[0, max]`), `err not homed` (`move` before
homing), `err busy` (`move`, `home` or `microstep` while moving or homing).
`stop` replaces the pending `ok` of the move or homing it interrupts, so each
motion command ends in exactly one `ok` or `err` line.

Defaults: microstep 1 (all MS pins low, as on the wired hardware), speed 500,
accel 1000. `microstep` leaves speed and accel numerically unchanged, so the
physical speed drops by the factor; send `speed` and `accel` after it to keep
it.

### Homing

1. If the home switch is pressed, move away until it releases, then 10 full
   steps further.
2. Approach at `speed` until it trips; stop dead.
3. Back off at `speed / 2` until it releases, then 10 full steps further.
4. Approach at `speed / 16` until it trips; that point is −10 full steps.
5. Move to 0, set homed, reply `ok 0`.

Each search is bounded: the approach to 1321 full steps (travel + 100), the back-offs
to 100 and the slow approach to 200; exceeding a bound stops with
`err home timeout`. The end switch aborts homing with `err limit`.

### Limit switches

Outside homing, the home or end switch pressed while homed or moving stops
the motor at once, clears homed and replies `err limit`; `move` then needs
`home`. Both switch inputs use the internal pull-up, so the unwired end
switch reads released.

### Build, test and flash

`docker/firmware.Dockerfile` pins arduino-cli, the `arduino:avr` core,
AccelStepper, doctest, clang-format and gcovr; `firmware/Makefile` runs
them.

```
docker build -f docker/firmware.Dockerfile -t qmrdk-firmware .
docker run --rm qmrdk-firmware make format     # clang-format (LLVM) check
docker run --rm qmrdk-firmware make test       # core unit tests on the host, coverage
docker run --rm qmrdk-firmware make compile    # sled.ino for arduino:avr:leonardo
```

The core tests (`firmware/sled/test/test_core.cpp`) drive the state machine
against a simulated motor and rail; `firmware/sled/test/protocol.txt` is a
request/reply transcript that both they and the host driver's fake
controller (`tests/test_sled.py`) must reproduce.

Flashing is an operator action with the sled attached. Stop anything holding
the port (e.g. the socat bridge below) first. The Leonardo's upload resets it
into its bootloader, which enumerates as a new ttyACM node, so pass the
ttyACM device class rather than one node. `PORT` is required; other USB
serial adapters also enumerate as ttyACM, so find the Leonardo by its USB id
(`grep -l 8036 /sys/class/tty/ttyACM*/device/../idProduct`):

```
docker run --rm --device-cgroup-rule='c 166:* rmw' -v /dev:/dev qmrdk-firmware \
  make compile upload PORT=/dev/ttyACMn
```

## Host driver

`HardwareSled(url, origin, settle)` opens `url` with
`serial.serial_for_url`; without one it uses `$QMRDK_SLED`, else the first
port with the Leonardo's USB id. It sends nothing until it knows the
controller: the legacy sketch parses any line as an integer, so a probe such
as `id?` would re-home it. It reads for up to 2 s:

* the firmware banner: firmware protocol. It sends `status?`, adopts the
  step length (banner) and microstep and soft limit (status), and sends
  `home` unless already homed.
* `homing` (legacy sketch after power-up): waits for the `ready` that ends
  homing (60 s); legacy protocol.
* nothing: legacy protocol.

Rail position 0 is `origin` metres from the controller's `x = 0`, 50 mm by
default (`--origin`), so the whole scan stays clear of the switch. `home()`
moves to rail 0 without re-homing; `rehome()` runs the homing sequence.
`position()` reports `steps · step − origin` (m), the step actually reached,
not the requested `x`.

With the firmware, `move_to(x)` sends `move N` for the nearest step
`N = round((origin + x) / step)` (refused before sending outside
`[0, max]`), skipping it when `N` is the step already held; it waits for
`ok` (an `err` raises `SledError`) and then `settle` seconds. Positions are
exact steps, with no millimetre quantisation.

From a container without the device, bridge the port with socat in the
container's network namespace and point the driver at it:

```
docker run -d --name sled-bridge --network container:<id> --device /dev/ttyACMn alpine/socat \
  tcp-listen:7001,bind=127.0.0.1,reuseaddr,fork file:/dev/ttyACMn,raw,echo=0,b9600
export QMRDK_SLED=socket://127.0.0.1:7001
```

Each TCP connection opens the tty afresh, so the firmware banner is sent per
connection.

## Legacy sketch

The third-party `radar_scan_platform.ino` (not part of this repository).

### Line protocol

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

### Quantisation

A command `N` moves to `trunc(float32(N) / float32(STEP))` steps (AVR float
division, truncated to `long`). The step is shorter than 1 mm, so each integer
command reaches a distinct step but the position is quantised to whole mm.

`HardwareSled.move_to(x)` sends `N = round(1000 · (origin + x))` (refused before
sending outside 1–950), skips the command when it reaches the step already
held, and records the reached step. `position()` reports
`steps · STEP − origin` (m), the step actually reached, not the requested `x`;
it raises before the first move. Each command waits for the echo, then for
`ready` (or `error`, which raises `SledError`), then `settle` seconds.
`rehome()` sends `0`.

### Limitations

* Limit switch trips during moves only set a flag; the move continues.
* Homing has no timeout: a missing or failed switch drives the sled forever.
* Non-numeric input parses as 0 and re-homes.
* There is no position query; the host tracks the last reached step.
