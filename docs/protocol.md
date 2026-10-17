# QM-RDK USB control protocol

Host-side specification for controlling the Quonset Microwave QM-RDK radar
board over USB from Linux.

Bluetooth (RFCOMM serial, `CAPTure:STREam`) is out of scope: the board's
Bluetooth radio shares the 2.4 GHz band the radar transmits in, so the link
interferes with the measurement. `CAPT:STRE` is never sent.

## Sources and confidence

Every statement is tagged with where it comes from:

| Tag | Source |
|-----|--------|
| M | [QM-RDK User Manual rev 1.2.0](https://www.quonsetmicrowave.com/v/vspfiles/Manuals/QM-RDKIT/QM-RDK%20User%20Manual%20REV%201.2.0.pdf) |
| S | [Vendor MATLAB scripts](https://www.quonsetmicrowave.com/v/vspfiles/Software/QM-RDKIT/RDK_Matlab_Scripts_v2.6.0.zip) (`RDK_CollectAndPlot_*.m`, `RDK_Plot_DataFile.m`) |
| H | Observed on hardware |
| A | Assumption, not yet confirmed on hardware; see [Open questions](#open-questions) |

Items tagged H were observed on a board (USB ID `2012:0013`, firmware
V1.1.0) with `tools/probe.py`. Items tagged A must be resolved by the bring-up probe (implementation plan, phase 0)
and this document updated before the device layer is written.

## 1. Physical and transport layer

| Property | Value | Tag |
|----------|-------|-----|
| Connector | USB Micro-B (J2), bus powered, 5 V / 0.5 A typical | M |
| Device class | USBTMC, USB488 subclass (interface class `0xFE`, subclass `0x03`, protocol `0x01`) | H |
| Speed / endpoints | full speed; bulk OUT `0x01`, bulk IN `0x81` (64-byte packets), interrupt IN `0x82` | H |
| Vendor ID | `0x2012` | S |
| Product ID | `0x0013` observed (H); `0x0017` in the vendor script (S). The host library matches any PID under the VID |
| Product string | `QM4004` | H |
| Serial string | Decimal serial number, zero-padded to 4 digits; V1.1.0 firmware never loads it from the EEPROM, so every board reports `0021` after power-up ([hardware.md](hardware.md) §4.4) | S, H |
| VISA resource | `USB0::0x2012::<pid>::<serial %04d>::INSTR` | S, H |

The device is a standard USBTMC instrument, so no vendor driver is needed.
Messages travel on one bulk-OUT and one bulk-IN endpoint with USBTMC framing:

* Host to device: `DEV_DEP_MSG_OUT` (MsgID 1) carrying one program message,
  EOM set.
* Device to host: host sends `REQUEST_DEV_DEP_MSG_IN` (MsgID 2), device
  answers with `DEV_DEP_MSG_IN` carrying one response message.
* 12-byte header: MsgID, bTag, ~bTag, reserved, 32-bit little-endian transfer
  size, attribute byte, 3 reserved bytes; payload padded to a 4-byte boundary.
* Device clear uses the USBTMC `INITIATE_CLEAR` / `CHECK_CLEAR_STATUS` control
  requests. On the device it aborts pending operations, resets the parser and
  empties the output buffer (M). pyvisa-py does not implement it
  (`VI_ERROR_NSUP_OPER`, H); the host instead discards pending frame data by
  reading `CAPT:FRAM?` until `Not Ready`.

USBTMC framing is delegated to PyVISA with the pyvisa-py backend (pyusb /
libusb); it is not reimplemented.

### Linux access

* udev rule granting access without root:
  `SUBSYSTEM=="usb", ATTR{idVendor}=="2012", MODE="0660", TAG+="uaccess"`.
* The in-kernel `usbtmc` driver binds the interface on plug-in and exposes
  `/dev/usbtmcN`. The libusb path must detach it or it must be blocked for
  this VID. pyvisa-py detaches it automatically when it has permission (H).
* In a container the USB bus must be passed through (`/dev/bus/usb` plus the
  `c 189:* rmw` device cgroup rule) so re-enumeration after reset survives.
  `*RST` reboots the board: it disconnects from USB and re-enumerates with a
  new device number (H), so the new `/dev/bus/usb` node must exist in the
  container (a bind mount of the host's `/dev/bus/usb`, not a single device).

### Discovery

1. Enumerate USB devices with VID `0x2012` exposing a USBTMC interface.
2. Open each and send `*IDN?`.
3. Accept the device if the response has the form
   `Quonset Microwave,QM4004,<serial>,<firmware>` (H).

## 2. Message syntax

| Rule | Tag |
|------|-----|
| ASCII, SCPI-style. Program messages end with LF; CR LF also accepted | M |
| Keywords have a short form (upper-case part) and long form; the short form is used on the wire | M, S |
| One or more spaces/tabs separate a keyword from its parameter | M |
| `;` separates multiple commands in one message | M |
| A query is a command ending in `?`. Only queries produce a response | M |
| If several queries are sent before reading, only the last response is returned. The host must therefore read each response before sending the next query | M |
| Responses are ASCII terminated by a single LF | S (script strips one trailing character) |
| Booleans accept `ON`/`OFF`/`1`/`0` and always read back `0`/`1` | M |
| Default units: frequency GHz, time ms, temperature °C | M |
| Header mnemonics longer than 12 characters are rejected (-112) | M |

Numeric parameters are sent as plain decimals (`%f` for frequencies, integers
elsewhere). Observed responses (H): frequencies `2.400`, ramp time `16.00`,
integers and booleans bare (`2`, `1`), temperature `19.28`. The parser accepts
NR1, NR2 and NR3 forms.

## 3. Command reference

Wire form shown is the short form the vendor software uses.

### 3.1 Sweep (transmit waveform)

| Command | Query | Parameter | Range | Power-up default | Tag |
|---------|-------|-----------|-------|------------------|-----|
| `SWEEP:FREQSTAR <f>` | `SWEEP:FREQSTAR?` | start frequency, GHz | 2.4 – 2.5 | 2.4 | M, S |
| `SWEEP:FREQSTOP <f>` | `SWEEP:FREQSTOP?` | stop frequency, GHz | 2.4 – 2.5 | 2.5 | M, S |
| `SWEEP:RAMPTIME <t>` | `SWEEP:RAMPTIME?` | one-way ramp time, integer ms. A decimal point is a syntax error (-102) although the query returns `16.00`. `0` is accepted by the firmware without error and must be rejected by the host (H) | 1 – 65536, further bounded by §3.3 | 16 | M, S, H |
| `SWEEP:TYPE <type>` | `SWEEP:TYPE?` | `0`–`3` only; the mnemonics in the manual are a syntax error (-102) (H). Query returns the number | — | 2 (AUTO) | M, H |
| `SWEEP:START` | — | start sweeping with current settings | — | — | M |
| `SWEEP:STOP` | — | stop sweep and turn RF off | — | — | M |

Sweep types:

| Value | Name | Waveform | Repetition |
|-------|------|----------|------------|
| 0 | RAMP | linear start→stop, then jump back to start | one ramp per `SWEEP:START` / `*TRG` |
| 1 | TRI | linear start→stop→start | one triangle per `SWEEP:START` / `*TRG` |
| 2 | AUTO | linear start→stop→start | free-running until stopped |
| 3 | CW | single tone at the start frequency; stop frequency and ramp time are accepted but ignored | continuous |

Side effects:

* Changing `SWEEP:TYPE` stops the sweep; a `SWEEP:START` is required
  afterwards (M). `POWE:RF?` still reads 1 after the change (H).
* Out-of-range parameters are rejected with error -222 and leave the setting
  unchanged (H; the manual says 201).
* Measured sweep period in AUTO matches `2 * T` to within 1 % (H, from the
  audio tap: 8.01, 31.93, 79.17 ms for `T` = 4, 16, 40 ms).
* In types 0 and 1 a frame shows a single sweep of duration `T` at its start
  and nothing after it (H). Whether type 1 produces a down ramp is unresolved.

CV firmware accepts decimal ramp times, adds type 4 (sawtooth) and returns
201 for out-of-range parameters (§6).

### 3.2 RF output

| Command | Query | Meaning | Default | Tag |
|---------|-------|---------|---------|-----|
| `POWE:RF <bool>` | `POWE:RF?` | un-mute / mute the RF output | manual: 0 (off); observed after power-up: 1 with the PLL locked in AUTO sweep (H) |  M, H |

The board transmits from the moment it is powered, before any host command.
RF on/off switches an active-low RF enable line (hardware.md §4.6).
Output power is not adjustable (M: up to 1 W class output listed for sweep
modes, 0.125 W for CW). The host must leave the RF off whenever it is not
actively capturing: session close, error paths and signal handlers send
`SWEEP:STOP`.

### 3.3 PLL

| Command | Query | Meaning | Range | Tag |
|---------|-------|---------|-------|-----|
| — | `FREQ:LOCK?` | PLL lock: 1 locked, 0 unlocked | — | M, H |
| `FREQ:REF:DIV <n>` | `FREQ:REF:DIV?` | reference divider | 1 – 256 | M |

The synthesizer steps frequency from a 20 MHz reference with a 25-bit
modulus, which sets a minimum sweep slope and therefore a maximum usable ramp
time for a given bandwidth (M, eq. 2.1/2.2, restated in SI units):

```
slope_min = f_ref^2 / (refdiv * 2^25)        [Hz/s],  f_ref = 20e6 Hz
T_max     = (f_stop - f_start) / slope_min   [s]
```

The host validates `ramp time <= T_max` before sending `SWEEP:RAMPTIME` and
offers the smallest `refdiv` that satisfies a requested ramp time.

`FREQ:LOCK?` reports the synthesiser's lock-detect status, read over its
serial port; no lock-detect line is wired to the controller
([hardware.md](hardware.md) §4.1). It can read 0 in every sweep type after
power-up and recover only on a power cycle (H).

The synthesiser's fractional word is 24 bits and its sweep increment is a
whole number of fractional LSBs per reference period, truncated, so a ramp
sweeps up to one LSB per step less than requested; the shortfall matters for
long ramps over small bandwidths (hardware.md §4.1). The host stores the
read-back settings, not the realised bandwidth. The host refuses to capture unless it reads 1 after
`SWEEP:START`.

CV firmware has no reference divider (f_PD is always 20 MHz), a 2.25–2.5 GHz
range and the slope rule `slope >= f_ref^2 / 2^24`, and reads back the
realised sweep (§6).

### 3.4 Capture

| Command | Meaning | Range | Tag |
|---------|---------|-------|-----|
| `CAPT:FRAM <n>` | acquire `n` consecutive ADC samples into device memory | 1 – 4096 | M |
| `CAPT:FRAM?` | return the next chunk of the acquired frame | — | M |

ADC: 16 bit, input span 5 V centred on mid-scale (M, S); all 16 bits are
live in recorded frames (H). **The sample rate
is 21 977 Hz, not the documented 20 kHz** (H): fitted against a simultaneous
96 kHz audio recording of the IF, and consistent with the sweep period seen
in frames (702 samples per 32 ms) and with frame acquisition time. It is
16 MHz / 728: the ADC is clocked from the controller's own 12 MHz oscillator,
independent of the synthesiser's 20 MHz reference (hardware.md §4.2). All host processing uses the measured rate; axes
computed by the vendor software with 20 kHz are 9 % short.

`CAPT:FRAM?` response:

* Up to 31 samples per response, each 4 hexadecimal digits, most significant
  digit first, no separators: at most 124 characters plus the LF terminator.
* Samples are returned in acquisition order; each query advances a read
  cursor. A frame of `n` samples needs `ceil(n / 31)` queries; the final
  response carries `n mod 31` samples when that is non-zero.
* `Not Ready` is returned when no frame data is pending: after the last
  chunk has been read, or with no capture requested (H). It is not returned
  while a capture is in progress; the first `CAPT:FRAM?` simply blocks until
  the frame is complete (H).

Sample decoding (S):

```
code  = int(hex4, 16)            # 0 .. 65535, unsigned
volts = code * 5 / 65535 - 2.5
```

Timing and synchronisation (H):

* `CAPT:FRAM <n>` returns immediately. Acquisition starts about 95 ms later
  and lasts `n / 21977` s; the first `CAPT:FRAM?` returns when it is done
  (284 ms for 4096 samples).
* Each further `CAPT:FRAM?` round trip takes 3.4 ms. A 4096-sample frame
  takes 0.73 s end to end (1.37 frames/s); 1024 samples take 0.26 s.
* **Acquisition is synchronised to the sweep.** In AUTO, consecutive frames
  of a static scene are identical sample for sample (correlation 1.00 at
  zero lag) for every ramp time tested. The sweep is restarted for the
  capture: the audio tap shows the sweep phase reset at the start of each
  acquisition. The first few milliseconds of a frame contain the restart
  transient.
* In types 0 and 1, `CAPT:FRAM` triggers one sweep aligned to the frame,
  provided `SWEEP:START` was sent after the type was selected. Sending
  `SWEEP:START` or `*TRG` separately is unnecessary and, sent after
  `CAPT:FRAM`, corrupts the alignment.

Host capture procedure:

1. Send `CAPT:FRAM <n>`.
2. Send `CAPT:FRAM?` with a read timeout of the acquisition time plus a
   margin. `Not Ready` at this point is an error.
3. Repeat `CAPT:FRAM?` until `n` samples are collected. Validate every
   response: length is a multiple of 4, at most 124, all characters
   hexadecimal, and the total never exceeds `n`.
4. On any validation failure or timeout: read `CAPT:FRAM?` until
   `Not Ready` (in place of a device clear, §1), drain `SYST:ERR?`, discard
   the frame and retry.

There is no continuous streaming over USB. Consecutive frames are separated
by the read-out time (`ceil(n/31)` request/response round trips) and are
timestamped on the host at step 1.

CV firmware streams acquisitions of any length as binary blocks, at a
sample rate locked to the synthesiser reference and known exactly (§6).

### 3.5 System

| Command | Response | Tag |
|---------|----------|-----|
| `*IDN?` | `Quonset Microwave,QM4004,<serial>,<firmware>` | H |
| `SYST:IDEN?` | `QM4004` (not the `*IDN?` string the manual describes) | H |
| `SYST:SERNUM?` | serial number | M |
| `SYST:MODNUM?` | model number | M |
| `SYST:FIRM?` | firmware version | M |
| `SYST:VERS?` | SCPI version `YYYY.V` | M |
| `SYST:TEMP?` | board temperature, °C, from the single on-board sensor (1/32 °C resolution, hardware.md §4.3) | M |
| `SYST:BLUE?` | documented (M) but rejected as an undefined header (-113) with no response on firmware V1.1.0 (H); not used | M, H |
| `SYST:STAT?` | `<code>, "<text>"` (table below) | M |
| `SYST:ERR?` | `<code>, "<text>"`; pops the oldest entry of a 10-deep FIFO; `0, "No error"` when empty. First read after power-up returns `-500,"Power on"` (H) | M, H |
| `SYST:PRES` / `*RST` | return to power-up state (memory location 0) | M |
| `SYST:REST` | overwrite memory location 0 with factory defaults | M |
| `SYST:CLRM <1-9>` | erase a saved state | M |
| `*SAV <0-9>` / `*RCL <0-9>` | save / recall instrument state | M |

`SYST:STAT?` codes: 0 operational, 1 device has been reset, 2 awaiting user
input, 100 recoverable error, 101 non-recoverable error, 110 over
temperature.

Memory location 0 is the power-up state. The host library refuses `*SAV 0`
and `SYST:REST` unless explicitly forced.

### 3.6 IEEE 488.2 common commands

`*CLS`, `*ESE`/`*ESE?`, `*ESR?`, `*OPC`/`*OPC?`, `*OPT?`, `*SRE`/`*SRE?`,
`*STB?`, `*TRG`, `*TST?` (0 pass, 1 fail), `*WAI` (M).

| Register | Bit 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|----------|-------|---|---|---|---|---|---|---|
| ESR / ESE | operation complete | — | query error | device error | execution error | command error | — | power on |
| STB / SRE | — | device status | error queue not empty | questionable summary | message available | event status | master summary (STB only) | operation summary |

`*TRG` starts one sweep in the RAMP and TRI types; it raises "trigger
ignored" when the device is not waiting for a trigger (M).

### 3.7 Error codes

Device-specific:

| Code | Meaning |
|------|---------|
| 0 | no error |
| 110 | command invalid for this device |
| 201 | parameter outside the device operating range (M); firmware V1.1.0 returns -222 instead (H) |

Standard SCPI codes the firmware can return: -101, -102, -103, -105, -108,
-109, -112, -113, -121, -123, -124, -128, -131, -134, -138, -141, -148, -151,
-158, -161, -168, -178 (command errors); -200, -211, -213, -222, -224, -230,
-241 (execution errors); -310, -330, -350 (device errors); -410, -420, -430,
-440 (query errors).

Host error policy: every setter is followed by `SYST:ERR?` until the queue
reads 0; a non-zero code raises a typed exception carrying code and text.
The light bar on the board (10 segments) flashes while an error is queued (M).

### 3.8 Command set used by the host

The host sends only the commands documented in the user manual and listed in
§3.1–3.6; any other header is refused before it reaches the board
(`qmrdk/transport.py`). `CAPT:STRE` (Bluetooth streaming) and `SYST:BLUE?`
are documented but not used. The CV-only commands of §6.5 are allowed only
when `*IDN?` reports CV firmware.

## 4. Session sequences

Connect:

```
CAPT:FRAM? until Not Ready  (discard pending frame data)
*IDN?                      -> identify, record serial and firmware
*CLS
SYST:ERR?                  -> expect 0
SWEEP:FREQSTAR? SWEEP:FREQSTOP? SWEEP:RAMPTIME? SWEEP:TYPE? FREQ:REF:DIV? POWE:RF?
```

Configure and capture (each setter followed by the error check of §3.7):

```
SWEEP:TYPE <t>
SWEEP:FREQSTAR <f0>
SWEEP:FREQSTOP <f1>
FREQ:REF:DIV <n>           (only when the ramp time requires it)
SWEEP:RAMPTIME <T>
SWEEP:START
FREQ:LOCK?                 -> expect 1
CAPT:FRAM <n>  ...  CAPT:FRAM? x ceil(n/31)
```

Disconnect (also on every abnormal exit):

```
SWEEP:STOP
POWE:RF?                   -> expect 0
```

The host always reads the sweep parameters back and stores the read-back
values, not the requested ones, with every capture.

## 5. Recorded data

Native recording: one compressed `.npz` per recording holding

| Field | Type | Content |
|-------|------|---------|
| `codes` | `uint16 [captures, samples]` | raw ADC codes |
| `t_host` | `float64 [captures]` | host UNIX time at `CAPT:FRAM` |
| `x_pos` | `float64 [captures]`, SAR scans only | sled position, m |
| `temperature` | `float64 [captures]`, optional | `SYST:TEMP?` read before each `CAPT:FRAM`, °C |
| `meta` | JSON string | start/stop frequency (GHz), ramp time (ms), sweep type, reference divider, sample rate, `*IDN?` fields, software version; with CV firmware also the device timing (§6.10) |

Vendor CSV (import/export, for exchange with files saved by the Windows GUI)
(S): one header row, then one row per sample with columns time (s),
amplitude (V), and on the first data row start frequency (GHz), stop
frequency (GHz), ramp time (ms).

## 6. CV firmware

Replacement controller firmware ([firmware.md](firmware.md) holds its
design). It keeps the USB identity of §1 and the stock commands the host
sends, locks the sample clock to the synthesiser reference and adds
streaming. Everything in this section is the firmware's specification, not
hardware observation.

### 6.1 Detection

The firmware field of `*IDN?` starts with `CV`:
`Quonset Microwave,QM4004,<serial>,CV1.0.0-<commit>`. The host selects its CV path
from it (`Idn.cv`).

### 6.2 Constants

| Name | Value | Meaning |
|------|-------|---------|
| `F_REF` | 20 MHz | synthesiser reference, equal to the phase-detector rate f_PD |
| `R` | default 800, range 500 – 16383 | sample clock divider: `fs = F_REF / R` exactly (25 kHz default, 1220.78 Hz – 40 kHz) |
| `f_min`, `f_max` | 2.25, 2.5 GHz | frequency range |
| `slope_min` | `F_REF^2 / 2^24` = 23 841 857.91 Hz/s | sweep step of at least one fractional LSB per PD cycle |
| `M` | 16 PD cycles | programmed ramp shortening (trigger margin) |
| `D` | 22.5 PD cycles | trigger delay from the sample instant |
| block | 1024 samples | samples per `CAPT:BLOC?` block |

### 6.3 Sweep model

The sweep is exact; the read-backs are the realised values.

* Frequencies are Q24 multiples of `F_REF`: `Nq = round(f * 2^24 / F_REF)`.
* `Nr = round(T * fs)` samples per ramp, at least 4; realised ramp time
  `T' = Nr / fs`, so `Nr` is an integer.
* `K = Nr * R` PD cycles per ramp; `step = round((N1q - N0q) / K)`, signed,
  non-zero, `|step| < 2^23`.
* `SWEEP:FREQSTAR?` = `N0q * F_REF / 2^24`, `SWEEP:FREQSTOP?` =
  `(N0q + step * K) * F_REF / 2^24`; slope `(FREQSTOP - FREQSTAR) / T'`
  exactly.
* The programmed ramp is `K - M` PD cycles long, starts `M / 2` steps after
  FREQSTAR and is triggered from the sample interrupt every `Nr` samples, so
  the ideal triangle through the ramp lines has its apexes at FREQSTAR and
  FREQSTOP and its turnarounds at `n0 + k * Nr` with
  `n0 = (D - M / 2) / R = 14.5 / R` samples (0.018125 at R = 800).
* `first_up` = 1 if FREQSTAR < FREQSTOP, else 0 (the first ramp sweeps
  down).
* Sawtooth (type 4): a ramp of `Nr` samples in the `first_up` direction,
  a hop back to the start frequency and a rest of `Nd = round(dwell * fs)`
  samples (at least 4); period `Nr + Nd`, ramps starting at
  `n0 + j * (Nr + Nd)`.

| `SWEEP:TYPE` | Host kind | Behaviour |
|--------------|-----------|-----------|
| 0 | `ramp` | one ramp per acquisition, triggered at its sample 0 |
| 1 | `tri` | one triangle per acquisition, triggered at its sample 0 |
| 2 | `triangle` | continuous triangle; acquisitions aligned to its period `2 Nr` |
| 3 | `cw` | CW at FREQSTAR |
| 4 | `saw` | continuous sawtooth (CV only); acquisitions aligned to its period `Nr + Nd` |

### 6.4 Stock commands

As §2–§3.7, with these differences:

* Out-of-range parameters return 201 `"Out of range"` (the manual's code),
  not -222. `SWEEP:RAMPTIME 0` and ramps under 4 samples are refused;
  `SWEEP:START` with `step = 0` returns 201 and does not start.
* `SWEEP:RAMPTIME` accepts decimal ms (up to 65536). Read-backs: frequencies
  GHz with 9 decimals, ramp time ms with 6 decimals.
* `SWEEP:TYPE` accepts 0–4.
* `FREQ:REF:DIV` is accepted and read back but has no effect; the host never
  sends it to CV firmware.
* RF is off at power-up until `SWEEP:START` or `POWE:RF 1`.
* `FREQ:LOCK?` and `SYST:TEMP?` return the values read just before the
  sample clock started while it runs.
* Not implemented: `*SAV`, `*RCL`, `*TRG`, `*OPT?`, `SYST:PRES`, `SYST:REST`,
  `SYST:CLRM`, `CAPT:STRE?`.

### 6.5 New commands

| Command | Response / effect |
|---------|-------------------|
| `SWEEP:DWEL <ms>` / `SWEEP:DWEL?` | sawtooth rest; default 1 ms; `Nd >= 4` else 201; query realised ms, 6 decimals |
| `SWEEP:ORIG?` | `<n0>,<first_up>`: commanded first turnaround in samples from acquisition sample 0 (`round(14.5e6 / R)` micro-samples, 6 decimals: `0.018125,1` at R = 800); CW `0.000000,1` |
| `SWEEP:LIM?` | `<f_min GHz>,<f_max GHz>,<slope_min Hz/s>,<t_quantum ms>,<t_min ms>` with `t_quantum = 1 / fs`, `t_min = 4 / fs` |
| `CAPT:RATE <Hz>` / `CAPT:RATE?` | `R = round(F_REF / Hz)` in 500 – 16383 else 201; query `F_REF / R`, 6 decimals; restarts a running sweep |
| `CAPT:CONT <n>` | start an acquisition of `n` samples, 0 – 2147483647 (0: until `CAPT:ABOR`); read with `CAPT:BLOC?` only |
| `CAPT:ABOR` | stop the acquisition and discard pending data |
| `CAPT:BLOC?` | next block (§6.6); deferred while the acquisition runs and no block is complete; `#6000000` when nothing is pending |
| `POWE:RF:TIME <s>` / `POWE:RF:TIME?` | RF auto-mute after `s` s without a host message while no acquisition runs; 0 disables; default 30; 0 – 3600 |

The host reads `CAPT:RATE?`, `SWEEP:LIM?` and `SWEEP:ORIG?` but does not set
the rate or the auto-mute; `qmrdk scpi` can.

### 6.6 Block format

IEEE 488.2 definite-length block with a fixed 6-digit length, then LF:

```
'#' '6' dddddd  payload[dddddd]  '\n'
payload = u32 seq | u32 first | u16 count | u16 flags | count x u16 samples   (little-endian)
```

* `seq`: block number in the acquisition, from 0.
* `first`: index of the block's first sample from acquisition sample 0.
* `count`: 1024 except possibly in the last block.
* `flags`: bit 0 OVERFLOW (samples were dropped immediately before this
  block; `first` skips them), bit 1 END (last block of the acquisition).
* Empty: `#6000000` + LF.

Samples are the raw codes of §3.4.

### 6.7 Acquisition alignment

Sample 0 of an acquisition (`CAPT:FRAM` or `CAPT:CONT`) is the sample whose
interrupt triggers a ramp in the `first_up` direction, for types 0, 1, 2
and 4, so the turnarounds are at `n0 + k * Nr` (sawtooth ramp starts at
`n0 + j * (Nr + Nd)`) in every acquisition:

* types 2 and 4 running: the next sample on the sweep period grid; latency
  at most one period, no restart transient;
* types 0 and 1: after the VCO is re-parked at the start frequency and lock
  polled;
* CW or stopped: the first sample after the clock starts.

### 6.8 Host capture procedure

1. Send `CAPT:CONT <n>`, any `n >= 1` (no 4096 limit); the host time is
   taken here.
2. Read `CAPT:BLOC?` until `n` samples are collected. The first read's
   timeout is the alignment latency (`2 T` triangle, `T + dwell` sawtooth,
   the 50 ms lock wait for types 0 and 1, 0 for CW), plus the block time
   `min(n, 1024) / fs`, plus a margin; later reads `1024 / fs` plus the
   margin.
3. Validate every block as `qmrdk.device.decode_block` does: a well-formed
   header and length, `seq` and `first` continuing the previous block, no
   OVERFLOW, `0 < count <= n - first`, and END exactly on the block that
   completes `n`. An empty block before the end is an error.
4. On any validation failure or timeout: `CAPT:ABOR`, drain `SYST:ERR?`,
   discard the frame and retry.

On every settings read the host refreshes `fs` from `CAPT:RATE?` (as
`F_REF / round(F_REF / rate)`) and the limits from `SWEEP:LIM?`, and reads
the timing with `SWEEP:ORIG?`.

### 6.9 Session sequences

Connect (`*IDN?` has already identified CV firmware):

```
CAPT:ABOR                  (stop a stream left running)
CAPT:FRAM? until Not Ready
SYST:ERR?                  -> drain
*CLS
CAPT:RATE? SWEEP:LIM? SWEEP:FREQSTAR? SWEEP:FREQSTOP? SWEEP:RAMPTIME? SWEEP:TYPE?
SWEEP:DWEL? (sawtooth)  POWE:RF? FREQ:LOCK? SWEEP:ORIG?
```

Configure and capture (each setter followed by the error check of §3.7):

```
SWEEP:TYPE <t>
SWEEP:FREQSTAR <f0>        (GHz, 9 decimals)
SWEEP:FREQSTOP <f1>
SWEEP:RAMPTIME <T>         (ms, 6 decimals)
SWEEP:DWEL <d>             (sawtooth with a dwell given)
SWEEP:START
FREQ:LOCK?                 -> expect 1
settings read (above)
CAPT:CONT <n>  ...  CAPT:BLOC? until n samples
```

Disconnect is as §4.

### 6.10 Recorded data

The device timing travels with every recording made on CV firmware, so the
processing need not estimate it: `qmrdk capture` and `qmrdk sar scan` store
in `meta`'s `extra` the exact `fs` and `timing` = `{fs, n0, first_up}`
(`SWEEP:ORIG?` with the rate); calibration frame files store `timing` next
to `sweep`.

## Open questions

Resolved by the phase 0 probe; each is one observable test.

| # | Question | Why it matters |
|---|----------|----------------|
| 1 | USB488 capability bits | transport configuration |
| 2 | Resolved (H): formats recorded in `artifacts/probe/protocol.json`; `*ESR?` 160 after power-up, `*OPT?` `0`, `SYST:VERS?` `1999.0`, `CAPT:STRE?` `0` | — |
| 3 | Resolved (H): the board powers up sweeping with RF on | — |
| 4 | Resolved (H): synchronised, see §3.4. Remaining: the exact sample offset of the first turnaround and the length of the restart transient | segmentation constant, guard |
| 5 | Resolved (H): `CAPT:FRAM` triggers the sweep, see §3.4. Remaining: whether type 1 sweeps down | single-sweep capture |
| 6 | Resolved (H): the first query blocks | — |
| 7 | Resolved (H): `Not Ready` after the last chunk; no re-read | — |
| 8 | Resolved (H): 3.4 ms | — |
| 9 | Resolved (H): pyvisa-py detaches the kernel driver | — |
| 10 | Resolved: sample rate 21 977 Hz (H), so a ramp is not an integer number of samples (351.6 for 16 ms). ADC and sweep run from independent oscillators (hardware.md §1), so their ratio drifts by the difference of the two oscillator errors (tens of ppm, temperature dependent) | sweep segmentation |
| 11 | Scaling divisor: the vendor script divides by 65535 (S); 2^16 would differ | gain error of 1.5e-5, documentation only |
