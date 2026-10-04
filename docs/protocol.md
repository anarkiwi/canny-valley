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
| M | QM-RDK User Manual rev 1.2.0 |
| S | Vendor MATLAB scripts (`RDK_CollectAndPlot_*.m`, `RDK_Plot_DataFile.m`) |
| B | Strings and constants in the vendor Windows GUI binary (v1.2.3) |
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
| Vendor ID | `0x2012` | S, B |
| Product ID | `0x0013` observed (H); `0x0017` in the vendor script (S). The GUI matches any PID under the VID (B), and so does the host library |
| Product string | `QM4004` | H |
| Serial string | Decimal serial number, zero-padded to 4 digits | S, H |
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
  empties the output buffer (M).

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

### Discovery

1. Enumerate USB devices with VID `0x2012` exposing a USBTMC interface.
2. Open each and send `*IDN?`.
3. Accept the device if the response has the form
   `Quonset Microwave,QM4004,<serial>,<firmware>` (H).

## 2. Message syntax

| Rule | Tag |
|------|-----|
| ASCII, SCPI-style. Program messages end with LF; CR LF also accepted | M |
| Keywords have a short form (upper-case part) and long form; the short form is used on the wire | M, S, B |
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

### 3.2 RF output

| Command | Query | Meaning | Default | Tag |
|---------|-------|---------|---------|-----|
| `POWE:RF <bool>` | `POWE:RF?` | un-mute / mute the RF output | manual: 0 (off); observed after power-up: 1 with the PLL locked in AUTO sweep (H) |  M, H |

The board transmits from the moment it is powered, before any host command.
Output power is not adjustable (M: up to 1 W class output listed for sweep
modes, 0.125 W for CW). The host must leave the RF off whenever it is not
actively capturing: session close, error paths and signal handlers send
`SWEEP:STOP`.

### 3.3 PLL

| Command | Query | Meaning | Range | Tag |
|---------|-------|---------|-------|-----|
| — | `FREQ:LOCK?` | PLL lock: 1 locked, 0 unlocked | — | M |
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

### 3.4 Capture

| Command | Meaning | Range | Tag |
|---------|---------|-------|-----|
| `CAPT:FRAM <n>` | acquire `n` consecutive ADC samples into device memory | 1 – 4096 | M |
| `CAPT:FRAM?` | return the next chunk of the acquired frame | — | M |

ADC: 16 bit, input span 5 V centred on mid-scale (M, S). **The sample rate
is 21 977 Hz, not the documented 20 kHz** (H): fitted against a simultaneous
96 kHz audio recording of the IF, and consistent with the sweep period seen
in frames (702 samples per 32 ms) and with frame acquisition time. The value
matches 16 MHz / 728. All host processing uses the measured rate; axes
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
4. On any validation failure or timeout: device clear, `SYST:ERR?` drain,
   discard the frame.

There is no continuous streaming over USB. Consecutive frames are separated
by the read-out time (`ceil(n/31)` request/response round trips) and are
timestamped on the host at step 1.

### 3.5 System

| Command | Response | Tag |
|---------|----------|-----|
| `*IDN?` | `Quonset Microwave,QM4004,<serial>,<firmware>` | H |
| `SYST:IDEN?` | `QM4004` (not the `*IDN?` string the manual describes) | H |
| `SYST:SERNUM?` | serial number | M |
| `SYST:MODNUM?` | model number | M |
| `SYST:FIRM?` | firmware version | M |
| `SYST:VERS?` | SCPI version `YYYY.V` | M |
| `SYST:TEMP?` | maximum board temperature, °C | M |
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
The light bar on the board flashes while an error is queued (M).

### 3.8 Commands that must not be sent

The GUI binary contains commands from the vendor's shared code base that are
not documented for this board (B): `FACT:BIASGATE`, `SYST:POWE`, `SYST:ATTN`,
`SYST:TIME?`, `SYST:CURR?`, `SYST:MODULESTATUS?`, `SYST:FIRMID?`,
`SYST:MODU?`, `SYST:VENDOR?`, `SYST:ALTERA?`, `SYST:SENDSTR`/`READSTR`,
`LCD:*`. The host library exposes none of them; the `FACT:` subsystem in
particular is factory calibration and is never sent, including by the probe.

## 4. Session sequences

Connect:

```
device clear
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
| `meta` | JSON string | start/stop frequency (GHz), ramp time (ms), sweep type, reference divider, sample rate, `*IDN?` fields, software version |

Vendor CSV (import/export, for exchange with files saved by the Windows GUI)
(S): one header row, then one row per sample with columns time (s),
amplitude (V), and on the first data row start frequency (GHz), stop
frequency (GHz), ramp time (ms).

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
| 10 | Sample rate measured as 21 977 Hz (H), so a ramp is not an integer number of samples (351.6 for 16 ms). Remaining: stability of the ratio between ADC and sweep clocks | sweep segmentation |
| 11 | Scaling divisor: the vendor script divides by 65535; one GUI code path multiplies by 2^-16 | gain error of 1.5e-5, documentation only |
