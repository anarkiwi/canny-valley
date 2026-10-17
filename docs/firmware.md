# CV controller firmware

Replacement program for the QM-RDK controller (U4, PIC24FJ256GB106,
[hardware.md](hardware.md)). It keeps the stock USB identity and every
command the `qmrdk` host sends, so the host and the vendor-compatible paths
work unchanged, and adds a sample clock locked to the synthesiser
reference, deterministic sweep timing, binary streaming and the synthesiser
corrections below. `*IDN?` reads `Quonset Microwave,QM4004,<serial>,CV1.0.0-<commit>`, where `<commit>` is the first 12 hex digits of the commit the image was built from (`unknown` if the build was not given one; `-dirty` marks uncommitted changes);
the host selects its CV path when the firmware field starts with `CV`
([driver.md](driver.md)).

## Structure

| Path | Content |
|---|---|
| `firmware/radar/core/` | portable C: SCPI command set (libscpi), HMC703 words and register values (`synth.c`), per-sample scheduler run from the sample ISR (`isr_sched.h`), block framing and output (`radar.c`), exact fixed-point formatting (`fmt.c`); hardware access only through `hal.h` |
| `firmware/radar/test/` | host tests of the core with a fake board (doctest, gcovr): register sequences, trigger instants, frame alignment, block and hex output, errors |
| `firmware/radar/pic/` | board layer: pins, PPS, SPI1/SPI2, sample ISR, Timer1 tick, Timer2 edge counter, TinyUSB USBTMC/USB488 device and descriptors |
| `firmware/radar/lib/` | TinyUSB and libscpi, pinned git submodules |
| `tools/isr_cycles.py` | best/worst-case cycle count of the sample ISR from `build/radar/isr.lst` |

Interrupt priorities: sample clock (INT1) 7, USB 4, millisecond tick lowest.

## Sample clock

The HMC703 runs with Reg 06h[21] = 1 (Force RDIV bypass): the phase detector
uses the undivided 20 MHz reference (f_PD = f_ref) and the R divider
(Reg 02h = R) is free to be exported on the GPO (HMC703 datasheet, "Ref Path
'R' Divider"). During acquisitions and while a continuous sweep runs,
Reg 0Fh = GPOSel | GPOAlways (bit 6) | GPOOn (bit 7), so LD_SDO always
drives the divider output and is never muxed to SDO. LD_SDO reaches U4 RB14
(RP14), which PPS maps to SDI2, INT1 and T2CK at once.

* GPOSel 9 (`rdiv_pfd`), the selection the datasheet gives for viewing the R
  divider on LD_SDO. At power-up the firmware counts GPO edges with Timer2
  for 20 ms and accepts the selection if the count is within 2 % of
  f_ref / R; otherwise it tries 7 (`rdiv core`). If neither matches, error
  -330 is queued and sweeps and captures are refused.
* Each rising edge runs one INT1 interrupt that ends the previous AD7694
  conversion (CNV low), reads its 16 bits on SPI1 at 8 MHz (the PIC24 maximum,
  FCY / 2; AD7694 tSCK >= 50 ns at 5 V, 125 ns below 4.75 V) and starts the
  next conversion (CNV high). CNV then stays high for the rest of the period,
  which is always longer than tCONV max (3.2 µs at VDD 4.75–5.25 V, 4.66 µs
  below).
* fs = f_ref / R exactly; default R = 800 (fs = 25 kHz, so every ramp time
  that is a multiple of 40 µs is a whole number of samples). `CAPT:RATE`
  selects R in 500 .. 16383.
* Outside acquisitions and continuous sweeps Reg 0Fh = 0x000001 (lock
  detect, driver off except during reads), so the GPO is quiet and SPI2 reads
  work. No synthesiser read happens while the clock runs: `FREQ:LOCK?` then
  returns the lock state read just before the clock started, and
  `SYST:TEMP?` the temperature read then (SPI1 belongs to the ISR).

### Timing budget

FCY = 16 MHz. From the INT1 edge: about 5 TCY entry latency, CNV rising edge
about 57 TCY later, trigger frame registered 18 TCY after the CNV rising edge.
Cycle counts of `_INT1Interrupt` from the listing (`tools/isr_cycles.py`,
SPI poll loops at 8 MHz added):

| Path | TCY |
|---|---|
| clock running, no acquisition, no trigger frame | ~120 |
| acquiring, with a trigger frame | ~225 |
| worst case (block boundary, trigger frame, end of acquisition) | ~300 |

The next edge must not arrive before the worst case finishes, or the
following CNV edge is late. This sets R >= 500: at R = 500 (40 kHz) the
period is 400 TCY, above the worst case, and the typical path leaves about
two thirds of the CPU to USB and the main loop. The only sample-time
jitter is the interrupt entry (the instruction in progress, 0–2 TCY).

## Sweeps

Frequencies are Q24 multiples of f_PD: `Nq = round(f · 2^24 / f_PD)`, written
as Reg 03h = Nq >> 24, Reg 04h = Nq mod 2^24 (and 0Ch/0Dh for the stop).

* Ramp time is quantised to whole samples: `Nr = round(T · fs)` (>= 4),
  `T' = Nr / fs`, `K = Nr · R` PD cycles. The sweep step is counted per PD
  cycle (f_PD = f_ref with the bypass), which removes the stock `R × T` ramp
  length error.
* MODSTEP (Reg 0Ah, 24-bit two's complement) `= round((N1q − N0q) / K)`,
  nearest rather than truncated. The realised stop `N0q + MODSTEP · K` is what
  `SWEEP:FREQSTOP?` reports. A negative MODSTEP (FREQSTAR above FREQSTOP)
  sweeps down first. The slope limit is MODSTEP >= 1 LSB per PD cycle:
  `slope_min = f_PD² / 2^24 = 23.84 MHz/s`; ramp times are free otherwise
  (no integer-ms restriction, up to 65 536 ms).
* Every ramp is triggered from the sample ISR (Reg 0Eh 0 → 1, EXTTRIG_EN = 0)
  at an exact sample index, so the sweep period is a whole number of sample
  periods by construction, whatever the chip does at the turnaround. The
  programmed ramp is M = 16 PD cycles shorter than K and starts M/2 steps
  above the requested start (`Ns = N0q + MODSTEP · M/2`, stop
  `Ne = Ns + MODSTEP · (K − M)`), so it ends before the next trigger and the
  ideal triangle through the ramp lines has its apexes exactly at the
  reported FREQSTAR/FREQSTOP.
* The trigger frame is clocked out as three bytes in the ISR before its
  sample and the last byte right after its CNV rising edge; the HMC703
  registers a write on the 32nd SCK rising edge (datasheet "HMC Mode – Serial
  Port WRITE Operation"), D = 18 TCY = 22.5 PD cycles after the sample
  instant. Turnarounds are therefore at `n0 + k · Nr` with
  `n0 = (D − M/2) / R` samples (0.018125 at R = 800), reported by
  `SWEEP:ORIG?` with the first-ramp direction.

| `SWEEP:TYPE` | SD_MODE (Reg 06h[7:5]) | Triggers |
|---|---|---|
| 0 ramp | 5 one-way | one per acquisition, at its sample 0 (the VCO is re-parked at the start frequency first, lock polled) |
| 1 tri | 6 two-way | at sample 0 and Nr of each acquisition |
| 2 triangle (AUTO) | 6 two-way | every Nr samples while the sweep runs |
| 3 cw | 0 fractional | none |
| 4 saw | 5 one-way | ramp start, hop back after Nr, next start after Nd more (`SWEEP:DWEL`), period Nr + Nd |

Acquisitions on a running type 2 or 4 sweep start on the next period
boundary (an up-ramp start), at most one period after the command, with no
restart transient; types 0/1 start within the lock wait (<= 50 ms) and CW or
stopped captures at the next sample.

### Register settings

| Reg | Value | Note |
|---|---|---|
| 01h | `0x000002` | chip enable from SPI (stock) |
| 02h | R | GPO divider only (PFD bypassed) |
| 06h | `0x201D1F | mode << 5` | stock modulator settings plus RDIV bypass (bit 21) |
| 07h | `0x204865`, then `0x304865` | bit 21 = 1 as the datasheet requires; lock-detect timer trained (bit 20 0 → 1) after f_PD is set at power-up; R changes do not change f_PD |
| 08h | `0x016FFF` | stock |
| 09h | `0x803264 | offset << 14 | 1 << 21` | 2 mA up/down, CPHiK; offset in every mode (all modes used are fractional), source polarity because 0Bh PFDInv = 1 (Reg 09h table: CPSrcEn = PFDInv, CPSnkEn = NOT PFDInv) |
| 0Bh | `0x01EC71` | stock (PFDInv = 1) |
| 0Fh | `0x000001` idle, `GPOSel | 0xC0` exporting | above |

Charge-pump offset (datasheet EQ 4): `I_off = (2.5 ns + 4 T_ps) · f_PD · I_CP`
with T_ps = 1 / f_VCO (RF ÷2 bypassed), f_PD = 20 MHz, I_CP = 2 mA, in 5 µA
steps: `offset = 20 + round(32e9 / f_c)` with f_c the sweep centre, i.e.
33 (165 µA) across most of the band and 34 at 2.25 GHz. The stock firmware
applied 107 (535 µA) in CW only.

VCO limits: 2.25–2.5 GHz, the HMC385LP4's specified tuning range (HMC385LP4
datasheet, Electrical Specifications). Outside 2.4–2.4835 GHz the board
transmits outside the ISM band: use a 50 Ω termination or shielded setup
there unless licensed.

## Safety and housekeeping

* RF enable (RB5, active low) is driven off as the first action at reset and
  stays off until `SWEEP:START` or `POWE:RF 1`.
* RF auto-mute: RF goes off after `POWE:RF:TIME` seconds (default 30, 0
  disables) without any host message while no acquisition runs.
* The Bluetooth module is never configured: UART1 off and unmapped, RB2 held
  at 0, U1TX pin held high (idle).
* Serial number from EEPROM 0x0100 (uint16 LE) if the 0x00FE marker is 0x6F
  and the value is below 10 000, else 0021; used for the USB serial string
  and `*IDN?`.
* Out-of-range parameters return error 201 `"Out of range"` (manual §3.7);
  `SWEEP:RAMPTIME 0` is refused.

## Command set

Kept, with the formats of [protocol.md](protocol.md) §2–§3.4: `*IDN?`,
`*CLS`, `*RST`, `*ESE(?)`, `*ESR?`, `*OPC(?)`, `*SRE(?)`, `*STB?`, `*TST?`,
`*WAI`, `SYST:ERR?`, `SYST:STAT?`, `SYST:TEMP?`, `SYST:IDEN?`, `SYST:SERNUM?`,
`SYST:MODNUM?`, `SYST:FIRM?`, `SYST:VERS?`, `SWEEP:FREQSTAR(?)`,
`SWEEP:FREQSTOP(?)`, `SWEEP:RAMPTIME(?)`, `SWEEP:TYPE(?)`, `SWEEP:START`,
`SWEEP:STOP`, `POWE:RF(?)`, `FREQ:LOCK?`, `FREQ:REF:DIV(?)` (accepted and
read back, no effect: f_PD is always f_ref), `CAPT:FRAM <1..4096>`,
`CAPT:FRAM?` (31-sample hex chunks, `Not Ready`). Read-backs carry more
digits (GHz with 9 decimals, ms with 6); ramp times accept decimals; type 4
is new. Saved states (`*SAV`, `*RCL`, `SYST:PRES`, `SYST:REST`,
`SYST:CLRM`) and Bluetooth streaming are not implemented.

New:

| Command | Meaning |
|---|---|
| `SWEEP:DWEL <ms>` / `?` | sawtooth rest (default 1 ms, >= 4 samples) |
| `SWEEP:ORIG?` | `<n0>,<first_up>`: first turnaround (samples from acquisition sample 0) and direction of the ramp starting there |
| `SWEEP:LIM?` | `<f_min GHz>,<f_max GHz>,<slope_min Hz/s>,<ramp quantum ms>,<minimum ramp ms>` |
| `CAPT:RATE <Hz>` / `?` | sample rate, R = round(f_ref / Hz) |
| `CAPT:CONT <n>` | acquire n samples (0: until `CAPT:ABOR`), read as blocks |
| `CAPT:ABOR` | stop the acquisition, discard its data |
| `CAPT:BLOC?` | next block; deferred until one is complete; `#6000000` LF when none |
| `POWE:RF:TIME <s>` / `?` | RF auto-mute timeout |

### Blocks

```
'#' '6' dddddd  payload  LF        IEEE 488.2 definite-length block
payload = u32 seq | u32 first | u16 count | u16 flags | count × u16 sample    (little-endian)
```

`seq` counts blocks from 0, `first` is the index of the block's first sample
from acquisition sample 0, `count` is 1024 except for the last block; flags
bit 0 OVERFLOW (samples before this block were dropped; `first` skips them),
bit 1 END (last block). Five 1024-sample slots (10 KB of the 16 KB RAM) form
the ring the ISR fills; a response is sent zero-copy from its slot in as many
USBTMC transfers as the host's TransferSize requires. A host that falls more
than four blocks behind loses samples, which the next block flags; nothing is
dropped silently. Frames from `CAPT:FRAM` use the same slots and can be read
as blocks or hex.

## Build, flash, restore

```
docker build -f docker/firmware.Dockerfile -t qmrdk-firmware .
docker run --rm -v "$PWD/firmware/build:/work/firmware/build" qmrdk-firmware \
  make all RADAR_COMMIT=$(git describe --always --dirty --abbrev=12 --exclude '*')
```

The image has no `.git`, so the commit is passed in as `RADAR_COMMIT`; CI
passes the commit it builds.

`make test` runs the core tests with coverage, `make compile` produces
`firmware/build/radar/radar.hex` (XC16, free edition) together with the map
and `isr.lst`. CI uploads them as the `radar-firmware` artifact. There is no
bootloader: flash with a PICkit 5 on J1 and restore the stock image the same
way ([programming.md](programming.md)).

## Bring-up checklist

Everything above is verified only by the host tests of the core, the
simulated board in `qmrdk.sim` and the XC16 build. On the board:

1. Flash, power-cycle, `qmrdk list`: serial and `CV1.0.0-<commit>` matching the flashed build; `qmrdk info`: no
   -330 in the error queue (GPO clock self-test passed), RF off.
2. Lock across the VCO range: CW at 2.25, 2.30, … 2.50 GHz, `FREQ:LOCK?` = 1
   at each; triangle sweeps over 2.25–2.5 GHz and 2.4–2.5 GHz at 1, 8 and
   40 ms with lock read after `SWEEP:STOP`.
3. Spurs with the GPO clock on vs off: on a spectrum analyser (TX terminated)
   compare CW alone with CW during `CAPT:CONT 0`; the datasheet warns that a
   toggling GPO can degrade spectral performance.
4. Sample rate against the audio tap: record the IF on the audio interface
   and a triangle sweep; the sweep period must equal 2 · Nr / fs with
   fs = f_ref / R (and the ADC/sweep ratio must not drift: `qmrdk calib
   timing` reports `locked`).
5. n0 and direction: `qmrdk calib timing` on a static reflector gives the
   apparent turnaround; it must equal `SWEEP:ORIG?` n0 plus the IF delay for
   every ramp time and rate (constant `if_delay`), and step 3 must agree with
   the reported first-ramp direction.
6. Streaming: `qmrdk capture --n 2000000` at 25 kHz and at 40 kHz with no
   OVERFLOW block.
7. RF auto-mute: RF on, no host traffic for 30 s, `POWE:RF?` = 0.
