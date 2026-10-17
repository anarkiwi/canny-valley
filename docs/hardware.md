# QM-RDK board notes

Parts, wiring and device-level interfaces of the owner's board (silkscreen
`QM4004-R3`, serial 0021, firmware V1.1.0). No vendor schematic or parts list
is available. Confidence: **H** established from register-level behaviour or
measurement, **M** inferred from interface shape or naming, **L** plausible
but unconfirmed.

## Parts

| Ref | Part | Role |
|-----|------|------|
| U4 | Microchip PIC24FJ256GB106-I/PT (64-pin TQFP; 256 KB flash, 16 KB RAM, full-speed USB, remappable peripheral pins) | controller |
| U1 | Microchip 25AA1024 SPI EEPROM (128 KB, 256-byte pages; identified by its electronic signature 0x29) | serial number and saved instrument states |
| U3 | 12 MHz source on OSCI/OSCO (H for frequency, part unread) | controller clock |
| J1 (1×6, right angle) | Microchip ICSP header | U4 programming and debug, see [programming.md](programming.md) |
| XJ1 (2×4) | header, unpopulated | odd pins GND; even pins carry U4 RB6–RB9 (§2) |
| XJ2 (2×3), XJ3 (2×4) | headers, unpopulated | unknown |
| U2 | Maxim MAX6966 (marking `MAX6966 AEE`), 10-port constant-current LED driver | light bar, one segment per port P0–P9 |
| U6 | shielded module, Bluegiga iWRAP command set (WT11/WT12 class) | Bluetooth serial link |
| U13 | oscillator module | synthesiser reference (20 MHz per the manual) |
| U15 | Hittite HMC703LP4E (marking `H703`) | fractional-N synthesiser with built-in sweeper |
| U17 | Hittite HMC385LP4 (marking `H385`) | 2.25–2.5 GHz VCO |
| U24 | Mini-Circuits SYM-25DMHW+ | receive mixer (40–2500 MHz, +13 dBm LO class) |
| U16 | Linear Technology LTC6242 (marking `6242I`), 16-pin SSOP | quad op-amp forming the active IF filter between jumpers J6/J7 and J8/J9 |
| U10 | Analog Devices AD7694ARMZ (brand `C4K`, A grade, ±6 LSB INL), 16-bit 250 kSPS SAR, 8-lead MSOP | IF digitiser |
| — | ADT7301-compatible temperature sensor (14-bit, 1/32 °C), location unknown | board temperature |
| A1, A2, A3 | chip attenuators marked `2` | RF pads |
| U14 | SOIC-8, unread | presumed loop-filter amplifier |
| U11 | Linear Technology SOT-23-6 (marking `LTAHG`), with inductor L2 | presumed switching supply |
| U12 | Linear Technology MSOP-8 (marking `LTDTF`) beside U10 | presumed AD7694 reference (5 V input span) |
| U8, U9 | SOIC-8, unread | U8 with volume trimmer R18: speaker amplifier, no connection to U4 |
| U20, U21, U22, U23, U25, U26, U27 | unread | remaining RF chain: amplifiers, splitter, switch |

Signal path: U13 → U15/U17 (swept source) → amplifiers and splitter → TX
connector, and → LO of U24; RX connector → amplifier → U24 → U16 → ADC → U4.

## 1. Controller clock

| Item | Value | Conf |
|---|---|---|
| Oscillator | primary oscillator, HS mode, 12 MHz on OSCI (pin 39) / OSCO (pin 40) | H |
| PLL | 12 MHz ÷ 3 → 4 MHz PLL input → 96 MHz; CPU divider 1 | H |
| FOSC / FCY | 32 MHz / 16 MHz | H |
| USB clock | 48 MHz from the same PLL | H |
| Configuration words | CW1 `0x3F10` (watchdog off, JTAG off, ICS on PGEC1/PGED1, no code protection), CW2 `0xA33A` (HSPLL, ÷3, two-speed start-up, clock switching and fail-safe monitor enabled, USB regulator disabled, IOL1WAY), CW3 `0xFFFF` | H |
| VUSB | on-chip USB regulator disabled: VUSB (pin 35) is supplied by the board | H |

U4's clock and the synthesiser reference (U13) are independent oscillators.
All controller timing, including the ADC sample clock (§4.2), is derived
from U4's 12 MHz source; all frequency and sweep timing from U13.

## 2. U4 pin map

Pins not listed are unused outputs, except the supplies.

| Pin | Port | Dir | Idle | Function | Conf |
|---|---|---|---|---|---|
| 1 | RE5 | out | 1 | D3 (the LED beside the `USB` silkscreen), active low: lit while the USB device is configured | H |
| 11 | RB5 | out | 0 | **RF enable, active low** (§4.6) | M |
| 14 | RB2 | out | 0 | Bluetooth module control line, pulsed during module set-up | L |
| 15 | RB1 | in | — | PGEC1, J1 | H |
| 16 | RB0 | in | — | PGED1, J1 | H |
| 17, 18, 21, 22 | RB6, RB7, RB8, RB9 | out | 0 | XJ1 even pins; RB6/RB7 are PGEC2/PGED2; no other use | H |
| 28 | RB13 | in | — | not used | H |
| 29 | RB14 (RP14) | in | — | SDI2 ← synthesiser SDO | H |
| 30 | RB15 (RP29) | out | — | SCK2 → synthesiser SCK | H |
| 33 | RF3 (RP16) | out | — | SDO2 → synthesiser SDI | H |
| 36, 37 | RG3, RG2 | — | — | USB D−, D+ | H |
| 39, 40 | RC12, RC15 | — | — | 12 MHz oscillator | H |
| 42 | RD8 (RP2) | out | — | U1RTS (mapped, hardware flow control off) | H |
| 44 | RD10 (RP3) | out | — | U1TX → Bluetooth RX | H |
| 45 | RD11 (RP12) | in | — | U1RX ← Bluetooth TX | H |
| 46 | RD0 (RP11) | out | — | U1CTS (mapped, flow control off) | H |
| 49 | RD1 | out | 1 | EEPROM CS, active low | H |
| 50 | RD2 | out | 0 | synthesiser SEN, active high | H |
| 51 | RD3 (RP22) | out | — | SDO1 → EEPROM, LED driver, temperature sensor | H |
| 52 | RD4 (RP25) | in | — | SDI1 ← EEPROM, temperature sensor, ADC | H |
| 53 | RD5 (RP20) | out | — | SCK1 | H |
| 54 | RD6 | out | 1 | temperature sensor CS, active low | H |
| 55 | RD7 | out | 0 | ADC CNV (convert start / read enable) | H |
| 63 | RE3 | in | — | any level change restarts the controller; routed through a via, source not traced; may be ignored by a replacement program | M |
| 64 | RE4 | out | 1 | LED driver CS, active low | H |

All analogue inputs are configured digital (AD1PCFGL = `0xFFFF`); the
on-chip ADC, timers, output compare, input capture, I²C, PMP and RTCC are
unused. Peripheral pin select is written once (IOL1WAY).

## 3. Buses

| Bus | Mode | Clock | Devices |
|---|---|---|---|
| SPI1 | master, 8-bit transfers, mode 0 (CKP 0, CKE 1, SMP 0), polled | 4 MHz | EEPROM, LED driver, temperature sensor, ADC |
| SPI2 | master, 8-bit transfers, mode 0, polled | 1 MHz | synthesiser |
| UART1 | 8N1, BRGH, U1BRG 8 → 444 444 baud (3.5 % below 460 800), no flow control, receive interrupt | — | Bluetooth module |
| USB | full-speed device, on-chip transceiver | — | host |

USB identity: VID `0x2012`, PID `0x0013`, bcdDevice 1.00, manufacturer
string `Quonset Micro`, product `QM4004`, serial string = EEPROM serial
number as 4 decimal digits. One configuration (self-powered attribute set),
one interface class `0xFE`/`0x03`/`0x01` (USBTMC, USB488): bulk OUT `0x01`
and bulk IN `0x81`, 64 bytes; interrupt IN `0x82`, 2 bytes. Protocol on top:
[protocol.md](protocol.md).

## 4. Devices

### 4.1 Synthesiser (U15, HMC703) on SPI2

Interface: Hittite **HMC Mode** serial port. A read issued at power-up with
SEN raised before the first SCK edge selects HMC Mode. Frames are 32 bits,
MSB first, sent as four bytes with SEN high around them (50 µs set-up and
hold are used):

```
write: [31] 0  [30:25] register  [24:1] data (24 bits)  [0] 0
read:  first byte 1 a5..a0 0, then three 0x00 bytes; data = the 24 bits
       following the address (bit 0 of byte 1 is data bit 23)
```

Power-up sequence: SEN high 1 s, low; then the registers below, in order.

| Reg | Value | Use | Conf |
|---|---|---|---|
| 01h | `0x000002` | chip enable via SPI | H value, M meaning |
| 02h | R | reference divider, default 1 | H |
| 06h | `0x001D3F` + mode | sigma-delta configuration; bits 7:5 select the operating mode (0 fractional CW, 1 integer CW, 5/6/7 sweeper single ramp / single triangle / continuous triangle) | H value, M meaning |
| 07h | `0x004865`, then `0x104865` | lock-detect configuration; bit 20 is cleared and set again (50 µs apart) after every change of R; bit 15 enables cycle-slip prevention | H value, M meaning |
| 08h | `0x016FFF` | analogue enables and bias | H |
| 09h | `0x803264` (CW: `0x9AF264`) | charge pump: down gain bits 6:0, up gain bits 13:7 (100 each), offset bits 20:14, source/sink offset enables bits 21/22, high-current bit 23 | H layout, M units |
| 0Bh | `0x01EC71` | fixed | H |
| 0Fh | `0x000013` | GPO / SDO configuration (lock detect readable over SPI) | H value, L meaning |
| 0Eh | 0, then 1 | written 0 before and 1 after every reprogramming; a 0 → 1 edge (5 µs) is the software sweep trigger | H sequence, M meaning |

Frequency words, with `N = f × R / f_ref` (f_ref 20 MHz, f the VCO frequency):
register 03h = ⌊N⌋, register 04h = frac(N) · 2²⁴.

Sweeper set-up: the ramp is divided into `steps = T × f_ref` reference
periods (M). Register 0Ah holds the per-step increment `(N_stop − N_start) /
steps · 2²⁴`, truncated to an integer; registers 0Ch/0Dh hold the integer and
fractional parts of the end point recomputed from the truncated increment,
and 03h/04h the start (H). The swept bandwidth is therefore at most the
requested `f_stop − f_start` and falls short of it by up to one increment
LSB per step, which matters when the increment is small (long ramps, small
bandwidth). If start > stop the two are exchanged. Start and stop are limited to
2400–2500 MHz.

Lock state is read from register 12h bit 1; register 03h/04h read back give
the actual frequency. No lock-detect pin is wired to U4.

### 4.2 IF ADC on SPI1

| Item | Value | Conf |
|---|---|---|
| Part | AD7694: pseudo-differential input, IN+ 0 V to VREF relative to IN− (ground sense), straight binary output; VREF up to VDD, 5 V here (5 V span) | H part, M VREF |
| Conversion | rising edge of CNV (RD7) starts a conversion and tri-states SDO; falling edge of CNV puts the MSB on SDO; the other 15 bits follow on SCK falling edges (16 SCK1 clocks, no leading zeros) | H |
| CNV high time | 2 µs on this board, below the AD7694's maximum tCONV of 3.2 µs (VDD 4.75–5.25 V; 4.66 µs below 4.75 V). A replacement program should hold CNV high for at least tCONV | H |
| Word | 16 bits, unsigned, mid-scale at zero IF (IF biased to VREF/2); all 16 bits are live in recorded frames | H |
| Sample period | instruction-timed from U4's 16 MHz clock: 2 µs convert, 16-bit read, 36 µs wait and loop overhead; measured 21 977 Hz = 16 MHz / 728 (45.5 µs) | H measured, M cycle count |
| Frame | up to 4096 words held in U4 RAM | H |

The sample clock is a fixed number of controller cycles, so its accuracy is
that of U4's 12 MHz source, and it is not locked to U13: the
ADC-to-sweep-clock ratio drifts with the difference of the two oscillator
errors (tens of ppm, temperature dependent).

### 4.3 Temperature sensor on SPI1

CS RD6 low, 10 µs, read two bytes (0x00 sent), 10 µs, CS high. 14-bit two's
complement, `T = code / 32 °C` (ADT7301 format). This is the only
temperature measurement on the board. (H)

### 4.4 EEPROM (U1, 25AA1024) on SPI1

Instructions: `0xAB` + 3 dummy bytes → signature (must read `0x29`), `0x05`
RDSR (poll WIP, check WEL), `0x06` WREN, `0x01` WRSR (clears block
protection when WEL does not set), `0x03` READ and `0x02` WRITE with 24-bit
addresses (17 significant bits); writes are split at 256-byte page
boundaries.

| Address | Content | Conf |
|---|---|---|
| `0x0000` + 5k | directory entry k: k, data address (16-bit little-endian), length | H |
| `0x00FE` | `0x6F` = formatted; any other value → defaults are written | H |
| `0x0100` | serial number, uint16 little-endian, < 10000 (USB serial string, `*IDN?`) | H |
| `0x0102` | power-up state: slot number, then `0x01` = valid | H |
| `0x010C` + … | saved states, 22 bytes: +0 PLL mode (0 fractional, 1 integer), +2 CW frequency (float32, MHz), +6 R (uint16), +8 RF on, +10 sweep start (float32, MHz), +14 sweep stop (float32, MHz), +18 ramp time (float32, ms) | H |

Slot n is written at `0x010C + 0x28·n` but the directory places it at
`0x010C + 0x14·n` and a 22-byte record does not fit a 20-byte stride; read
the EEPROM before relying on stored slots. (H; unverified against a dump)

### 4.5 LED driver (U2, MAX6966) on SPI1

16-bit frames, CS RE4 low for the frame with 5 µs set-up and hold: register
byte (bit 7 = 0, write), data byte. Configuration register `0x10` ← `0x01`
(run) at power-up. Port registers `0x00`–`0x09` drive light-bar segments
P0–P9 in order: `0x00` lit, `0xFF` off. (H; polarity M)

### 4.6 RF enable

RB5, active low, gates the transmit chain: driven low to transmit, high for
RF off. The board powers up with RF enabled. (M)

### 4.7 Bluetooth module (U6) on UART1

iWRAP configuration applied at every power-up, each line ended with CR LF
and followed by 100 ms: `SET BT SSP 0 0`, `SET BT AUTH * 0000`,
`SET BT NAME RDK <serial>`, `SET CONTROL BAUD 460800,8n1`, then `RESET`
(500 ms), with RB2 pulsed around the sequence. The module has to be at
460 800 baud already for the commands to be understood. The module reports
link loss as `NO CARRIER 0 ERROR`. (H strings, L RB2)

## Modifications

Present on this board (by the owner): speaker removed; audio tap on a
two-pin connector at the IF filter output (J7 side) feeding channel 1 of a
USB audio interface, with its ground lead clipped to the GND test point; MCX
connectors P1 and P2 capped.

## Open items for a replacement controller program

* ADC reference voltage and U12's part number (measure VREF on U10 pin 1).
* Temperature sensor location and part.
* HMC703 register semantics are from the register layout and behaviour;
  check 06h, 07h, 08h, 0Bh and 0Fh against the HMC703 datasheet.
* The Bluetooth module (§4.7, RB2) is not needed for USB-only control and
  can be left unconfigured.
* EEPROM slot stride (§4.4), from a dump of U1.
