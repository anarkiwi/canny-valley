# QM-RDK board notes

Parts and features identified on the owner's board (silkscreen `QM4004-R3`,
serial 0021, firmware V1.1.0). No vendor schematic or parts list is
available; entries marked "unread" need a close-up of the package marking.

| Ref | Part | Role |
|-----|------|------|
| U4 | Microchip PIC24FJ256GB106-I/PT (64-pin TQFP; 256 KB flash, 16 KB RAM, full-speed USB, remappable peripheral pins) | controller |
| U1 | Microchip 25AA-series SPI EEPROM, 8-pin (density unread) | non-volatile storage outside the controller, presumably the saved instrument states |
| U3 | oscillator module | controller clock |
| J1, XJ1, XJ3 (2×4), XJ2 (2×3) | headers (XJ unpopulated) | unknown |
| U2 | TSSOP, marking begins `MAX6` | presumed light-bar LED driver |
| U6 | shielded module | Bluetooth radio |
| U13 | oscillator module | synthesiser reference (20 MHz per the manual) |
| U15 | Hittite HMC703LP4E (marking `H703`) | fractional-N synthesiser with built-in triggered frequency sweeper; its one-way / two-way, triggered / automatic sweep modes correspond to the board's sweep types |
| U17 | Hittite HMC385LP4 (marking `H385`) | 2.25–2.5 GHz VCO |
| U24 | Mini-Circuits SYM-25DMHW+ | receive mixer (40–2500 MHz, +13 dBm LO class) |
| U16 | Linear Technology LTC6242 (marking `6242I`), 16-pin SSOP | quad op-amp forming the active IF filter between jumpers J6/J7 and J8/J9 |
| A1, A2, A3 | chip attenuators marked `2` | RF pads |
| U14 | SOIC-8, unread | presumed loop-filter amplifier |
| U10 | 8-pin MSOP, Analog Devices logo, unread | presumed ADC |
| U11, U12 | Linear Technology parts (markings `LTAHC`, `LTDTF`), with inductor L2 | presumed supply conversion and reference |
| U8, U9 | SOIC-8, unread | U8 with volume trimmer R18: speaker amplifier |
| U20, U21, U22, U23, U25, U26, U27 | unread | remaining RF chain: amplifiers, splitter, switch |

Signal path: U13 → U15/U17 (swept source) → amplifiers and splitter → TX
connector, and → LO of U24; RX connector → amplifier → U24 → U16 → ADC → U4.

The 16 KB of controller RAM accounts for the 4096-sample (8 KB) limit of
`CAPT:FRAM`.

Modifications present on this board (by the owner): speaker removed; audio
tap on a two-pin connector at the IF filter output (J7 side) feeding channel 1
of a USB audio interface, with its ground lead clipped to the GND test
point; MCX connectors P1 and P2 capped.
