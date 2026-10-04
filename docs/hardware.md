# QM-RDK board notes

Parts and features identified on the owner's board (silkscreen `QM4004-R3`,
serial 0021, firmware V1.1.0). No vendor schematic or parts list is
available; entries marked "unread" need a close-up of the package marking.

| Ref | Part | Role |
|-----|------|------|
| U4 | Microchip PIC24FJ256GB106-I/PT (64-pin TQFP; 256 KB flash, 16 KB RAM, full-speed USB, remappable peripheral pins) | controller |
| U1 | Microchip 25AA-series SPI EEPROM, 8-pin (density unread) | non-volatile storage outside the controller, presumably the saved instrument states |
| U3 | oscillator module | controller clock |
| J1 | 6-pin header beside U4 | presumed in-circuit programming header (to be confirmed by continuity to the MCLR and PGEC/PGED pins of U4) |
| XJ1, XJ3 (2×4), XJ2 (2×3) | unpopulated headers | unknown |
| U2 | TSSOP, marking begins `MAX6` | presumed light-bar LED driver |
| U6 | shielded module | Bluetooth radio |
| U13 | crystal in the RF section | PLL reference (20 MHz per the manual) |
| U15, U17, U14, U22, U23, U27 | unread | RF chain: synthesiser, VCO, amplifiers, mixer |
| U24 | Mini-Circuits module (marking unread) | RF chain |
| U16 and surrounding passives | unread | IF filter between jumpers J6/J7 and J8/J9 |
| U10, U11, U12 | unread | candidates for the ADC and its reference/regulator |

The 16 KB of controller RAM accounts for the 4096-sample (8 KB) limit of
`CAPT:FRAM`.

Modifications present on this board: speaker removed; a two-wire cable
attached at a connector beside J7 and leaving the board; a flying lead
with a test clip near the IF filter; MCX connectors P1 and P2 capped.
