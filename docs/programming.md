# Programming the controller with an MPLAB PICkit 5

The QM-RDK controller is U4, a Microchip PIC24FJ256GB106-I/PT (64-pin TQFP,
0.5 mm pitch). It is programmed and debugged over ICSP with low-voltage
entry: the programmer pulses MCLR and clocks a key sequence in on one of the
three PGEC/PGED pairs. No 12 V VPP is involved.

Pin numbers below are from the PIC24FJ256GB110 family datasheet (DS39897),
64-pin pin diagram. Routing on the board is read from the photo in
[`images/pic24-icsp-area.jpg`](images/pic24-icsp-area.jpg) and is marked
**traced** (visible top-layer copper), **inferred** (consistent with the
copper but not followed end to end), or **untested**. No vendor schematic is
available; confirm with a meter before relying on anything but the direct
connection.

## PICkit 5 connector

8-pin single row, pin 1 at the white arrow.

| PICkit 5 pin | ICSP signal | PIC24FJ256GB106 pin (channel 1) | (channel 2) | (channel 3) |
|---|---|---|---|---|
| 1 | MCLR / VPP | 7 MCLR | 7 | 7 |
| 2 | VDD (target sense) | 10 VDD (also 26, 38) | | |
| 3 | GND | 9 VSS (also 25, 41) | | |
| 4 | PGD (data) | 16 PGED1 / RB0 | 18 PGED2 / RB7 | 12 PGED3 / RB4 |
| 5 | PGC (clock) | 15 PGEC1 / RB1 | 17 PGEC2 / RB6 | 11 PGEC3 / RB5 |
| 6, 7, 8 | not used (TAUX, TTDI, TTMS) | | | |

The channel is selected in MPLAB X under the project's PICkit 5 properties
("Communication", ICSP channel). Any pair works for programming; the pair
only has to match the wiring.

## Direct connection

Viewed as in the photo: part marking upright, pin-1 dot at the bottom-left
corner, pins counted anticlockwise, so pins 1–16 run left to right along the
bottom edge and 17–32 bottom to top up the right edge.

```
MPLAB PICkit 5                       U4  PIC24FJ256GB106-I/PT, 64-pin TQFP
8-pin SIL, white arrow = pin 1       (pin-1 dot bottom-left, anticlockwise)
 +---+---+---+---+---+---+---+---+
 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |          48 ................................. 33
 +---+---+---+---+---+---+---+---+       49 +-----------------------------------+ 32
   |   |   |   |   |   nc  nc  nc           |                                   |
   |   |   |   |   |                        |            MICROCHIP              |- 22 RB9   -> XJ1 area
   |   |   |   |   |                        |         PIC24FJ256GB106           |- 21 RB8   -> XJ1 area
   |   |   |   |   |                        |                                   |- 18 PGED2 -> XJ1 area
   |   |   |   |   |                        |   o <- pin 1 dot                  |- 17 PGEC2 -> XJ1 area
   |   |   |   |   |                     64 +-----------------------------------+
   |   |   |   |   |                          1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16
   |   |   |   |   |                                            |     |  |              |  |
   |   |   |   |   |                                           MCLR  VSS VDD       PGEC1  PGED1
   |   |   |   |   |                                            |     |  |              |  |
   |   |   |   |   +--- PGC (ICSPCLK) --------------------------|-----|--|--------------+  |
   |   |   |   +------- PGD (ICSPDAT) --------------------------|-----|--|-----------------+
   |   |   +----------- GND -----------------------------------|-----+  |
   |   +--------------- VDD sense (3.3 V from the board) -------|--------+
   +------------------- MCLR / VPP -----------------------------+
```

Power the board from its own USB supply and set the PICkit 5 not to power
the target: the PIC is a 3.3 V part and pin 2 is a sense line. The fuse F1
beside the header is in the board's supply path.

## XJ1 as a programming header

XJ1 is an unpopulated 2×4 0.1" header beside U4, pin 1 the square pad at the
bottom right, odd pins up the right-hand column and even pins up the left:

```
          XJ1 (component side, as photographed)
           +-----------+
     left  |  8  :  7  |  right
   column  |  6  :  5  |  column
           |  4  :  3  |
           |  2  : [1] |   [1] = square pad
           +-----------+
```

What the photo shows:

| Net | Observation | Status |
|---|---|---|
| U4 pin 17 PGEC2 | leaves the pin to the right, runs under F1 to the XJ1 area at the height of rows 2/1 | traced to F1, inferred beyond |
| U4 pin 18 PGED2 | same run, one trace above pin 17's | traced to F1, inferred beyond |
| U4 pin 21 RB8 (RP8) | runs right under F1; a trace at that height bends up into XJ1 pin 6 | inferred |
| U4 pin 22 RB9 (RP9) | runs right under F1; a trace at that height bends up into XJ1 pin 8 | inferred |
| U4 pins 19 AVDD, 20 AVSS | no top-layer trace leaves the pin | — |
| U4 pin 7 MCLR | trace runs down to the top pad of R9 | traced |
| U4 pin 10 VDD | trace runs down to R10 | traced |
| U4 pin 16 PGED1 | trace runs down-right to a via | traced to the via |
| R12, R13 | both 10k (`103`); R13's lower pad goes to a via under F1, R12's lower pad is not visible | partial |
| XJ1 pin 7 | trace leaves upward | traced, destination unknown |
| XJ1 pin 3 | trace leaves to the right | traced, destination unknown |

Pins 17, 18, 21 and 22 are RP6–RP9, a block of four remappable pins, so XJ1
may be an expansion header that happens to carry the channel-2 ICSP pair
rather than a dedicated programming header. Either way it is usable for
programming on channel 2 if MCLR, VDD and GND are also present.

### Meter checklist

Board unpowered, continuity mode. Fill in the XJ1 pin each net lands on, or
"none".

| Net | Probe at | XJ1 pin |
|---|---|---|
| PGEC2 | U4 pin 17 | |
| PGED2 | U4 pin 18 | |
| RB8 | U4 pin 21 | |
| RB9 | U4 pin 22 | |
| MCLR | U4 pin 7, or the R9 pad nearest U4 | |
| VDD | U4 pin 10 | |
| GND | U4 pin 9 or the USB shell | |
| PGEC1 | U4 pin 15 | |
| PGED1 | U4 pin 16 | |

Also note which XJ1 pins R12 and R13 pull up, and to what: measure resistance
from each XJ1 pin to VDD; 10k identifies a pull-up through R12 or R13.

With MCLR, VDD, GND, PGEC2 and PGED2 all on XJ1, fit a header and wire the
PICkit 5 to those five pins, channel 2 selected. Otherwise use the direct
connection above.
