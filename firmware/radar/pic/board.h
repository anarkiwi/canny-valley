/* QM4004 (PIC24FJ256GB106) board layer: pins, buses and the USBTMC link. */
#ifndef RADAR_BOARD_H
#define RADAR_BOARD_H

/* Pins, peripheral pin select, SPI1/SPI2, the millisecond tick and the
   interrupt priorities; RF is off on return. */
void board_init(void);

/* Serve a deferred bulk-IN request once the core has data for it. */
void usb_poll(void);

#endif
