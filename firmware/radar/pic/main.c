/* QM4004 radar controller entry point. */
#include <xc.h>

#include "board.h"
#include "radar.h"
#include "tusb.h"

/* CW1 0x3F10, CW2 0xA33A (stock): 12 MHz HS crystal, PLL /3 -> 96 MHz,
   FCY 16 MHz; on-chip USB regulator off (VUSB from the board). */
#pragma config JTAGEN = OFF, GCP = OFF, GWRP = OFF, BKBUG = OFF, ICS = PGx1
#pragma config FWDTEN = OFF, WINDIS = ON, FWPSA = PR128, WDTPS = PS1
#pragma config IESO = ON, PLLDIV = DIV3, PLL_96MHZ = ON, FNOSC = PRIPLL
#pragma config FCKSM = CSECME, OSCIOFNC = OFF, IOL1WAY = ON, DISUVREG = OFF
#pragma config POSCMOD = HS

int main(void) {
  static const tusb_rhport_init_t usb = {.role = TUSB_ROLE_DEVICE,
                                         .speed = TUSB_SPEED_FULL};
  board_init();
  radar_init();
  tusb_init(0, &usb);
  for (;;) {
    tud_task();
    radar_poll();
    usb_poll();
    LATEbits.LATE5 = !tud_mounted();
  }
}
