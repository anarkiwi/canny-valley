/* TinyUSB configuration: PIC24 full-speed device, one USBTMC/USB488
   interface. */
#ifndef RADAR_TUSB_CONFIG_H
#define RADAR_TUSB_CONFIG_H

#define CFG_TUSB_MCU OPT_MCU_PIC24
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_MEM_ALIGN __attribute__((aligned(2)))

#define CFG_TUD_USBTMC 1
#define CFG_TUD_USBTMC_ENABLE_INT_EP 1
#define CFG_TUD_USBTMC_ENABLE_488 1

#ifdef __XC16__
/* dcd_pic.c needs __delay_ms (remote wakeup) and, in its idle handler, the
   PIC32 set-alias of U1OTGIE; ACTVIE is the only U1OTGIE bit it uses, so a
   plain write is equivalent. */
#include <libpic30.h>
#define U1OTGIESET U1OTGIE
#endif

/* dcd_pic.c still has one dcd_edpt_xfer call without the is_isr argument;
   supply false for it. The parenthesised name is not re-expanded, so the
   prototype, the definition and five-argument calls stay unchanged. */
#define TU_DCD_XFER_ARGS(_1, _2, _3, _4, _5, call, ...) call
#define dcd_edpt_xfer(...)                                                     \
  TU_DCD_XFER_ARGS(__VA_ARGS__, (dcd_edpt_xfer)(__VA_ARGS__),                  \
                   (dcd_edpt_xfer)(__VA_ARGS__, false), ~)

#endif
