/* hal.h on the QM4004: SPI1 (EEPROM, temperature sensor, ADC), SPI2
   (HMC703, HMC mode), the INT1 sample clock and the timers. FCY 16 MHz. */
#include <xc.h>

#include <libpic30.h>

#include "board.h"
#include "hal.h"
#include "radar.h"

#define RF_N LATBbits.LATB5
#define EE_CS LATDbits.LATD1
#define SEN LATDbits.LATD2
#define TEMP_CS LATDbits.LATD6
#define CNV LATDbits.LATD7

/* SPIxCON1 (same layout on both ports): master, mode 0 (CKP 0, CKE 1);
   SCK = FCY / (primary * secondary prescale). */
#define SPI_MODE0 (_SPI1CON1_MSTEN_MASK | _SPI1CON1_CKE_MASK)
#define SPI_4MHZ ((7u << _SPI1CON1_SPRE_POSITION) | 2u)
#define SPI_8MHZ ((6u << _SPI1CON1_SPRE_POSITION) | 3u)
#define SPI1_HK (SPI_MODE0 | SPI_4MHZ)
#define SPI1_ADC (SPI_MODE0 | SPI_8MHZ | _SPI1CON1_MODE16_MASK)
/* The HMC703 drives SDO after the SCK rising edge; sample on the falling
   edge (SMP = 1). */
#define SPI2_HMC (SPI_MODE0 | SPI_8MHZ | _SPI1CON1_SMP_MASK)

/* PPS output function codes and input pins. */
#define PPS_SDO1 7u
#define PPS_SCK1OUT 8u
#define PPS_SDO2 10u
#define PPS_SCK2OUT 11u
#define RP_SDI1 25u
#define RP_GPO 14u

/* SPI2 frame tail, owned by the sample ISR while INT1 is enabled: the last
   byte of a loaded write frame (TAIL_SEND | byte), or TAIL_DONE once it has
   been sent and SEN is still high. */
#define TAIL_SEND 0x0100u
#define TAIL_DONE 0x0200u
static volatile uint16_t spi2_tail;
static volatile uint32_t ms_count;

static void spi1_config(uint16_t con1) {
  SPI1STAT = 0;
  SPI1CON1 = con1;
  SPI1CON2 = 0;
  SPI1STAT = _SPI1STAT_SPIEN_MASK;
}

static uint8_t spi1_byte(uint8_t b) {
  SPI1BUF = b;
  while (!SPI1STATbits.SPIRBF)
    ;
  return (uint8_t)SPI1BUF;
}

/* Collect the four bytes clocked in by a frame (the RX FIFO holds nothing
   else); the last arrives after the 32nd SCK edge. */
static uint32_t spi2_rx4(void) {
  uint32_t r = 0;
  uint8_t i;
  for (i = 0; i < 4; i++) {
    while (SPI2STATbits.SRXMPT)
      ;
    r = r << 8 | (uint8_t)SPI2BUF;
  }
  return r;
}

static uint32_t spi2_frame(uint32_t f) {
  uint32_t r;
  SEN = 1;
  SPI2BUF = (uint8_t)(f >> 24);
  SPI2BUF = (uint8_t)(f >> 16);
  SPI2BUF = (uint8_t)(f >> 8);
  SPI2BUF = (uint8_t)f;
  r = spi2_rx4();
  SEN = 0;
  return r;
}

static uint32_t hmc_write_frame(uint8_t reg, uint32_t data) {
  return (uint32_t)(reg & 0x3Fu) << 25 | (data & 0xFFFFFFul) << 1;
}

void board_init(void) {
  RF_N = 1;
  TRISBbits.TRISB5 = 0;
  AD1PCFGL = 0xFFFF;
  LATB = 1u << 5;
  LATC = 0;
  LATD = 1u << 1 | 1u << 6 | 1u << 10;
  LATE = 1u << 4 | 1u << 5;
  LATF = 0;
  LATG = 0;
  TRISB = 0x6003;
  TRISC = 0x1000;
  TRISD = 0x0810;
  TRISE = 0x0008;
  TRISF = 0x0044;
  TRISG = 0x000C;

  __builtin_write_OSCCONL(OSCCON & ~_OSCCON_IOLOCK_MASK);
  RPOR11bits.RP22R = PPS_SDO1;
  RPOR10bits.RP20R = PPS_SCK1OUT;
  RPINR20bits.SDI1R = RP_SDI1;
  RPOR8bits.RP16R = PPS_SDO2;
  RPOR14bits.RP29R = PPS_SCK2OUT;
  RPINR22bits.SDI2R = RP_GPO;
  RPINR0bits.INT1R = RP_GPO;
  RPINR3bits.T2CKR = RP_GPO;
  __builtin_write_OSCCONL(OSCCON | _OSCCON_IOLOCK_MASK);

  spi1_config(SPI1_HK);
  SPI2STAT = 0;
  SPI2CON1 = SPI2_HMC;
  SPI2CON2 = _SPI2CON2_SPIBEN_MASK;
  SPI2STAT = _SPI2STAT_SPIEN_MASK;

  _INT1EP = 0;
  _INT1IP = 7;
  _USB1IP = 4;
  T1CON = 0;
  TMR1 = 0;
  PR1 = FCY / 1000u - 1u;
  _T1IP = 1;
  _T1IF = 0;
  _T1IE = 1;
  T1CONbits.TON = 1;
}

void hal_rf(bool on) { RF_N = !on; }

void hal_synth_write(uint8_t reg, uint32_t data) {
  (void)spi2_frame(hmc_write_frame(reg, data));
}

uint32_t hal_synth_read(uint8_t reg) {
  return spi2_frame(1ul << 31 | (uint32_t)(reg & 0x3Fu) << 25) >> 1 &
         0xFFFFFFul;
}

void hal_synth_load(uint8_t reg, uint32_t data) {
  uint32_t f = hmc_write_frame(reg, data);
  SEN = 1;
  SPI2BUF = (uint8_t)(f >> 24);
  SPI2BUF = (uint8_t)(f >> 16);
  SPI2BUF = (uint8_t)(f >> 8);
  spi2_tail = TAIL_SEND | (uint8_t)f;
}

/* Sample clock edge. The CNV rising edge is the sample instant and the last
   byte of a loaded frame follows it directly. */
void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
  uint16_t t = spi2_tail;
  uint16_t w;
  uint8_t a;
  _INT1IF = 0;
  CNV = 0;
  Nop();
  SPI1BUF = 0;
  while (!SPI1STATbits.SPIRBF)
    ;
  w = SPI1BUF;
  CNV = 1;
  if (t & TAIL_SEND) {
    SPI2BUF = t;
    t = TAIL_DONE;
  } else if (t) {
    SEN = 0;
    while (!SPI2STATbits.SRXMPT)
      (void)SPI2BUF;
    t = 0;
  }
  a = radar_isr_step(w);
  if (a != SCHED_NONE) {
    SEN = 1;
    SPI2BUF = SCHED_B1;
    SPI2BUF = SCHED_B2;
    SPI2BUF = SCHED_B3;
    t = TAIL_SEND | SCHED_B4(a);
  }
  spi2_tail = t;
}

void hal_clock_start(void) {
  spi1_config(SPI1_ADC);
  _INT1IF = 0;
  _INT1IE = 1;
}

void hal_clock_stop(void) {
  _INT1IE = 0;
  if (spi2_tail & TAIL_SEND)
    SPI2BUF = spi2_tail;
  if (spi2_tail) {
    (void)spi2_rx4();
    SEN = 0;
    spi2_tail = 0;
  }
  spi1_config(SPI1_HK);
  CNV = 0;
}

uint32_t hal_edges(uint16_t ms) {
  uint32_t t0;
  uint32_t n;
  T2CON = _T2CON_T32_MASK | _T2CON_TCS_MASK;
  PR3 = 0xFFFF;
  PR2 = 0xFFFF;
  TMR3HLD = 0;
  TMR2 = 0;
  t0 = hal_ms();
  while (hal_ms() == t0)
    ;
  T2CONbits.TON = 1;
  t0++;
  while (hal_ms() - t0 < ms)
    ;
  T2CONbits.TON = 0;
  n = TMR2;
  return n | (uint32_t)TMR3HLD << 16;
}

bool hal_gpo_level(void) { return PORTBbits.RB14; }

uint16_t hal_temp_raw(void) {
  uint16_t v;
  TEMP_CS = 0;
  v = (uint16_t)spi1_byte(0) << 8;
  v |= spi1_byte(0);
  TEMP_CS = 1;
  return v;
}

bool hal_eeprom_read(uint32_t addr, uint8_t *buf, size_t n) {
  uint8_t sig;
  EE_CS = 0;
  spi1_byte(0xAB);
  spi1_byte(0);
  spi1_byte(0);
  spi1_byte(0);
  sig = spi1_byte(0);
  EE_CS = 1;
  if (sig != 0x29)
    return false;
  hal_delay_us(100);
  EE_CS = 0;
  spi1_byte(0x03);
  spi1_byte((uint8_t)(addr >> 16));
  spi1_byte((uint8_t)(addr >> 8));
  spi1_byte((uint8_t)addr);
  while (n--)
    *buf++ = spi1_byte(0);
  EE_CS = 1;
  return true;
}

void hal_delay_us(uint16_t us) {
  if (us)
    __delay32((unsigned long)us * (FCY / 1000000ul));
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
  _T1IF = 0;
  ms_count++;
}

uint32_t hal_ms(void) {
  uint32_t a;
  do
    a = ms_count;
  while (a != ms_count);
  return a;
}

void hal_reset(void) { __asm__ volatile("reset"); }
