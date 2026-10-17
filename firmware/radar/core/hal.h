/* Hardware interface the portable core calls; implemented by pic/ on the
   board and by test/ on the host. */
#ifndef RADAR_HAL_H
#define RADAR_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RF enable (RB5, active low): true transmits. */
void hal_rf(bool on);

/* HMC703 HMC-mode frames on SPI2 (sample clock stopped only). */
void hal_synth_write(uint8_t reg, uint32_t data);
uint32_t hal_synth_read(uint8_t reg);
/* Raise SEN and clock out the first 24 bits of a write frame; the next
   sample ISR sends the last byte right after its CNV rising edge, which
   registers the frame (32nd SCK rising edge), and lowers SEN in the ISR
   after that. Sample clock stopped only. */
void hal_synth_load(uint8_t reg, uint32_t data);

/* Sample clock: INT1 on RP14 (the HMC703 GPO), one ADC conversion per edge.
   Start clears any stale edge; stop leaves SEN low and SPI2 idle. */
void hal_clock_start(void);
void hal_clock_stop(void);
/* Rising edges counted on RP14 (Timer2 external clock) over `ms`. */
uint32_t hal_edges(uint16_t ms);
/* Level of RP14 (HMC703 LD_SDO). */
bool hal_gpo_level(void);

/* SPI1 housekeeping (sample clock stopped only). */
uint16_t hal_temp_raw(void);
/* 25AA1024: release from deep power-down, check the 0x29 signature, read. */
bool hal_eeprom_read(uint32_t addr, uint8_t *buf, size_t n);

void hal_delay_us(uint16_t us);
uint32_t hal_ms(void);
void hal_reset(void);

#ifdef __cplusplus
}
#endif

#endif
