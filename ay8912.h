#ifndef AY8912_H
#define AY8912_H

#include <stdint.h>

/*
 * AY-3-8912 (PSG) emulation for ZX Spectrum 128K/+3.
 * Wraps the emu2149 library (EMU2149_VOL_AY_3_8910 table).
 *
 * The PSG is clocked at CPU_CLK/2 (~1.7735 MHz on ZX Spectrum 128K/+3)
 * and is stepped sample-by-sample at the host audio output rate via
 * ay8912_calc(), which must be called once per output sample inside the
 * audio advance loop (beeper_advance_to).
 *
 * Port usage (128K/+3):
 *   OUT 0xFFFD  -> ay8912_select_reg() : latch AY register address
 *   OUT 0xBFFD  -> ay8912_write_data() : write data to latched register
 *   IN  0xFFFD  -> ay8912_read_data()  : read data from latched register
 *
 * In all three cases the caller must advance the audio engine to the
 * current t-state (beeper_advance_to) BEFORE calling these functions so
 * that the register change is applied at the correct point in the audio
 * stream.
 */

typedef struct ay8912 ay8912_t;

ay8912_t *ay8912_create(uint32_t psg_clock, uint32_t sample_rate);
void      ay8912_destroy(ay8912_t *ay);

/* OUT 0xFFFD: latch register address (R0..R15). */
void    ay8912_select_reg(ay8912_t *ay, uint8_t reg);

/* OUT 0xBFFD: write value to the currently selected register. */
void    ay8912_write_data(ay8912_t *ay, uint8_t val);

/* IN  0xFFFD: read value from the currently selected register. */
uint8_t ay8912_read_data(ay8912_t *ay);

/*
 * Step the PSG by one output sample and return the mono mixed value.
 * Output range: [0, AY8912_MAX_OUTPUT].
 */
int16_t ay8912_calc(ay8912_t *ay);

/*
 * Maximum value that ay8912_calc() can return
 * (three channels each at full AY-3-8910 volume: 0xFF<<4 * 3 = 12240).
 */
#define AY8912_MAX_OUTPUT  12240

#endif /* AY8912_H */