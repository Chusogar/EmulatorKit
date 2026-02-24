/*
 * ay8912.c – AY-3-8912 (PSG) emulation for ZX Spectrum 128K/+3.
 *
 * Thin wrapper around the emu2149 library.  All three tone channels are
 * mixed into a single mono sample by PSG_calc(); no external mixing is
 * required.
 */

#include <stdlib.h>
#include "ay8912.h"
#include "emu2149/emu2149.h"

struct ay8912 {
    PSG *psg;
};

ay8912_t *ay8912_create(uint32_t psg_clock, uint32_t sample_rate)
{
    ay8912_t *ay = malloc(sizeof(ay8912_t));
    if (!ay)
        return NULL;
    ay->psg = PSG_new(psg_clock, sample_rate);
    if (!ay->psg) {
        free(ay);
        return NULL;
    }
    /* Use the authentic AY-3-8910 non-linear volume table. */
    PSG_setVolumeMode(ay->psg, EMU2149_VOL_AY_3_8910);
    PSG_set_quality(ay->psg, 0);
    PSG_reset(ay->psg);
    return ay;
}

void ay8912_destroy(ay8912_t *ay)
{
    if (!ay)
        return;
    PSG_delete(ay->psg);
    free(ay);
}

/* OUT 0xFFFD: latch register address. */
void ay8912_select_reg(ay8912_t *ay, uint8_t reg)
{
    PSG_writeIO(ay->psg, 0, reg);   /* adr=0 → address strobe */
}

/* OUT 0xBFFD: write data to the latched register. */
void ay8912_write_data(ay8912_t *ay, uint8_t val)
{
    PSG_writeIO(ay->psg, 1, val);   /* adr=1 → data write */
}

/* IN 0xFFFD: read data from the latched register. */
uint8_t ay8912_read_data(ay8912_t *ay)
{
    return PSG_readIO(ay->psg);
}

/* Step one output sample; returns mono mix in [0, AY8912_MAX_OUTPUT]. */
int16_t ay8912_calc(ay8912_t *ay)
{
    return PSG_calc(ay->psg);
}