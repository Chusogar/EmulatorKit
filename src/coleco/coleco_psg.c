/*
 *  ColecoVision PSG skeleton (SN76489)
 *
 *  coleco_psg.c
 *
 *  This wraps the sn76489 / emu76489 driver already present in the repo.
 *  Audio output is handled internally by sn76489_sdl (SDL2 audio queue).
 *
 *  TODO: integrate with the beeper/audio mixing used by other emulators in
 *        this repository once the skeleton is fleshed out.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "../../sn76489.h"
#include "coleco_psg.h"

static struct sn76489 *psg;

bool coleco_psg_init(void)
{
    psg = sn76489_create();
    if (!psg) {
        fprintf(stderr, "coleco_psg: sn76489_create failed\n");
        return false;
    }
    return true;
}

void coleco_psg_shutdown(void)
{
    if (psg) {
        sn76489_destroy(psg);
        psg = NULL;
    }
}

void coleco_psg_reset(void)
{
    /*
     * The SN76489 has no explicit reset pin; the common approach is to
     * mute all channels by writing 0x9F, 0xBF, 0xDF, 0xFF (volume = 0).
     */
    if (!psg)
        return;
    coleco_psg_write(0x9F);
    coleco_psg_write(0xBF);
    coleco_psg_write(0xDF);
    coleco_psg_write(0xFF);
}

void coleco_psg_write(uint8_t val)
{
    if (psg)
        sn76489_write(psg, val);
}
