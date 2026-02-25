/*
 *  ColecoVision I/O port dispatch
 *
 *  coleco_io.c
 *
 *  Delegates port accesses to the appropriate sub-system (VDP, PSG, input).
 *  See coleco_io.h for the port map summary.
 */

#include <stdio.h>
#include <stdint.h>

#include "coleco_io.h"
#include "coleco_vdp.h"
#include "coleco_psg.h"

/*
 * Port read.
 * NOTE: Only the lower 8 bits of `port` are significant on the Z80 I/O bus
 *       for ColecoVision; the upper byte is ignored here.
 */
uint8_t coleco_io_read(int param, uint16_t port)
{
    (void)param;
    uint8_t p = (uint8_t)(port & 0xFF);

    /* VDP data/status: 0xBE (data) and 0xBF (status) */
    if (p == 0xBE || p == 0xBF)
        return coleco_vdp_io_read(p);

    /* Controller port 1 stub – returns 0xFF (no buttons pressed) */
    if (p == 0xFC)
        return 0xFF;

    /* Controller port 2 stub */
    if (p == 0xFF)
        return 0xFF;

    /* Unknown port */
    return 0xFF;
}

/*
 * Port write.
 */
void coleco_io_write(int param, uint16_t port, uint8_t val)
{
    (void)param;
    uint8_t p = (uint8_t)(port & 0xFF);

    /* VDP data: 0xBE, VDP control: 0xBF */
    if (p == 0xBE || p == 0xBF) {
        coleco_vdp_io_write(p, val);
        return;
    }

    /* PSG: 0xFF */
    if (p == 0xFF) {
        coleco_psg_write(val);
        return;
    }
}
