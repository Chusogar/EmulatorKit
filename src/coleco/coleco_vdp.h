/*
 *  ColecoVision VDP adapter (TMS9918A) – header
 *
 *  coleco_vdp.h
 *
 *  ColecoVision VDP I/O ports:
 *    0xBE  –  data read/write
 *    0xBF  –  control/status read/write
 */

#ifndef COLECO_VDP_H
#define COLECO_VDP_H

#include <stdint.h>
#include <stdbool.h>
#include "../../tms9918a.h"
#include "../../tms9918a_render.h"

/* Initialise the VDP and its SDL2 renderer.  Returns true on success. */
bool coleco_vdp_init(void);

/* Release VDP resources. */
void coleco_vdp_shutdown(void);

/* Reset VDP to power-on state. */
void coleco_vdp_reset(void);

/* Rasterise the current frame and blit it to the SDL window. */
void coleco_vdp_render(void);

/* Returns non-zero if the VDP has a pending interrupt. */
int coleco_vdp_irq_pending(void);

/* I/O callbacks (delegated from coleco_io) */
uint8_t coleco_vdp_io_read(uint16_t port);
void    coleco_vdp_io_write(uint16_t port, uint8_t val);

/* Expose internal VDP pointer so callers can configure it further. */
struct tms9918a *coleco_vdp_get(void);

#endif /* COLECO_VDP_H */
