/*
 *  ColecoVision VDP adapter (TMS9918A)
 *
 *  coleco_vdp.c
 *
 *  Port mapping:
 *    0xBE  data  (addr bit 0 == 0)
 *    0xBF  ctrl  (addr bit 0 == 1)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include "../../tms9918a.h"
#include "../../tms9918a_render.h"
#include "coleco_vdp.h"

static struct tms9918a          *vdp;
static struct tms9918a_renderer *vdp_rend;

static SDL_Window   *vdp_window;
static SDL_Renderer *vdp_render;
static SDL_Texture  *vdp_texture;

bool coleco_vdp_init(void)
{
    vdp = tms9918a_create();
    if (!vdp) {
        fprintf(stderr, "coleco_vdp: tms9918a_create failed\n");
        return false;
    }

    if (SDL_InitSubSystem(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "coleco_vdp: SDL video init failed: %s\n", SDL_GetError());
        return false;
    }

    vdp_window = SDL_CreateWindow("ColecoVision",
                                  SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED,
                                  320, 240, 0);
    if (!vdp_window) {
        fprintf(stderr, "coleco_vdp: SDL_CreateWindow: %s\n", SDL_GetError());
        return false;
    }

    vdp_render = SDL_CreateRenderer(vdp_window, -1,
                                    SDL_RENDERER_ACCELERATED |
                                    SDL_RENDERER_PRESENTVSYNC);
    if (!vdp_render) {
        fprintf(stderr, "coleco_vdp: SDL_CreateRenderer: %s\n", SDL_GetError());
        return false;
    }

    vdp_texture = SDL_CreateTexture(vdp_render,
                                    SDL_PIXELFORMAT_ARGB8888,
                                    SDL_TEXTUREACCESS_STREAMING,
                                    256, 192);
    if (!vdp_texture) {
        fprintf(stderr, "coleco_vdp: SDL_CreateTexture: %s\n", SDL_GetError());
        return false;
    }

    vdp_rend = tms9918a_renderer_create(vdp);
    if (!vdp_rend) {
        fprintf(stderr, "coleco_vdp: tms9918a_renderer_create failed\n");
        return false;
    }

    return true;
}

void coleco_vdp_shutdown(void)
{
    if (vdp_rend) {
        tms9918a_renderer_free(vdp_rend);
        vdp_rend = NULL;
    }
    if (vdp_texture) {
        SDL_DestroyTexture(vdp_texture);
        vdp_texture = NULL;
    }
    if (vdp_render) {
        SDL_DestroyRenderer(vdp_render);
        vdp_render = NULL;
    }
    if (vdp_window) {
        SDL_DestroyWindow(vdp_window);
        vdp_window = NULL;
    }
    if (vdp) {
        tms9918a_free(vdp);
        vdp = NULL;
    }
}

void coleco_vdp_reset(void)
{
    if (vdp)
        tms9918a_reset(vdp);
}

void coleco_vdp_render(void)
{
    if (!vdp || !vdp_rend)
        return;
    tms9918a_rasterize(vdp);
    tms9918a_render(vdp_rend);
}

int coleco_vdp_irq_pending(void)
{
    return vdp ? tms9918a_irq_pending(vdp) : 0;
}

/*
 * ColecoVision port map:
 *   read  0xBE → data port (addr bit 0 = 0)
 *   read  0xBF → status    (addr bit 0 = 1)
 *   write 0xBE → data      (addr bit 0 = 0)
 *   write 0xBF → control   (addr bit 0 = 1)
 */
uint8_t coleco_vdp_io_read(uint16_t port)
{
    if (!vdp)
        return 0xFF;
    /* tms9918a_read: addr 0 = data, addr 1 = status */
    return tms9918a_read(vdp, (uint8_t)(port & 1));
}

void coleco_vdp_io_write(uint16_t port, uint8_t val)
{
    if (!vdp)
        return;
    /* tms9918a_write: addr 0 = data, addr 1 = control */
    tms9918a_write(vdp, (uint8_t)(port & 1), val);
}

struct tms9918a *coleco_vdp_get(void)
{
    return vdp;
}
