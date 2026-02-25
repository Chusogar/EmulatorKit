/*
 *  ColecoVision emulator skeleton
 *
 *  coleco.c
 *
 *  Entry-point and main emulation loop.  Wires together:
 *    - Z80 CPU (libz80)
 *    - TMS9918A VDP  (coleco_vdp)
 *    - SN76489 PSG   (coleco_psg)
 *    - Memory map    (coleco_mem)
 *    - I/O dispatch  (coleco_io)
 *    - SDL2 event pump (event_sdl2)
 *
 *  Usage:
 *    coleco [-bios <bios.rom>] [-cart <cart.rom>]
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <SDL2/SDL.h>

#include "../../libz80/z80.h"
#include "../../event.h"

#include "coleco.h"
#include "coleco_mem.h"
#include "coleco_vdp.h"
#include "coleco_psg.h"
#include "coleco_io.h"

/* ─── Global CPU context ─────────────────────────────────────────────────── */
static Z80Context cpu_z80;

/* ─── Graceful-exit flag (set by SIGINT / SDL quit event) ─────────────────── */
static volatile int emulator_done;

/* ─── Signal handler ─────────────────────────────────────────────────────── */
static void sigint_handler(int sig)
{
    (void)sig;
    emulator_done = 1;
}

/* ─── coleco_init ─────────────────────────────────────────────────────────── */
bool coleco_init(void)
{
    /* Memory subsystem */
    coleco_mem_init();

    /* VDP + SDL2 window */
    if (!coleco_vdp_init())
        return false;

    /* PSG */
    if (!coleco_psg_init())
        return false;

    /* Z80 CPU */
    memset(&cpu_z80, 0, sizeof(cpu_z80));
    cpu_z80.memRead   = coleco_mem_read;
    cpu_z80.memWrite  = coleco_mem_write;
    cpu_z80.memParam  = 0;
    cpu_z80.ioRead    = coleco_io_read;
    cpu_z80.ioWrite   = coleco_io_write;
    cpu_z80.ioParam   = 0;
    Z80RESET(&cpu_z80);

    return true;
}

/* ─── coleco_reset ────────────────────────────────────────────────────────── */
void coleco_reset(void)
{
    Z80RESET(&cpu_z80);
    coleco_vdp_reset();
    coleco_psg_reset();
}

/* ─── coleco_run_frame ────────────────────────────────────────────────────── */
void coleco_run_frame(void)
{
    /* Execute one frame's worth of CPU cycles */
    Z80ExecuteTStates(&cpu_z80, COLECO_TSTATES_FRAME);

    /* Raise VDP interrupt (INT) if the VDP asserts its /INT line */
    if (coleco_vdp_irq_pending())
        Z80INT(&cpu_z80, 0xFF);

    /* Rasterise and present the frame */
    coleco_vdp_render();

    /* Pump SDL events */
    if (ui_event())
        emulator_done = 1;
}

/* ─── coleco_shutdown ─────────────────────────────────────────────────────── */
void coleco_shutdown(void)
{
    coleco_psg_shutdown();
    coleco_vdp_shutdown();
    SDL_Quit();
}

/* ─── main ────────────────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    const char *bios_path = NULL;
    const char *cart_path = NULL;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-bios") == 0 && i + 1 < argc)
            bios_path = argv[++i];
        else if (strcmp(argv[i], "-cart") == 0 && i + 1 < argc)
            cart_path = argv[++i];
        else {
            fprintf(stderr, "Usage: %s [-bios <bios.rom>] [-cart <cart.rom>]\n",
                    argv[0]);
            return 1;
        }
    }

    signal(SIGINT, sigint_handler);

    if (SDL_Init(0) < 0) {
        fprintf(stderr, "coleco: SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    /* Initialise all subsystems */
    if (!coleco_init()) {
        fprintf(stderr, "coleco: initialisation failed\n");
        SDL_Quit();
        return 1;
    }

    /* Load optional BIOS */
    if (bios_path)
        coleco_load_bios(bios_path);

    /* Load optional cartridge */
    if (cart_path)
        coleco_load_cart(cart_path);

    coleco_reset();

    fprintf(stderr, "coleco: entering main loop (Ctrl-C or close window to exit)\n");

    /* Main emulation loop */
    while (!emulator_done)
        coleco_run_frame();

    coleco_shutdown();
    return 0;
}
