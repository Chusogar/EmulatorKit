/*
 *  ColecoVision emulator skeleton – public API
 *
 *  coleco.h
 */

#ifndef COLECO_H
#define COLECO_H

#include <stdbool.h>
#include <stdint.h>

/* ColecoVision hardware constants */
#define COLECO_CPU_HZ       3579545u    /* ~3.58 MHz Z80 */
#define COLECO_FPS          60u
#define COLECO_TSTATES_FRAME (COLECO_CPU_HZ / COLECO_FPS)

/* Screen geometry (TMS9918A native) */
#define COLECO_SCREEN_W     256
#define COLECO_SCREEN_H     192

/*
 * Initialise all subsystems (CPU, VDP, PSG, I/O).
 * Returns true on success.
 */
bool coleco_init(void);

/* Hard-reset the machine. */
void coleco_reset(void);

/* Execute one video frame worth of emulation and render to screen. */
void coleco_run_frame(void);

/* Release all resources. */
void coleco_shutdown(void);

#endif /* COLECO_H */
