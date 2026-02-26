/*
 *	Phoenix arcade emulator – public API
 *
 *	Hardware: Intel 8080A @ 2 MHz, custom video (two 32×32 tile planes),
 *	          TMS36XX melody + discrete noise, 8-color palette.
 *
 *	See docs/PHOENIX-HARDWARE.md for a full hardware reference.
 */

#ifndef PHOENIX_H
#define PHOENIX_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Geometry / timing constants
 * ---------------------------------------------------------------------- */

#define PHOENIX_SCREEN_W	208	/* pixels wide  (monitor rotated 90°) */
#define PHOENIX_SCREEN_H	256	/* pixels tall */
#define PHOENIX_FPS		60	/* nominal frames per second          */

/* 8080A runs at 2 MHz; 2 000 000 / 60 ≈ 33 333 T-states per frame.      */
#define PHOENIX_TSTATES_PER_FRAME	33333

/* -------------------------------------------------------------------------
 * Subsystem state structures
 * ---------------------------------------------------------------------- */

/*
 * phoenix_video – mirrors of the on-board video registers that the caller
 * may inspect after each frame.
 */
struct phoenix_video {
	uint8_t  fg_map[0x400];	/* foreground tile map  (32×32)           */
	uint8_t  bg_map[0x400];	/* background tile map  (32×32)           */
	uint8_t  color_ram[0x400];	/* color / attribute RAM                  */
	uint8_t  bg_scroll;	/* background horizontal scroll (0–15)    */
	uint8_t  palette_sel;	/* palette register (written via port 04) */
};

/*
 * phoenix_sound – current state of the sound registers (read-only snapshot).
 */
struct phoenix_sound {
	uint8_t  melody_latch;	/* last byte written to port 0x05 */
	uint8_t  noise_latch;	/* last byte written to port 0x06 */
};

/*
 * phoenix_input – host-side input state.  Set the bit to 1 to assert the
 * corresponding active-low button.  The emulator inverts the bits before
 * presenting them to the 8080 I/O read.
 */
struct phoenix_input {
	uint8_t  p1;	/* bit mask: see PHOENIX_P1_* below */
	uint8_t  p2;	/* bit mask: see PHOENIX_P2_* below */
	uint8_t  dip;	/* DIP switch byte (port 0x02)      */
	uint8_t  coins;	/* coin / start inputs (port 0x03)  */
};

/* Player 1 button bitmasks (phoenix_input.p1) */
#define PHOENIX_P1_LEFT		(1 << 0)
#define PHOENIX_P1_RIGHT	(1 << 1)
#define PHOENIX_P1_FIRE		(1 << 2)
#define PHOENIX_P1_START	(1 << 3)
#define PHOENIX_P1_START2	(1 << 4)
#define PHOENIX_P1_COIN		(1 << 7)

/*
 * phoenix_state – top-level emulator state handle.
 * The fields are exposed so that front-ends can inspect them; they must
 * not be modified directly.
 */
struct phoenix_state {
	struct phoenix_video  video;
	struct phoenix_sound  sound;
	struct phoenix_input  input;

	int  irq_enable;	/* set by port 0x07 bit 0 */
	int  running;		/* non-zero while the emulation loop is active */
};

/* -------------------------------------------------------------------------
 * Lifecycle API
 * ---------------------------------------------------------------------- */

/*
 * phoenix_init – allocate state, load ROMs from rom_dir, and reset the CPU.
 * Returns a pointer to a heap-allocated phoenix_state on success, or NULL
 * on failure (a diagnostic is printed to stderr).
 *
 * rom_dir must contain the ROM files listed in docs/PHOENIX-HARDWARE.md.
 */
struct phoenix_state *phoenix_init(const char *rom_dir);

/*
 * phoenix_reset – assert the 8080 RESET line and reinitialise all
 * subsystem registers to their power-on values.
 */
void phoenix_reset(struct phoenix_state *ph);

/*
 * phoenix_run_frame – advance the emulation by one video frame.
 *
 * Internally this ticks the CPU for PHOENIX_TSTATES_PER_FRAME cycles,
 * calling the begin_slice / end_slice hooks at a configurable granularity
 * (default: 8 slices per frame, ~2 ms each).
 *
 * Returns 0 on success, -1 if a fatal error occurred.
 */
int phoenix_run_frame(struct phoenix_state *ph);

/*
 * phoenix_shutdown – release all resources owned by ph.
 * The pointer must not be used after this call.
 */
void phoenix_shutdown(struct phoenix_state *ph);

/* -------------------------------------------------------------------------
 * Optional slice hooks
 *
 * Register a begin_slice / end_slice pair to receive a callback before and
 * after each CPU slice within a frame.  Useful for audio mixing, renderer
 * rasterisation, or debug tracing.
 * ---------------------------------------------------------------------- */

/*
 * phoenix_set_slice_hooks – register (or clear) per-slice callbacks.
 *
 *   begin_slice(ph, slice_index, user_data) – called just before CPU ticks
 *   end_slice(ph, slice_index, tstates, user_data) – called after; tstates
 *       is the number of T-states actually executed in this slice.
 *
 * Pass NULL for both to disable.
 */
void phoenix_set_slice_hooks(
	struct phoenix_state *ph,
	void (*begin_slice)(struct phoenix_state *, int, void *),
	void (*end_slice)(struct phoenix_state *, int, int, void *),
	void *user_data);

#endif /* PHOENIX_H */
