/*
 *	Phoenix arcade emulator
 *
 *	CPU:   Intel 8080A @ 2.000 MHz
 *	Video: Two 32×32 tile planes, 8×8 pixel tiles, 8 colours
 *	Sound: TMS36XX melody + discrete noise (skeleton)
 *	Input: Two joysticks + fire, coin, start; DIP switches
 *
 *	Usage:  phoenix -roms <rom_dir> [-f] [-d <trace_mask>]
 *
 *	See docs/PHOENIX-HARDWARE.md for the full hardware reference.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>

#include <SDL2/SDL.h>

/* The 8080A CPU core shared by the rest of the emulator kit. */
#include "../../intel_8080_emulator.h"

#include "phoenix.h"

/* -------------------------------------------------------------------------
 * Trace / debug flags
 * ---------------------------------------------------------------------- */

#define TRACE_MEM	0x01
#define TRACE_IO	0x02
#define TRACE_IRQ	0x04
#define TRACE_CPU	0x08
#define TRACE_VIDEO	0x10
#define TRACE_SOUND	0x20

static int trace = 0;

/* -------------------------------------------------------------------------
 * ROM / RAM layout
 *
 * 0x0000–0x1FFF  Program ROMs (4 × 2 KB = 8 KB)
 * 0x4000–0x43FF  Foreground tile map (VRAM)
 * 0x4400–0x47FF  Background tile map (VRAM)
 * 0x4800–0x4BFF  Color / attribute RAM
 * 0x4C00–0x4FFF  Work RAM (1 KB)
 * ---------------------------------------------------------------------- */

#define ROM_SIZE	0x2000	/* 8 KB program ROM                       */
#define FG_MAP_BASE	0x4000
#define BG_MAP_BASE	0x4400
#define COLOR_BASE	0x4800
#define RAM_BASE	0x4C00
#define RAM_SIZE	0x0400	/* 1 KB work RAM                          */

/* Character/tile ROMs (graphics only, not mapped into the 8080 address
   space – accessed exclusively by the video renderer).                   */
#define CHAR_ROM_SIZE	0x1000	/* 4 KB per tile set                      */

static uint8_t prog_rom[ROM_SIZE];
static uint8_t work_ram[RAM_SIZE];
static uint8_t fg_vram[0x400];		/* foreground tile map */
static uint8_t bg_vram[0x400];		/* background tile map */
static uint8_t color_ram[0x400];	/* color / attribute   */
static uint8_t bg_char_rom[CHAR_ROM_SIZE];
static uint8_t fg_char_rom[CHAR_ROM_SIZE];

/* -------------------------------------------------------------------------
 * Global emulator state
 * ---------------------------------------------------------------------- */

static struct phoenix_state state;

/* Slice-hook callbacks (optional). */
static void (*hook_begin_slice)(struct phoenix_state *, int, void *) = NULL;
static void (*hook_end_slice)(struct phoenix_state *, int, int, void *) = NULL;
static void *hook_user_data = NULL;

/* Number of CPU slices per frame (increase for finer audio granularity). */
#define SLICES_PER_FRAME	8
#define TSTATES_PER_SLICE	(PHOENIX_TSTATES_PER_FRAME / SLICES_PER_FRAME)

/* -------------------------------------------------------------------------
 * SDL2 display
 * ---------------------------------------------------------------------- */

static SDL_Window   *window   = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture  *texture  = NULL;

/* Pixel buffer: PHOENIX_SCREEN_W × PHOENIX_SCREEN_H, ARGB8888. */
static uint32_t framebuf[PHOENIX_SCREEN_W * PHOENIX_SCREEN_H];

/* 3-bit RGB hardware palette (index 0–7). */
static const uint32_t palette[8] = {
	0xFF000000,	/* 0 – Black   */
	0xFF0000AA,	/* 1 – Blue    */
	0xFF00AA00,	/* 2 – Green   */
	0xFF00AAAA,	/* 3 – Cyan    */
	0xFFAA0000,	/* 4 – Red     */
	0xFFAA00AA,	/* 5 – Magenta */
	0xFFAAAA00,	/* 6 – Yellow  */
	0xFFAAAAAA,	/* 7 – White   */
};

volatile int emulator_done = 0;

/* -------------------------------------------------------------------------
 * ROM loading helpers
 * ---------------------------------------------------------------------- */

/*
 * load_rom – open <dir>/<name1> (falling back to <name2>) and read exactly
 * <size> bytes into <dest>.  Returns 0 on success, -1 on error.
 */
static int load_rom(const char *dir, const char *name1, const char *name2,
		    uint8_t *dest, size_t size)
{
	char path[512];
	int fd;
	ssize_t n;

	snprintf(path, sizeof(path), "%s/%s", dir, name1);
	fd = open(path, O_RDONLY);
	if (fd == -1 && name2 != NULL) {
		snprintf(path, sizeof(path), "%s/%s", dir, name2);
		fd = open(path, O_RDONLY);
	}
	if (fd == -1) {
		fprintf(stderr, "phoenix: cannot open ROM '%s' (or '%s'): ",
			name1, name2 ? name2 : "n/a");
		perror(NULL);
		return -1;
	}

	n = read(fd, dest, size);
	close(fd);
	if ((size_t)n != size) {
		fprintf(stderr, "phoenix: ROM '%s': expected %zu bytes, got %zd\n",
			name1, size, n);
		return -1;
	}
	return 0;
}

/* -------------------------------------------------------------------------
 * 8080A memory callbacks (called by intel_8080_emulator)
 * ---------------------------------------------------------------------- */

uint8_t i8080_read(uint16_t addr)
{
	if (trace & TRACE_MEM)
		fprintf(stderr, "MR %04X\n", addr);

	if (addr < ROM_SIZE)
		return prog_rom[addr];
	if (addr >= FG_MAP_BASE && addr < FG_MAP_BASE + 0x400)
		return fg_vram[addr - FG_MAP_BASE];
	if (addr >= BG_MAP_BASE && addr < BG_MAP_BASE + 0x400)
		return bg_vram[addr - BG_MAP_BASE];
	if (addr >= COLOR_BASE && addr < COLOR_BASE + 0x400)
		return color_ram[addr - COLOR_BASE];
	if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE)
		return work_ram[addr - RAM_BASE];

	if (trace & TRACE_MEM)
		fprintf(stderr, "MR unmapped %04X\n", addr);
	return 0xFF;
}

uint8_t i8080_debug_read(uint16_t addr)
{
	/* Non-side-effecting version used by the disassembler / debugger. */
	return i8080_read(addr);
}

void i8080_write(uint16_t addr, uint8_t val)
{
	if (trace & TRACE_MEM)
		fprintf(stderr, "MW %04X <- %02X\n", addr, val);

	if (addr < ROM_SIZE) {
		/* Writes to ROM space are ignored. */
		return;
	}
	if (addr >= FG_MAP_BASE && addr < FG_MAP_BASE + 0x400) {
		fg_vram[addr - FG_MAP_BASE] = val;
		state.video.fg_map[addr - FG_MAP_BASE] = val;
		return;
	}
	if (addr >= BG_MAP_BASE && addr < BG_MAP_BASE + 0x400) {
		bg_vram[addr - BG_MAP_BASE] = val;
		state.video.bg_map[addr - BG_MAP_BASE] = val;
		return;
	}
	if (addr >= COLOR_BASE && addr < COLOR_BASE + 0x400) {
		color_ram[addr - COLOR_BASE] = val;
		state.video.color_ram[addr - COLOR_BASE] = val;
		return;
	}
	if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
		work_ram[addr - RAM_BASE] = val;
		return;
	}

	if (trace & TRACE_MEM)
		fprintf(stderr, "MW unmapped %04X <- %02X\n", addr, val);
}

/* -------------------------------------------------------------------------
 * 8080A I/O callbacks
 * ---------------------------------------------------------------------- */

uint8_t i8080_inport(uint8_t port)
{
	uint8_t r = 0xFF;

	switch (port) {
	case 0x00:
		/* Player 1 controls – active low, invert our bitmask. */
		r = (uint8_t)(~state.input.p1 & 0xFF);
		break;
	case 0x01:
		/* Player 2 controls. */
		r = (uint8_t)(~state.input.p2 & 0xFF);
		break;
	case 0x02:
		/* DIP switches. */
		r = state.input.dip;
		break;
	case 0x03:
		/* Coin / watchdog inputs. */
		r = state.input.coins;
		break;
	default:
		if (trace & TRACE_IO)
			fprintf(stderr, "IO IN  %02X (unmapped)\n", port);
		break;
	}

	if (trace & TRACE_IO)
		fprintf(stderr, "IO IN  %02X -> %02X\n", port, r);
	return r;
}

void i8080_outport(uint8_t port, uint8_t val)
{
	if (trace & TRACE_IO)
		fprintf(stderr, "IO OUT %02X <- %02X\n", port, val);

	switch (port) {
	case 0x04:
		/* Palette select / coin counter. */
		state.video.palette_sel = val;
		break;
	case 0x05:
		/* TMS36XX melody latch – TODO: drive melody subsystem. */
		state.sound.melody_latch = val;
		if (trace & TRACE_SOUND)
			fprintf(stderr, "SOUND melody <- %02X\n", val);
		break;
	case 0x06:
		/* Discrete noise triggers – TODO: drive noise subsystem. */
		state.sound.noise_latch = val;
		if (trace & TRACE_SOUND)
			fprintf(stderr, "SOUND noise  <- %02X\n", val);
		break;
	case 0x07:
		/* Bit 0: IRQ enable; bits 1-4: background scroll offset. */
		state.irq_enable     = val & 0x01;
		state.video.bg_scroll = (val >> 1) & 0x0F;
		if (trace & TRACE_IRQ)
			fprintf(stderr, "IRQ enable=%d bg_scroll=%d\n",
				state.irq_enable, state.video.bg_scroll);
		break;
	default:
		if (trace & TRACE_IO)
			fprintf(stderr, "IO OUT %02X <- %02X (unmapped)\n",
				port, val);
		break;
	}
}

/* Required by the 8080 core: return the RST vector placed on the data bus
   when the CPU acknowledges the interrupt.  Phoenix uses RST 7 (0xFF). */
uint8_t i8080_get_vector(void)
{
	return 0xFF;	/* RST 7 → jump to 0x0038 */
}

/* -------------------------------------------------------------------------
 * Video rendering (stub)
 *
 * For each tile plane we walk the 32×32 tile map, look up the 8×8 pixel
 * pattern from the character ROM, and write ARGB pixels into framebuf.
 * Color is taken from color_ram (one byte per 8-pixel column strip).
 *
 * TODO: implement pixel-accurate blending of the two planes and the
 *       transparent / opaque pixel logic described in PHOENIX-HARDWARE.md.
 * ---------------------------------------------------------------------- */

static void render_tile_plane(const uint8_t *tile_map,
			      const uint8_t *char_rom,
			      int priority)
{
	int tx, ty, px, py;
	(void)priority;	/* will be used for transparency blending */

	for (ty = 0; ty < 32; ty++) {
		for (tx = 0; tx < 32; tx++) {
			uint8_t tile   = tile_map[ty * 32 + tx];
			int     offset = tile * 8;
			/* Color attribute: one byte per 8-column strip. */
			uint8_t attr   = color_ram[tx];
			int     fg_idx = (attr >> 0) & 0x07;
			int     bg_idx = (attr >> 4) & 0x07;

			for (py = 0; py < 8; py++) {
				uint8_t row = char_rom[offset + py];
				for (px = 0; px < 8; px++) {
					int bit = (row >> (7 - px)) & 1;
					uint32_t color = bit ? palette[fg_idx]
							     : palette[bg_idx];
					int sx = tx * 8 + px;
					int sy = ty * 8 + py;
					if (sx < PHOENIX_SCREEN_W &&
					    sy < PHOENIX_SCREEN_H)
						framebuf[sy * PHOENIX_SCREEN_W + sx] = color;
				}
			}
		}
	}
}

static void video_render(void)
{
	memset(framebuf, 0, sizeof(framebuf));
	render_tile_plane(bg_vram, bg_char_rom, 0);
	render_tile_plane(fg_vram, fg_char_rom, 1);

	if (texture) {
		SDL_UpdateTexture(texture, NULL, framebuf,
				  PHOENIX_SCREEN_W * sizeof(uint32_t));
		SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, texture, NULL, NULL);
		SDL_RenderPresent(renderer);
	}
}

/* -------------------------------------------------------------------------
 * Input polling
 * ---------------------------------------------------------------------- */

static void input_poll(void)
{
	SDL_Event ev;
	const uint8_t *kb = SDL_GetKeyboardState(NULL);

	(void)kb;	/* will be used for key mapping once fleshed out */

	while (SDL_PollEvent(&ev)) {
		switch (ev.type) {
		case SDL_QUIT:
			emulator_done = 1;
			break;
		case SDL_KEYDOWN:
			switch (ev.key.keysym.sym) {
			case SDLK_LEFT:
				state.input.p1 |= PHOENIX_P1_LEFT;
				break;
			case SDLK_RIGHT:
				state.input.p1 |= PHOENIX_P1_RIGHT;
				break;
			case SDLK_SPACE:
				state.input.p1 |= PHOENIX_P1_FIRE;
				break;
			case SDLK_1:
				state.input.p1 |= PHOENIX_P1_START;
				break;
			case SDLK_5:
				state.input.p1 |= PHOENIX_P1_COIN;
				break;
			case SDLK_ESCAPE:
				emulator_done = 1;
				break;
			default:
				break;
			}
			break;
		case SDL_KEYUP:
			switch (ev.key.keysym.sym) {
			case SDLK_LEFT:
				state.input.p1 &= (uint8_t)~PHOENIX_P1_LEFT;
				break;
			case SDLK_RIGHT:
				state.input.p1 &= (uint8_t)~PHOENIX_P1_RIGHT;
				break;
			case SDLK_SPACE:
				state.input.p1 &= (uint8_t)~PHOENIX_P1_FIRE;
				break;
			case SDLK_1:
				state.input.p1 &= (uint8_t)~PHOENIX_P1_START;
				break;
			case SDLK_5:
				state.input.p1 &= (uint8_t)~PHOENIX_P1_COIN;
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
	}
}

/* -------------------------------------------------------------------------
 * Public API – lifecycle
 * ---------------------------------------------------------------------- */

struct phoenix_state *phoenix_init(const char *rom_dir)
{
	/* Clear state. */
	memset(&state, 0, sizeof(state));
	memset(prog_rom,   0xFF, sizeof(prog_rom));
	memset(work_ram,   0x00, sizeof(work_ram));
	memset(fg_vram,    0x00, sizeof(fg_vram));
	memset(bg_vram,    0x00, sizeof(bg_vram));
	memset(color_ram,  0x00, sizeof(color_ram));
	memset(bg_char_rom, 0x00, sizeof(bg_char_rom));
	memset(fg_char_rom, 0x00, sizeof(fg_char_rom));

	/* ------------------------------------------------------------------
	 * Load program ROMs (4 × 2 KB).
	 * Accept both the Centuri names (phoenix.45 …) and the IC-stamp
	 * alternate names (h1-ic45.1a …).
	 * ---------------------------------------------------------------- */
	if (load_rom(rom_dir, "phoenix.45", "h1-ic45.1a",
		     prog_rom + 0x0000, 0x0800) != 0) return NULL;
	if (load_rom(rom_dir, "phoenix.46", "h2-ic46.2a",
		     prog_rom + 0x0800, 0x0800) != 0) return NULL;
	if (load_rom(rom_dir, "phoenix.47", "h3-ic47.3a",
		     prog_rom + 0x1000, 0x0800) != 0) return NULL;
	if (load_rom(rom_dir, "phoenix.48", "h4-ic48.4a",
		     prog_rom + 0x1800, 0x0800) != 0) return NULL;

	/* ------------------------------------------------------------------
	 * Load graphics ROMs.
	 * ---------------------------------------------------------------- */
	if (load_rom(rom_dir, "phoenix.b1-4k", NULL,
		     bg_char_rom, CHAR_ROM_SIZE) != 0) return NULL;
	if (load_rom(rom_dir, "phoenix.b2-4k", NULL,
		     fg_char_rom, CHAR_ROM_SIZE) != 0) return NULL;

	/* ------------------------------------------------------------------
	 * Initialise SDL2 window and texture.
	 * ---------------------------------------------------------------- */
	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		fprintf(stderr, "phoenix: SDL_Init: %s\n", SDL_GetError());
		return NULL;
	}

	window = SDL_CreateWindow("Phoenix",
				  SDL_WINDOWPOS_CENTERED,
				  SDL_WINDOWPOS_CENTERED,
				  PHOENIX_SCREEN_W * 2,
				  PHOENIX_SCREEN_H * 2,
				  0);
	if (!window) {
		fprintf(stderr, "phoenix: SDL_CreateWindow: %s\n",
			SDL_GetError());
		return NULL;
	}

	renderer = SDL_CreateRenderer(window, -1,
				      SDL_RENDERER_ACCELERATED |
				      SDL_RENDERER_PRESENTVSYNC);
	if (!renderer) {
		/* Fall back to software renderer. */
		renderer = SDL_CreateRenderer(window, -1,
					      SDL_RENDERER_SOFTWARE);
		if (!renderer) {
			fprintf(stderr, "phoenix: SDL_CreateRenderer: %s\n",
				SDL_GetError());
			return NULL;
		}
	}

	SDL_RenderSetLogicalSize(renderer, PHOENIX_SCREEN_W, PHOENIX_SCREEN_H);

	texture = SDL_CreateTexture(renderer,
				    SDL_PIXELFORMAT_ARGB8888,
				    SDL_TEXTUREACCESS_STREAMING,
				    PHOENIX_SCREEN_W, PHOENIX_SCREEN_H);
	if (!texture) {
		fprintf(stderr, "phoenix: SDL_CreateTexture: %s\n",
			SDL_GetError());
		return NULL;
	}

	/* Reset the CPU and set the initial state. */
	phoenix_reset(&state);

	state.running = 1;
	return &state;
}

void phoenix_reset(struct phoenix_state *ph)
{
	i8080_reset();
	ph->irq_enable = 0;
	ph->video.bg_scroll   = 0;
	ph->video.palette_sel = 0;
	ph->sound.melody_latch = 0;
	ph->sound.noise_latch  = 0;
	if (trace & TRACE_CPU)
		i8080_log = stderr;
}

int phoenix_run_frame(struct phoenix_state *ph)
{
	int s;

	for (s = 0; s < SLICES_PER_FRAME; s++) {
		if (hook_begin_slice)
			hook_begin_slice(ph, s, hook_user_data);

		int executed = i8080_exec(TSTATES_PER_SLICE);

		if (hook_end_slice)
			hook_end_slice(ph, s, executed, hook_user_data);
	}

	/* Fire VBLANK interrupt at end of frame if enabled. */
	if (ph->irq_enable) {
		if (trace & TRACE_IRQ)
			fprintf(stderr, "VBLANK IRQ\n");
		i8080_set_int(INT_IRQ);
		/* The 8080 core samples the interrupt at the start of the next
		   instruction; clear the request after a brief single-op run
		   so we do not re-assert it every frame.                      */
		i8080_exec(1);
		i8080_clear_int(INT_IRQ);
	}

	/* Poll SDL events and render the frame. */
	input_poll();
	video_render();

	return 0;
}

void phoenix_shutdown(struct phoenix_state *ph)
{
	ph->running = 0;

	if (texture)  { SDL_DestroyTexture(texture);   texture  = NULL; }
	if (renderer) { SDL_DestroyRenderer(renderer); renderer = NULL; }
	if (window)   { SDL_DestroyWindow(window);     window   = NULL; }
	SDL_Quit();
}

void phoenix_set_slice_hooks(
	struct phoenix_state *ph,
	void (*begin_slice)(struct phoenix_state *, int, void *),
	void (*end_slice)(struct phoenix_state *, int, int, void *),
	void *user_data)
{
	(void)ph;
	hook_begin_slice = begin_slice;
	hook_end_slice   = end_slice;
	hook_user_data   = user_data;
}

/* -------------------------------------------------------------------------
 * main
 * ---------------------------------------------------------------------- */

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s -roms <rom_dir> [-f] [-d <trace_mask>]\n"
		"\n"
		"  -roms <dir>   directory containing the Phoenix ROM files\n"
		"  -f            fast mode (disable 60 Hz frame throttle)\n"
		"  -d <mask>     debug trace bitmask:\n"
		"                  0x01 MEM  0x02 IO  0x04 IRQ\n"
		"                  0x08 CPU  0x10 VIDEO  0x20 SOUND\n",
		prog);
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	const char *rom_dir = NULL;
	int fast = 0;
	int opt;

	while ((opt = getopt(argc, argv, "r:d:f")) != -1) {
		switch (opt) {
		case 'r':
			rom_dir = optarg;
			break;
		case 'd':
			trace = (int)strtol(optarg, NULL, 0);
			break;
		case 'f':
			fast = 1;
			break;
		default:
			usage(argv[0]);
		}
	}

	/* Also accept -roms as a long-style flag (two arguments). */
	for (int i = 1; i < argc - 1; i++) {
		if (strcmp(argv[i], "-roms") == 0) {
			rom_dir = argv[i + 1];
			break;
		}
	}

	if (rom_dir == NULL)
		usage(argv[0]);

	struct phoenix_state *ph = phoenix_init(rom_dir);
	if (!ph) {
		fprintf(stderr, "phoenix: initialization failed\n");
		return EXIT_FAILURE;
	}

	/* Frame throttle: aim for PHOENIX_FPS frames per second. */
	struct timespec frame_time;
	frame_time.tv_sec  = 0;
	frame_time.tv_nsec = 1000000000L / PHOENIX_FPS;

	while (!emulator_done) {
		phoenix_run_frame(ph);
		if (!fast)
			nanosleep(&frame_time, NULL);
	}

	phoenix_shutdown(ph);
	return EXIT_SUCCESS;
}
