/*
 *  Very basic ZX Spectrum set up for debugging stuff. This does not
 *  do all the timing related magic required to run games correctly
 *  with effects and stuff.
 *
 *  TODO: ZXCF, Simple CF cards
 *
 *  Additions:
 *   - SNA loader (48K & 128K) fixed
 *   - Kempston joystick on port 0x1F (arrow keys + Ctrl/Space/Enter = FIRE)
 *   - TAP fast loader: injects CODE/SCREEN$ blocks to param1 address, no EAR/timing
 *   - TAP pulse player (ROM-accurate): pilot/sync/bits/pauses on EAR input
 *   - TZX pulse player: full TZX support via tzx.c (blocks 0x10-0x19, control)
 *   - Beeper (EAR|MIC) audio via SDL2 (queue mode)
 *   - Hotkeys: F6 (reload TAP & autostart fast), F7 (list TAP),
 *              F8 (Play/Pause tape/TZX pulses), F9 (Rewind tape/TZX pulses)
 *
 *  TAP format refs:
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/TAP_format
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/Spectrum_tape_interface
 *  TZX format refs:
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/TZX_format
 *  SNA format refs:
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/SNA_format
 *    - https://worldofspectrum.net/zx-modules/fileformats/snaformat.html
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>
#include "libz80/z80.h"
#include "z80dis.h"
#include "ide.h"
#include "lib765/include/765.h"
#include "tzx.h"
#include "src/tape.h"
#include "src/sna.h"
#include "src/divide.h"

#include <SDL2/SDL.h>
#include "event.h"
#include "keymatrix.h"

static SDL_Window *window;
static SDL_Renderer *render;
static SDL_Texture *texture;

#define BORDER  32
#define WIDTH   (256 + 2 * BORDER)
#define HEIGHT  (192 + 2 * BORDER)

static uint32_t texturebits[WIDTH * HEIGHT];

uint8_t border_color   = 7;

static uint32_t palette[16] = {
    0xFF000000, 0xFF0000D8, 0xFFD80000, 0xFFD800D8,
    0xFF00D800, 0xFF00D8D8, 0xFFD8D800, 0xFFD8D8D8,
    0xFF000000, 0xFF0000FF, 0xFFFF0000, 0xFFFF00FF,
    0xFF00FF00, 0xFF00FFFF, 0xFFFFFF00, 0xFFFFFFFF
};

/* Keep all the memory in one map for now. 128K will need to do a bit
   more work later */
static uint8_t ram[16][16384];
#define ROM(x)  (x)
#define RAM(x)  ((x) + 8)

static struct keymatrix *matrix;
static Z80Context cpu_z80;
static FDC_PTR fdc;
static FDRV_PTR drive_a, drive_b;
static struct ide_controller *ide;

//static int tape = -1;          /* Tape file handle (unused by fast TAP) */
static unsigned mem = 48;      /* First byte above RAM (defaults to 48K) */
static uint8_t ula;            /* ULA state */
static uint8_t frames;         /* Flash counter */
static uint8_t mlatch;
static uint8_t p3latch;
static unsigned map[4] = { ROM(0), RAM(5), RAM(2), RAM(0) };
static unsigned vram = RAM(5);

static unsigned drawline;      /* If rasterising */
static unsigned blanked;       /* True if blanked */

static divide_state_t divide_st;
static divide_ctx_t   divide_ctx;

#define ZX_48K_2    0
#define ZX_48K_3    1
#define ZX_128K     2
#define ZX_PLUS3    3
static unsigned model = ZX_48K_3;

/* ── Public wrappers used by the divide module (divide.c) ── */
unsigned spectrum_get_model(void)  { return model; }
uint8_t  spectrum_get_mlatch(void) { return mlatch; }

static volatile int emulator_done;
static unsigned fast;
static unsigned int_recalc;
/* static unsigned live_irq; */

#define TRACE_MEM   1
#define TRACE_IO    2
#define TRACE_IRQ   4
#define TRACE_KEY   8
#define TRACE_CPU   16
#define TRACE_FDC   32

static int trace = 0;

static void reti_event(void);

/* ─────────────────────────────────────────────────────────────
 * Beeper (EAR/MIC) + SDL2 audio queue (mono, S16)
 * ───────────────────────────────────────────────────────────── */
/* TSTATES_CPU is defined in tape.h */

static SDL_AudioDeviceID audio_dev = 0;
static SDL_AudioSpec have;
static int audio_rate = 44100;
static float beeper_volume = 0.30f;

static uint64_t beeper_frame_origin = 0;
static uint64_t beeper_slice_origin = 0;
static uint64_t beeper_last_tstate  = 0;
static int      beeper_level        = 0;   /* 0 o 1 (onda cuadrada) */
static double   beeper_frac_acc     = 0.0;

static int audio_init_sdl(int rate)
{
    SDL_AudioSpec want;
    SDL_zero(want);
    want.freq = rate;
    want.format = AUDIO_S16SYS;
    want.channels = 1;
    want.samples = 2048;

    audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!audio_dev) {
        fprintf(stderr, "SDL audio: unable to open: %s\n", SDL_GetError());
        return -1;
    }
    audio_rate = have.freq;
    SDL_PauseAudioDevice(audio_dev, 0);
    return 0;
}

static inline void beeper_advance_to(uint64_t t_now)
{
    if (!audio_dev) return;
    if (t_now <= beeper_last_tstate) return;

    uint64_t dt = t_now - beeper_last_tstate;
    beeper_last_tstate = t_now;

    double add = (double)dt * (double)audio_rate / TSTATES_CPU;
    beeper_frac_acc += add;
    int nsamp = (int)beeper_frac_acc;
    if (nsamp <= 0) return;
    beeper_frac_acc -= nsamp;

    int16_t val = beeper_level ? (int16_t)(+beeper_volume * 32767.0f)
                               : (int16_t)(-beeper_volume * 32767.0f);
    enum { CHUNK = 4096 };
    static int16_t buf[CHUNK];
    while (nsamp > 0) {
        int n = (nsamp > CHUNK) ? CHUNK : nsamp;
        for (int i = 0; i < n; ++i) buf[i] = val;
        SDL_QueueAudio(audio_dev, buf, n * (int)sizeof(int16_t));
        nsamp -= n;
    }
}

static inline void beeper_begin_slice(void)
{
    beeper_slice_origin = beeper_frame_origin;
}

static inline void beeper_end_slice(void)
{
    uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
    beeper_advance_to(t_now);
    beeper_frame_origin = t_now;
}

static inline void beeper_set_level(int level_now)
{
    uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
    beeper_advance_to(t_now);
    beeper_level = level_now ? 1 : 0;
}

/* EAR=b4, MIC=b3 → modelado sencillo como OR */
static inline void beeper_set_from_ula(uint8_t v)
{
    int level = ((v >> 4) & 1) | ((v >> 3) & 1);
    beeper_set_level(level);
}

/* ─────────────────────────────────────────────────────────────
 * Kempston joystick (cursores + fire)
 * Puerto 0x1F (31 dec). Bits activos en alto:
 * bit0 Right, bit1 Left, bit2 Down, bit3 Up, bit4 Fire
 * ───────────────────────────────────────────────────────────── */
static inline uint8_t kempston_state_from_sdl(void)
{
    const Uint8 *ks = SDL_GetKeyboardState(NULL);
    uint8_t v = 0;

    if (ks[SDL_SCANCODE_RIGHT]) v |= 0x01; // Right
    if (ks[SDL_SCANCODE_LEFT])  v |= 0x02; // Left
    if (ks[SDL_SCANCODE_DOWN])  v |= 0x04; // Down
    if (ks[SDL_SCANCODE_UP])    v |= 0x08; // Up

    // Fire: teclas cómodas
    if (ks[SDL_SCANCODE_LCTRL] || ks[SDL_SCANCODE_RCTRL] ||
        ks[SDL_SCANCODE_SPACE] || ks[SDL_SCANCODE_RETURN]) {
        v |= 0x10;
    }
    // Bits 5..7 no usados → 0
    return v;
}

/* TODO: memory contention */
static uint8_t do_mem_read(uint16_t addr, unsigned debug)
{
    unsigned bank = map[addr >> 14];
    /* For now until banked stuff */
    if (addr >= mem)
        return 0xFF;
    if (addr < 0x4000 && divide_is_mapped(&divide_st))
        return divide_mem_read(&divide_st, addr);
    return ram[bank][addr & 0x3FFF];
}

static void mem_write(int unused, uint16_t addr, uint8_t val)
{
    unsigned bank = map[addr >> 14];
    if (addr >= mem)
        return;
    if (addr < 0x4000 && divide_is_mapped(&divide_st)) {
        divide_mem_write(&divide_st, addr, val);
        return;
    }
    /* ROM is read only */
    if (bank >= RAM(0))
        ram[bank][addr & 0x3FFF] = val;
}

static uint8_t mem_read(int unused, uint16_t addr)
{
    static uint8_t rstate;
    uint8_t r;

    /* DivIDE+ modes other than 00 don't autopage */
    if (cpu_z80.M1)
        divide_m1_pre(&divide_ctx, addr);

    r = do_mem_read(addr, 0);

    /* Look for ED with M1, followed directly by 4D and if so trigger
       the interrupt chain */
    if (cpu_z80.M1) {
        divide_m1_post(&divide_st, addr);
        /* DD FD CB see the Z80 interrupt manual */
        if (r == 0xDD || r == 0xFD || r == 0xCB) {
            rstate = 2;
            return r;
        }
        /* Look for ED with M1, followed directly by 4D and if so trigger
           the interrupt chain */
        if (r == 0xED && rstate == 0) {
            rstate = 1;
            return r;
        }
    }
    if (r == 0x4D && rstate == 1)
        reti_event();
    rstate = 0;
    return r;
}

static void recalc_mmu(void)
{
    map[3] = RAM(mlatch & 7);
    if (mlatch & 0x08)
        vram = RAM(7);
    else
        vram = RAM(5);
    if (model == ZX_128K) {
        if (mlatch & 0x10)
            map[0] = ROM(1);
        else
            map[0] = ROM(0);
    }
    if (model == ZX_PLUS3) {
        unsigned rom = (mlatch & 0x10) ? 1 : 0;
        if (p3latch & 0x04)
            rom |= 2;
        map[0] = ROM(rom);
        switch(p3latch & 0x07) {
        case 1:
            map[0] = RAM(0);
            map[1] = RAM(1);
            map[2] = RAM(2);
            map[3] = RAM(3);
            break;
        case 3:
            map[0] = RAM(4);
            map[1] = RAM(5);
            map[2] = RAM(6);
            map[3] = RAM(7);
            break;
        case 5:
            map[0] = RAM(4);
            map[1] = RAM(5);
            map[2] = RAM(6);
            map[3] = RAM(3);
            break;
        case 7:
            map[0] = RAM(4);
            map[1] = RAM(7);
            map[2] = RAM(6);
            map[3] = RAM(3);
            break;
        }
    }
}

static void repaint_border(unsigned colour)
{
    border_color   = colour;

    uint32_t *p = texturebits;
    unsigned x,y;
    uint32_t border = palette[colour];

    for(y = 0; y < BORDER; y++)
        for(x = 0; x < WIDTH; x++)
            *p++ = border;
    for(y = BORDER; y < BORDER + 192; y++) {
        for(x = 0; x < BORDER; x++)
            *p++ = border;
        p += 256;
        for(x = 0; x < BORDER; x++)
            *p++ = border;
    }
    for(y = 0; y < BORDER; y++)
        for (x = 0; x < WIDTH; x++)
            *p++ = border;
}

static void fdc_log(int debuglevel, char *fmt, va_list ap)
{
    if ((trace & TRACE_FDC) || debuglevel == 0) {
        fprintf(stderr, "fdc: ");
        vfprintf(stderr, fmt, ap);
    }
}

/* ─────────────────────────────────────────────────────────────
 * TAP pulse player instance (types/functions in tape.c / tape.h)
 * ───────────────────────────────────────────────────────────── */
static tape_player_t tape = {0};

/* ─────────────────────────────────────────────────────────────
 * TZX player (via tzx.c)
 * ───────────────────────────────────────────────────────────── */
static tzx_player_t *tzx_player = NULL;
static uint64_t      tzx_frame_origin = 0;

/* ─────────────────────────────────────────────────────────────
 * ULA I/O
 * ───────────────────────────────────────────────────────────── */

static void ula_write(uint8_t v)
{
    /* ear is bit 4 mic is bit 3, border low bits */
    ula = v;

    /* ▶️ Actualiza beeper (EAR|MIC) al instante actual */
    beeper_set_from_ula(v);

    repaint_border(v & 7);
}

static uint8_t ula_read(uint16_t addr)
{
    uint8_t r = 0xA0;  /* Fixed bits */

    /* bit6 = EAR (entrada):
       - Si la cinta por pulsos está activa, domina la línea
       - Si no, conserva tu comportamiento previo para Issue 3/48K_2 */
    uint8_t ear_b6 = 0x00;
    if (tape_active(&tape)) {
        ear_b6 = tape_ear_bit6(&tape);
    } else if (tzx_active(tzx_player)) {
        ear_b6 = tzx_ear_bit6(tzx_player);
    } else if (model != ZX_PLUS3) {
        if (ula & 0x10)     /* Issue 3 and later */
            ear_b6 = 0x40;
        if (model == ZX_48K_2 && (ula & 0x08))
            ear_b6 = 0x40;
    }
    r = (r & ~0x40) | ear_b6;

    /* Low 5 bits are keyboard matrix map */
    r |= ~keymatrix_input(matrix, ~(addr >> 8)) & 0x1F;
    return r;
}

static uint8_t floating(void)
{
    unsigned n;
    if (blanked || model == ZX_PLUS3)
        return 0xFF;
    n = cpu_z80.tstates;
    n /= 4;
    if (n < 32)
        return ram[vram][0x1800 + 32 * drawline + n];
    return 0xFF;
}

static uint8_t io_read(int unused, uint16_t addr)
{
    int r;

    /* Kempston joystick: puerto 0x1F */
    if ((addr & 0xFF) == 0x1F) {
        return kempston_state_from_sdl();
    }

    /* Timex checks XXFE, Sinclair just the low bit */
    if ((addr & 0x01) == 0) /* ULA */
        return ula_read(addr);
    if (model == ZX_PLUS3) {
        if ((addr & 0xF002) == 0x2000)
            return fdc_read_ctrl(fdc);
        if ((addr & 0xF002) == 0x3000)
            return fdc_read_data(fdc);
    }
    r = divide_io_read(&divide_ctx, addr);
    if (r >= 0)
        return (uint8_t)r;
    return floating();
}

static void io_write(int unused, uint16_t addr, uint8_t val)
{
    if (trace & TRACE_IO)
        fprintf(stderr, "write %02x <- %02x\n", addr, val);
    if ((addr & 1) == 0)
        ula_write(val);
    if (model == ZX_128K && (addr & 0x8002) == 0) {
        if ((mlatch & 0x20) == 0) {
            mlatch = val;
            recalc_mmu();
        }
    }
    if (model == ZX_PLUS3 && (addr & 0xF002) == 0x3000) {
        fdc_write_data(fdc, val);
    }
    if (model == ZX_PLUS3 && (addr & 0xC002) == 0x4000) {
        if ((mlatch & 0x20) == 0) {
            mlatch = val;
            recalc_mmu();
        }
    }
    if (model == ZX_PLUS3 && (addr & 0xF002) == 0x1000) {
        /* Does the memory latch lock this too ? TODO */
        p3latch = val;
        if (p3latch & 0x08)
            fdc_set_motor(fdc, 3);
        else
            fdc_set_motor(fdc, 0);
        recalc_mmu();
    }
    divide_io_write(&divide_ctx, addr, val);
}

static unsigned int nbytes;

uint8_t z80dis_byte(uint16_t addr)
{
    uint8_t r = do_mem_read(addr, 1);
    fprintf(stderr, "%02X ", r);
    nbytes++;
    return r;
}

uint8_t z80dis_byte_quiet(uint16_t addr)
{
    return do_mem_read(addr, 1);
}

static void z80_trace(unsigned unused)
{
    static uint32_t lastpc = -1;
    char buf[256];

    if ((trace & TRACE_CPU) == 0)
        return;
    nbytes = 0;
    /* Spot XXXR repeating instructions and squash the trace */
    if (cpu_z80.M1PC == lastpc
        && z80dis_byte_quiet(lastpc) == 0xED
        && (z80dis_byte_quiet(lastpc + 1) & 0xF4) == 0xB0) {
        return;
    }
    lastpc = cpu_z80.M1PC;
    fprintf(stderr, "%04X: ", lastpc);
    z80_disasm(buf, lastpc);
    while (nbytes++ < 6)
        fprintf(stderr, "   ");
    fprintf(stderr, "%-16s ", buf);
    fprintf(stderr,
        "[ %02X:%02X %04X %04X %04X %04X %04X %04X ]\n",
        cpu_z80.R1.br.A, cpu_z80.R1.br.F, cpu_z80.R1.wr.BC,
        cpu_z80.R1.wr.DE, cpu_z80.R1.wr.HL,
        cpu_z80.R1.wr.IX, cpu_z80.R1.wr.IY, cpu_z80.R1.wr.SP);
}

static void poll_irq_event(void)
{
}

static void reti_event(void)
{
}

static void raster_byte(unsigned lines, unsigned cols, uint8_t byte, uint8_t attr)
{
    uint32_t *pixp;
    unsigned x;
    unsigned paper = (attr >> 3) & 0x0F;
    unsigned ink = attr & 7;
    if (attr & 0x40)
        paper |= 0x08;

    /* Flash swaps every 16 frames */
    if ((attr & 0x80) && (frames & 0x10)) {
        x = ink;
        ink = paper;
        paper = x;
    }

    pixp = texturebits + (lines + BORDER) * WIDTH + cols * 8 + BORDER;

    for (x = 0; x < 8; x++) {
        if (byte & 0x80)
            *pixp++ = palette[ink];
        else
            *pixp++ = palette[paper];
        byte <<= 1;
    }
}

static void raster_block(unsigned ybase, unsigned off, unsigned aoff)
{
    unsigned c,l,w;
    uint8_t *ptr = ram[vram] + off;
    uint8_t *aptr = ram[vram] + aoff;
    for (l = 0; l < 8; l++) {
        for (c = 0; c < 8; c++)
            for (w = 0; w < 32; w++)
                raster_byte(ybase + c * 8 + l, w, *ptr++, *aptr++);
        aptr -= 0x100;
    }
}

static void spectrum_rasterize(void)
{
    raster_block(0, 0x0000, 0x1800);
    raster_block(64, 0x0800, 0x1900);
    raster_block(128, 0x1000, 0x1A00);
}

static void spectrum_render(void)
{
    SDL_Rect rect;

    rect.x = rect.y = 0;
    rect.w = WIDTH;
    rect.h = HEIGHT;

    SDL_UpdateTexture(texture, NULL, texturebits, WIDTH * 4);
    SDL_RenderClear(render);
    SDL_RenderCopy(render, texture, NULL, &rect);
    SDL_RenderPresent(render);
}

/*
 *  Keyboard mapping.
 *  TODO:
 */

static SDL_Keycode keyboard[] = {
    SDLK_LSHIFT, SDLK_z, SDLK_x, SDLK_c, SDLK_v,
    SDLK_a, SDLK_s, SDLK_d, SDLK_f, SDLK_g,
    SDLK_q, SDLK_w, SDLK_e, SDLK_r, SDLK_t,
    SDLK_1, SDLK_2, SDLK_3, SDLK_4, SDLK_5,
    SDLK_0, SDLK_9, SDLK_8, SDLK_7, SDLK_6,
    SDLK_p, SDLK_o, SDLK_i, SDLK_u, SDLK_y,
    SDLK_RETURN, SDLK_l, SDLK_k, SDLK_j, SDLK_h,
    SDLK_SPACE, SDLK_RSHIFT, SDLK_m, SDLK_n, SDLK_b
};

static void run_scanlines(unsigned lines, unsigned blank)
{
    unsigned i;
    unsigned n = 224;   /* T States per op */

    blanked = blank;

    if (!blanked)
        drawline = 0;
    /* Run scanlines */
    for (i = 0; i < lines; i++) {
        /* Delimitamos porciones (beeper + cinta) alrededor de la ejecución */
        beeper_begin_slice();
        tape_begin_slice(&tape);
        if (tzx_player) {
            tzx_begin_slice(tzx_player, tzx_frame_origin);
        }

        n = 224 + 224 - Z80ExecuteTStates(&cpu_z80, n);

        beeper_end_slice();
        tape_end_slice(&tape, &cpu_z80);
        if (tzx_player) {
            tzx_end_slice(tzx_player, &cpu_z80, &tzx_frame_origin);
        }

        if (!blanked)
            drawline++;
    }
    if (ui_event())
        emulator_done = 1;

    if (int_recalc) {
        /* If there is no pending Z80 vector IRQ but we think
           there now might be one we use the same logic as for
           reti */
        poll_irq_event();
        /* Clear this after because reti_event may set the
           flags to indicate there is more happening. We will
           pick up the next state changes on the reti if so */
        if (!(cpu_z80.IFF1 | cpu_z80.IFF2))
            int_recalc = 0;
    }
}

/* ─────────────────────────────────────────────────────────────
 * Hotkeys (SDL): F6 = Reload TAP & Auto-Start; F7 = List TAP
 *                F8 = Play/Pause tape pulses; F9 = Rewind tape
 * ───────────────────────────────────────────────────────────── */
static void handle_hotkeys(const char* tap_path, const char* tzx_path)
{
    SDL_PumpEvents(); /* actualiza estado */

    const Uint8* ks = SDL_GetKeyboardState(NULL);
    static int prev_f6 = 0, prev_f7 = 0, prev_f8 = 0, prev_f9 = 0, prev_f12 = 0;

    int f6 = ks[SDL_SCANCODE_F6] ? 1 : 0;
    int f7 = ks[SDL_SCANCODE_F7] ? 1 : 0;
    int f8 = ks[SDL_SCANCODE_F8] ? 1 : 0; /* Play/Pause pulses */
    int f9 = ks[SDL_SCANCODE_F9] ? 1 : 0; /* Rewind pulses */
	int f11 = ks[SDL_SCANCODE_F11] ? 1: 0; //slow
	int f12 = ks[SDL_SCANCODE_F12] ? 1: 0; //fast

    if (f6 && !prev_f6) {
        if (!tap_path) {
            fprintf(stderr, "[F6] No hay TAP (usa -t <fichero.tap>)\n");
        } else {
            printf("[F6] Reload TAP & Auto-Start: %s\n", tap_path);
            if (!load_tap_fast(tap_path, /*auto_start=*/1, &cpu_z80, mem_write))
                fprintf(stderr, "[TAP] Fallo al recargar: %s\n", tap_path);
        }
    }
    if (f7 && !prev_f7) {
        if (!tap_path) {
            fprintf(stderr, "[F7] No hay TAP (usa -t <fichero.tap>)\n");
        } else {
            tap_list(tap_path);
        }
    }

    if (f8 && !prev_f8) {
        if (tzx_path && tzx_player) {
            if (tzx_active(tzx_player)) {
                tzx_pause(tzx_player, 1);
                fprintf(stdout, "[F8] TZX PAUSE\n");
            } else {
                tzx_play(tzx_player);
                fprintf(stdout, "[F8] TZX PLAY\n");
            }
        } else {
            tape.playing = !tape.playing;
            fprintf(stdout, "[F8] Tape %s\n", tape.playing ? "PLAY" : "PAUSE");
        }
    }
    if (f9 && !prev_f9) {
        if (tzx_path && tzx_player) {
            tzx_rewind(tzx_player);
            tzx_play(tzx_player);
            tzx_frame_origin = 0;
            fprintf(stdout, "[F9] TZX REWIND\n");
        } else {
            tape.i_blk = 0; tape.phase = TP_NEXTBLOCK; tape.frame_origin = tape.slice_origin = 0;
            tape.next_edge_at = 0; tape.pause_end_at = 0; tape.ear_level = 1; tape.playing = 1;
            fprintf(stdout, "[F9] Tape REWIND\n");
        }
    }

	if (f11) {
        fast = 0;
		fprintf(stdout, "[F12] SLOW!\n");
		return;
    }
	
	if (f12 && !prev_f12) {
        fast = !fast;
		if (fast)
		{
			fprintf(stdout, "[F12] SPEED!\n");
		} else {
			fprintf(stdout, "[F12] SLOW!\n");
		}
    }
	

    prev_f6 = f6;
    prev_f7 = f7;
    prev_f8 = f8;
    prev_f9 = f9;
	prev_f12 = f12;
}

static void usage(void)
{
    fprintf(stderr, "spectrum: [-f] [-r path] [-d debug] [-A disk] [-B disk]\n"
            "          [-i idedisk] [-I dividerom] [-t tap] [-s sna] [-T tap_pulses]\n"
            "          [-z tzxfile]\n");
    exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
    static struct timespec tc;
    int opt;
    int fd;
    int l;
    char *rompath = (char*)"spectrum.rom";
    char *divpath = (char*)"divide.rom";
    char *idepath = NULL;
    char *tapepath = NULL;
    char *patha = NULL;
    char *pathb = NULL;
    char *snapath = NULL;
    char *tap_pulses_path = NULL;
    char *tzx_path = NULL;

    /* Añadimos 't:' (tap fast), 'T:' (tap pulses) y 'z:' (TZX) */
    while ((opt = getopt(argc, argv, "d:f:r:m:i:I:A:B:s:t:T:z:")) != -1) {
        switch (opt) {
        case 'r':
            rompath = optarg;
            break;
        case 'd':
            trace = atoi(optarg);
            break;
        case 'f':
            fast = 1;
            break;
        case 't':
            tapepath = optarg;
            break;
        case 'T':
            tap_pulses_path = optarg;
            break;
        case 'z':
            tzx_path = optarg;
            break;
        case 'm':
            mem = atoi(optarg);
            break;
        case 'i':
            idepath = optarg;
            break;
        case 'I':
            divpath = optarg;
            break;
        case 'A':
            patha = optarg;
            break;
        case 'B':
            pathb = optarg;
            break;
        case 's':
            snapath = optarg;
            break;
        default:
            usage();
        }
    }
    if (optind < argc)
        usage();

    if (mem < 16 || mem > 48) {
        fprintf(stderr, "spectrum: base memory %dK is out of range.\n", mem);
        exit(1);
    }

    mem *= 1024;
    mem += 16384;

    fd = open(rompath, O_RDONLY);
    if (fd == -1) {
        perror(rompath);
        exit(EXIT_FAILURE);
    }
    l = read(fd, ram, 0x10000);
    switch(l) {
    case 0x4000:
        break;
    case 0x8000:
        model = ZX_128K;
        break;
    case 0x10000:
        model = ZX_PLUS3;
        break;
    default:
        fprintf(stderr, "spectrum: invalid rom '%s'.\n", rompath);
        exit(EXIT_FAILURE);
    }
    close(fd);

    if (model == ZX_PLUS3) {
        fdc = fdc_new();

        lib765_register_error_function(fdc_log);

        if (patha) {
            drive_a = fd_newdsk();
            fd_settype(drive_a, FD_30);
            fd_setheads(drive_a, 1);
            fd_setcyls(drive_a, 40);
            fdd_setfilename(drive_a, patha);
            printf("Attached disk '%s' as A\n", patha);
        } else
            drive_a = fd_new();

        if (pathb) {
            drive_b = fd_newdsk();
            fd_settype(drive_a, FD_35);
            fd_setheads(drive_a, 2);
            fd_setcyls(drive_a, 80);
            fdd_setfilename(drive_a, pathb);
        } else
            drive_b = fd_new();

        fdc_reset(fdc);
        fdc_setisr(fdc, NULL);

        fdc_setdrive(fdc, 0, drive_a);
        fdc_setdrive(fdc, 1, drive_b);
    }

    ui_init();

    window = SDL_CreateWindow("ZX Spectrum",
                  SDL_WINDOWPOS_UNDEFINED,
                  SDL_WINDOWPOS_UNDEFINED,
                  WIDTH,
                  HEIGHT, SDL_WINDOW_RESIZABLE);
    if (window == NULL) {
        fprintf(stderr,
            "spectrum: unable to open window: %s\n",
            SDL_GetError());
        exit(1);
    }
    render = SDL_CreateRenderer(window, -1, 0);
    if (render == NULL) {
        fprintf(stderr,
            "spectrum: unable to create renderer: %s\n",
            SDL_GetError());
        exit(1);
    }
    texture =
        SDL_CreateTexture(render,
                  SDL_PIXELFORMAT_ARGB8888,
                  SDL_TEXTUREACCESS_STREAMING,
                  WIDTH, HEIGHT);
    if (texture == NULL) {
        fprintf(stderr,
            "spectrum: unable to create texture: %s\n",
            SDL_GetError());
        exit(1);
    }
    SDL_SetRenderDrawColor(render, 0, 0, 0, 255);
    SDL_RenderClear(render);
    SDL_RenderPresent(render);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
    SDL_RenderSetLogicalSize(render, WIDTH, HEIGHT);

    matrix = keymatrix_create(8, 5, keyboard);
    keymatrix_trace(matrix, trace & TRACE_KEY);
    keymatrix_add_events(matrix);

    tc.tv_sec = 0;
    tc.tv_nsec = 20000000L; /* 20ms (50Hz frame rate) */

    Z80RESET(&cpu_z80);
    cpu_z80.ioRead = io_read;
    cpu_z80.ioWrite = io_write;
    cpu_z80.memRead = mem_read;
    cpu_z80.memWrite = mem_write;
    cpu_z80.trace = z80_trace;

    /* Initialise DivIDE state and context */
    divide_init(&divide_st);
    divide_ctx.state     = &divide_st;
    divide_ctx.ide       = NULL;
    divide_ctx.get_model = spectrum_get_model;
    divide_ctx.get_mlatch = spectrum_get_mlatch;

    /* Audio beeper */
    if (audio_init_sdl(44100) != 0) {
        fprintf(stderr, "Aviso: audio deshabilitado (SDL_OpenAudioDevice falló).\n");
    } else {
        beeper_frame_origin = 0;
        beeper_slice_origin = 0;
        beeper_last_tstate  = 0;
        beeper_level        = 0;
        beeper_frac_acc     = 0.0;
    }

    /* TAP (fast) – inyección directa */
    if (tapepath) {
        if (!load_tap_fast(tapepath, /*auto_start=*/1, &cpu_z80, mem_write)) {
            fprintf(stderr, "Fallo al cargar TAP (fast): %s\n", tapepath);
        } else {
            printf("TAP (fast) cargado correctamente.\n");
        }
    }

    /* TAP por pulsos – reproducción en EAR para la ROM */
    if (tap_pulses_path) {
        if (tape_load_tap_pulses(&tape, tap_pulses_path) != 0) {
            fprintf(stderr, "Fallo al cargar TAP (pulsos): %s\n", tap_pulses_path);
        } else {
            printf("Reproduciendo cinta por pulsos (ROM std): %s\n", tap_pulses_path);
        }
    }

    /* TZX – reproductor completo de ficheros .tzx */
    if (tzx_path) {
        tzx_player = tzx_create();
        if (!tzx_player) {
            fprintf(stderr, "TZX: error al crear el reproductor.\n");
            exit(EXIT_FAILURE);
        }
        if (tzx_load_file(tzx_player, tzx_path) != 0) {
            fprintf(stderr, "TZX: error al cargar '%s': %s\n",
                    tzx_path, tzx_last_error(tzx_player));
        } else {
            printf("TZX cargado: %s\n", tzx_path);
        }
    }

    if (snapath)
    {
        sna_context_t sna_ctx = {
            .cpu         = &cpu_z80,
            .ram         = ram,
            .border_color = &border_color,
            .mlatch      = &mlatch,
            .mem_write   = mem_write,
            .mem_read    = mem_read,
            .recalc_mmu  = recalc_mmu,
        };
        load_sna(snapath, &sna_ctx);
    }

    if (idepath) {
        ide = ide_allocate("divide0");
        fd = open(idepath, O_RDWR);
        if (fd == -1) {
            perror(idepath);
            exit(1);
        }
        if (ide_attach(ide, 0, fd) == 0)
            ide_reset_begin(ide);
        else {
            fprintf(stderr, "ide: attach failed.\n");
            exit(1);
        }
        if (divide_load_rom(&divide_st, divpath) != 0)
            exit(1);
        divide_ctx.ide = ide;
    }

    while (!emulator_done) {
        /* Hotkeys: F6 (reload TAP & autostart), F7 (list TAP),
                    F8 (play/pause pulses), F9 (rewind pulses) */
        handle_hotkeys(tapepath, tzx_path);

        /* 192 scanlines framebuffer */
        run_scanlines(192, 1);
        /* and border */
        run_scanlines(56, 0);
        spectrum_rasterize();
        spectrum_render();
        Z80INT(&cpu_z80, 0xFF);
        /* 64 scan lines of 224 T states for vblank etc */
        run_scanlines(64, 0);
        poll_irq_event();
        frames++;
        /* Do a small block of I/O and delays */
        if (!fast)
            nanosleep(&tc, NULL);
        if (fdc)
            fdc_tick(fdc);
    }

    if (audio_dev) {
        SDL_CloseAudioDevice(audio_dev);
        audio_dev = 0;
    }
    tzx_destroy(tzx_player);
    exit(0);
}