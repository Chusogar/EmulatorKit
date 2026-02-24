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
#include "tape.h"
#include "sna.h"

#include <SDL2/SDL.h>
#include "event.h"
#include "keymatrix.h"
#include "ay8912.h"

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

static uint8_t divmem[524288];/* Full 512K emulated */
static uint8_t divrom[524288];
static uint8_t divide_latch;
static unsigned divide_mapped; /* 0 no, 1 yes */
static unsigned divide_oe;
static unsigned divide_pair;   /* Latches other half of wordstream for IDE */
static unsigned divide;

static uint8_t divplus_latch;
static unsigned divplus_128k = 1;
static unsigned divplus_7ffd;

#define ZX_48K_2    0
#define ZX_48K_3    1
#define ZX_128K     2
#define ZX_PLUS3    3
static unsigned model = ZX_48K_3;

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
static float tape_volume   = 0.15f;  /* volume for tape EAR-in signal */
static float ay_volume     = 0.50f;  /* volume for AY-3-8912 output */

static uint64_t beeper_frame_origin = 0;
static uint64_t beeper_slice_origin = 0;
static uint64_t beeper_last_tstate  = 0;
static int      beeper_level        = 0;   /* 0 o 1 (onda cuadrada) */
static int      tape_ear_level      = 0;   /* 0 o 1: EAR input from tape/TZX */
static int      tape_ear_active     = 0;   /* 1 when tape/TZX is playing */
static double   beeper_frac_acc     = 0.0;

/* AY-3-8912 PSG (128K/+3 only; NULL on 48K). */
static ay8912_t *ay = NULL;

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

    /* Mix beeper (EAR/MIC out) with tape EAR-in signal.
     * Gate the tape contribution so silence when nothing is playing. */
    float bv = beeper_level ? beeper_volume : -beeper_volume;
    float tv = tape_ear_active ? (tape_ear_level ? tape_volume : -tape_volume) : 0.0f;

    enum { CHUNK = 4096 };
    static int16_t buf[CHUNK];
    while (nsamp > 0) {
        int n = (nsamp > CHUNK) ? CHUNK : nsamp;
        if (ay) {
            /* 128K/+3: step the AY PSG once per sample and mix. */
            for (int i = 0; i < n; ++i) {
                float mixed = bv + tv +
                    (float)ay8912_calc(ay) * ay_volume / AY8912_MAX_OUTPUT;
                if (mixed >  1.0f) mixed =  1.0f;
                if (mixed < -1.0f) mixed = -1.0f;
                buf[i] = (int16_t)(mixed * 32767.0f);
            }
        } else {
            float mixed = bv + tv;
            if (mixed >  1.0f) mixed =  1.0f;
            if (mixed < -1.0f) mixed = -1.0f;
            int16_t val = (int16_t)(mixed * 32767.0f);
            for (int i = 0; i < n; ++i) buf[i] = val;
        }
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
 * Tape EAR-in audio callback.
 * Called by tape.c / tzx.c before each EAR level change so the
 * audio generator is flushed to the exact edge time first.
 * ───────────────────────────────────────────────────────────── */
static void on_tape_ear_change(uint64_t t_abs, int new_level)
{
    beeper_advance_to(t_abs);   /* flush audio up to this edge */
    tape_ear_level = new_level;
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

static uint8_t *divbank(unsigned bank, unsigned page, unsigned off)
{
    bank <<= 2;
    bank |= page;
    return &divmem[bank * 0x2000 + (off & 0x1FFF)];
}

static uint8_t *divide_getmap(unsigned addr, unsigned w)
{
    unsigned bank = 0;
    if (divide == 2) {
        switch(divplus_latch & 0xC0) {
        case 0x00:  /* DivIDE mode */
            bank = (divplus_latch >> 1) & 0x0F;
            break;
        case 0x40:
            if (w & (divplus_latch & 0x20))
                return NULL;
            return &divmem[((divplus_latch & 0x1F) << 14) +
                (addr & 0x3FFF)];
        case 0x80:
            if (w)
                return NULL;
            return &divrom[((divplus_latch & 0x1F) << 14) +
                (addr & 0x3FFF)];
        }
    }
    /* TODO mapmem should probably stop RAM 3 writes without CONMEM
       even if 2000-3FFF */
    if (addr & 0x2000)
        return divbank(bank, divide_latch & 3, addr);
    /* CONMEM */
    if (divide_latch & 0x80) {
        if (w)
            return NULL;
        if (divide == 2)
            return divrom + bank * 0x8000 + 0x6000 + (addr & 0x1FFF);
        else
            return divrom + (addr & 0x1FFF);
    }
    /* MAPMEM */
    if (divide_latch & 0x40) {
        if (w)
            return NULL;
        return divbank(bank, 3, addr);
    }
    if (divide == 2)
        return divrom + bank * 0x8000 + 0x6000 + (addr & 0x1FFF);
    else
        return divrom + (addr & 0x1FFF);
}

static void divide_write(uint16_t addr, uint8_t val)
{
    uint8_t *p = divide_getmap(addr, 1);
    if (p)
        *p = val;
}

static uint8_t divide_read(uint16_t addr)
{
    return *divide_getmap(addr, 0);
}

/* TODO: memory contention */
static uint8_t do_mem_read(uint16_t addr, unsigned debug)
{
    unsigned bank = map[addr >> 14];
    /* For now until banked stuff */
    if (addr >= mem)
        return 0xFF;
    if (addr < 0x4000 && divide_mapped == 1)
        return divide_read(addr);
    return ram[bank][addr & 0x3FFF];
}

static void mem_write(int unused, uint16_t addr, uint8_t val)
{
    unsigned bank = map[addr >> 14];
    if (addr >= mem)
        return;
    if (addr < 0x4000 && divide_mapped == 1) {
        divide_write(addr, val);
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
    if (cpu_z80.M1 && !(divplus_latch & 0xC0)) {
        /* Immediate map */
        if (divide == 1 && addr >= 0x3D00 && addr <= 0x3DFF)
            divide_mapped = 1;
        /* TODO: correct this based on the B4 latch and 128K flag */
        if (divide == 2 && (model <= ZX_48K_3 || !divplus_128k || (mlatch & 0x10)) && addr >= 0x3D00 && addr <= 0x3DFF)
            divide_mapped = 1;
    }

    r = do_mem_read(addr, 0);

    /* Look for ED with M1, followed directly by 4D and if so trigger
       the interrupt chain */
    if (cpu_z80.M1) {
        if (!(divplus_latch & 0xC0)) {
            /* ROM paging logic */
            if (divide && addr >= 0x1FF8 && addr <= 0x1FFF)
                divide_mapped = 0;
            if (divide && (addr == 0x0000 || addr == 0x0008 || addr == 0x0038 ||
                addr == 0x0066 || addr == 0x04C6 || addr == 0x0562))
                divide_mapped = 1;
        }
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

/* ─────────────────────────────────────────────────────────────
 * Cycle-accurate (t-state) border rasterizer
 *
 * Timing model (PAL, from interrupt = t 0):
 *   48K  : 224 t-states/line, 312 lines/frame (69 888 t-states/frame)
 *   128K/+3: 228 t-states/line, 312 lines/frame (71 136 t-states/frame)
 *
 * Visible frame layout (line numbers relative to INT):
 *   lines   0 – 15  : top retrace   (invisible)
 *   lines  16 – 47  : top border    → texture rows 0 .. BORDER-1
 *   lines  48 – 63  : top overscan  (invisible)
 *   lines  64 – 255 : screen        → texture rows BORDER .. BORDER+191
 *                      (VRAM-rasterised at end of frame; only left/right
 *                       border columns are written here)
 *   lines 256 – 287 : bottom border → texture rows BORDER+192 .. HEIGHT-1
 *   lines 288 – 311 : bottom retrace (invisible)
 *
 * Horizontal timing within a line (48K, 224 t-states/line):
 *   t  0 – 23  : left border  (24 t-states → BORDER=32 pixels)
 *   t 24 – 151 : active video (128 t-states → 256 pixels; not drawn here)
 *   t 152 – 175: right border (24 t-states → BORDER=32 pixels)
 *   t 176 – 223: horizontal flyback (invisible)
 * For 128K/+3 (228 t-states/line) the left/right border spans are 26
 * t-states each; the flyback accounts for the extra 4 t-states.
 * ───────────────────────────────────────────────────────────── */

/* First visible line numbers within the 312-line frame (from INT) */
#define FIRST_TOP_LINE  16u   /* first top-border line  */
#define FIRST_SCR_LINE  64u   /* first screen line      */
#define FIRST_BOT_LINE  256u  /* first bottom-border line */

/* T-states per scanline for the active model */
static inline bool is_48k_model(void)
{
    return model == ZX_48K_2 || model == ZX_48K_3;
}

static inline unsigned tstates_per_line(void)
{
    return is_48k_model() ? 224u : 228u;
}

/* Horizontal border timing: left/right border t-state span per model.
 * Active video always spans 128 t-states (256 pixels at 2 t-states/pixel).
 * Returns 24 for 48K or 26 for 128K/+3 – never 0. */
static inline unsigned h_border_t(void)
{
    return is_48k_model() ? 24u : 26u;
}

/* Border rasterizer state */
static uint64_t brd_frame_org = 0; /* absolute t-state at frame start    */
static uint64_t brd_slice_org = 0; /* absolute t-state at slice start    */
static uint64_t brd_drawn_to  = 0; /* absolute t-state drawn up to       */

/* Advance border drawing from brd_drawn_to up to t_abs (absolute t-state).
 * Uses the current border_color for all newly drawn pixels.
 *
 * Invariant: brd_frame_org <= brd_drawn_to <= t_abs.
 * border_begin_frame() sets brd_frame_org = brd_drawn_to; after that
 * brd_drawn_to only grows, so the unsigned subtractions below never wrap. */
static void border_advance_to(uint64_t t_abs)
{
    if (t_abs <= brd_drawn_to)
        return;
    /* Safety: ensure we never underflow the frame-relative conversion */
    if (brd_drawn_to < brd_frame_org)
        brd_drawn_to = brd_frame_org;

    unsigned tpl  = tstates_per_line();
    unsigned hbt  = h_border_t();          /* left/right border span (t-states) */
    unsigned hle  = hbt;                   /* left border end within line        */
    unsigned hse  = hle + 128u;            /* screen end (= right border start)  */
    unsigned hbe  = hse + hbt;             /* right border end (flyback starts)  */

    uint32_t col  = palette[border_color & 0x0Fu];

    /* Work in frame-relative t-states */
    uint64_t ft     = brd_drawn_to - brd_frame_org;
    uint64_t ft_end = t_abs        - brd_frame_org;

    while (ft < ft_end) {
        unsigned line   = (unsigned)(ft / tpl);
        unsigned col_t  = (unsigned)(ft % tpl);

        /* End of current line in frame-relative t-states */
        uint64_t line_end_ft = (uint64_t)(line + 1u) * tpl;
        uint64_t seg_end_ft  = (ft_end < line_end_ft) ? ft_end : line_end_ft;
        unsigned col_t_end   = (unsigned)(seg_end_ft - (uint64_t)line * tpl);

        /* Map line to texture row and row type */
        int      tex_row  = -1;
        int      row_type = -1; /* 0 = full-width top/bottom, 1 = screen left+right */

        if (line >= FIRST_TOP_LINE && line < FIRST_TOP_LINE + BORDER) {
            tex_row  = (int)(line - FIRST_TOP_LINE);
            row_type = 0;
        } else if (line >= FIRST_SCR_LINE && line < FIRST_SCR_LINE + 192u) {
            tex_row  = (int)(BORDER + (line - FIRST_SCR_LINE));
            row_type = 1;
        } else if (line >= FIRST_BOT_LINE && line < FIRST_BOT_LINE + BORDER) {
            tex_row  = (int)(BORDER + 192u + (line - FIRST_BOT_LINE));
            row_type = 0;
        } else {
            /* Invisible line (retrace / overscan) – skip to next line */
            ft = line_end_ft;
            continue;
        }

        if (row_type == 0) {
            /* Full-width top/bottom border row.
             * Map t-states [0, hbe) linearly to pixels [0, WIDTH).
             * Anything in the flyback region [hbe, tpl) is not drawn. */
            if (col_t < hbe) {
                unsigned ce = (col_t_end < hbe) ? col_t_end : hbe;
                unsigned x0 = (unsigned)((uint64_t)col_t  * WIDTH / hbe);
                unsigned x1 = (unsigned)((uint64_t)ce     * WIDTH / hbe);
                if (x1 > (unsigned)WIDTH) x1 = (unsigned)WIDTH;
                uint32_t *p = texturebits + tex_row * WIDTH + (int)x0;
                for (unsigned x = x0; x < x1; x++)
                    *p++ = col;
            }
        } else {
            /* Screen row: only the left and right border columns are drawn.
             *   Left border : t [0, hle)   → pixel x [0, BORDER)
             *   Right border: t [hse, hbe) → pixel x [BORDER+256, WIDTH) */

            /* Left border */
            if (col_t < hle) {
                unsigned ce = (col_t_end < hle) ? col_t_end : hle;
                unsigned x0 = (unsigned)((uint64_t)col_t * BORDER / hle);
                unsigned x1 = (unsigned)((uint64_t)ce    * BORDER / hle);
                if (x1 > BORDER) x1 = BORDER;
                uint32_t *p = texturebits + tex_row * WIDTH + (int)x0;
                for (unsigned x = x0; x < x1; x++)
                    *p++ = col;
            }

            /* Right border */
            if (col_t_end > hse && col_t < hbe) {
                unsigned cs = (col_t  > hse) ? col_t     : hse;
                unsigned ce = (col_t_end < hbe) ? col_t_end : hbe;
                unsigned x0 = (unsigned)(BORDER + 256u +
                               (uint64_t)(cs - hse) * BORDER / hbt);
                unsigned x1 = (unsigned)(BORDER + 256u +
                               (uint64_t)(ce - hse) * BORDER / hbt);
                if (x1 > (unsigned)WIDTH) x1 = (unsigned)WIDTH;
                uint32_t *p = texturebits + tex_row * WIDTH + (int)x0;
                for (unsigned x = x0; x < x1; x++)
                    *p++ = col;
            }
        }

        ft = seg_end_ft;
    }

    brd_drawn_to = t_abs;
}

/* Called once at the start of each 312-line frame (after INT fires). */
static inline void border_begin_frame(void)
{
    brd_frame_org = brd_drawn_to;
}

/* Called before each Z80ExecuteTStates slice. */
static inline void border_begin_slice(void)
{
    brd_slice_org = brd_drawn_to;
}

/* Called after each Z80ExecuteTStates slice. */
static inline void border_end_slice(void)
{
    uint64_t t_now = brd_slice_org + (uint64_t)cpu_z80.tstates;
    border_advance_to(t_now);
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

    /* Catch up border drawing to the current t-state, then apply new color */
    uint64_t t_now = brd_slice_org + (uint64_t)cpu_z80.tstates;
    border_advance_to(t_now);
    border_color = v & 7;
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

static void divplus_ctrl(uint8_t val)
{
    divplus_latch = val;
    switch(val & 0xE0) {
    case 0xC0:
    case 0xE0:
        divide_latch = 0;
        divplus_latch = 0;
        divplus_7ffd = 0;
        divide_mapped = 0;
        break;
    case 0x00:
        /* DivIDE mode */
        /* bits 4-1 selects the extended banking */
        break;
    case 0x20:
        /* Enable 128K mode */
        divplus_128k = 1;
        break;
        /* RAM mode: 10WAAAAA */
        /* 32K pages x 16K replace Spectrum ROM, DivIDE traps off */
    case 0x40:
    case 0x60:
        break;
    case 0x80:
        /* ROM mode: as above for 16K ROM banks */
    case 0xA0:
        break;
    }
}

static uint8_t io_read(int unused, uint16_t addr)
{
    unsigned r;

    /* Kempston joystick: puerto 0x1F */
    if ((addr & 0xFF) == 0x1F) {
        return kempston_state_from_sdl();
    }

    /* Timex checks XXFE, Sinclair just the low bit */
    if ((addr & 0x01) == 0) /* ULA */
        return ula_read(addr);
    /* AY-3-8912: IN 0xFFFD reads the currently selected register (128K/+3 only).
     * Advance audio to current t-state first to preserve event ordering. */
    if ((model == ZX_128K || model == ZX_PLUS3) && (addr & 0xC002) == 0xC000) {
        if (ay) {
            uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
            beeper_advance_to(t_now);
            return ay8912_read_data(ay);
        }
        return 0xFF;
    }
    if (model == ZX_PLUS3) {
        if ((addr & 0xF002) == 0x2000)
            return fdc_read_ctrl(fdc);
        if ((addr & 0xF002) == 0x3000)
            return fdc_read_data(fdc);
    }
    if (divide) {
        if ((addr & 0xE3) == 0xA3) {
            r = (addr >> 2) & 0x07;
            if (r) {
                divide_oe = 1;  /* Odd mode */
                return ide_read16(ide, r);
            }
            if (divide_oe == 0) {
                divide_oe = 1;
                return divide_pair;
            }
            r = ide_read16(ide, 0);
            divide_pair = r >> 8;
            divide_oe = 0;
            return r & 0xFF;
        }
    }
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
    /* AY-3-8912 ports (128K/+3 only).
     * Advance audio to the current t-state before changing AY state so
     * that the register write lands at the correct position in the stream. */
    if ((model == ZX_128K || model == ZX_PLUS3) && ay) {
        if ((addr & 0xC002) == 0xC000) {
            /* OUT 0xFFFD: select AY register */
            uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
            beeper_advance_to(t_now);
            ay8912_select_reg(ay, val);
        } else if ((addr & 0xC002) == 0x8000) {
            /* OUT 0xBFFD: write data to selected AY register */
            uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
            beeper_advance_to(t_now);
            ay8912_write_data(ay, val);
        }
    }
    if (divide) {
        if ((addr & 0xE3) == 0xA3) {
            uint8_t r = (addr >> 2) & 0x07;
            if (r) {
                divide_oe = 1;  /* Odd mode */
                ide_write16(ide, r, val);
            } else if (divide_oe == 1) {
                divide_oe = 0;
                divide_pair = val;
            } else {
                ide_write16(ide, 0, divide_pair | (val << 8));
                divide_oe = 0;
            }
        }
        if ((addr & 0xE3) == 0xE3) {
            /* MAPRAM cannot be cleared */
            val |= divide_latch & 0x40;
            divide_latch = val;
            if (val & 0x80)
                divide_mapped = 1;
        }
        if (divide ==2 && (addr & 0xFF) == 0x17)
            divplus_ctrl(val);
    }
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
    unsigned tpl = tstates_per_line(); /* PAL t-states/line: 224 (48K) or 228 (128K/+3) */
    unsigned n   = tpl;

    blanked = blank;

    if (!blanked)
        drawline = 0;
    /* Run scanlines */
    for (i = 0; i < lines; i++) {
        /* Delimitamos porciones (beeper + cinta + border) alrededor de la ejecución */
        beeper_begin_slice();
        border_begin_slice();
        tape_begin_slice(&tape);
        if (tzx_player) {
            tzx_begin_slice(tzx_player, tzx_frame_origin);
        }

        /* Update tape_ear_active once per scanline */
        tape_ear_active = tape_active(&tape) || (tzx_player && tzx_active(tzx_player));

        n = tpl + tpl - Z80ExecuteTStates(&cpu_z80, n);

        /* Tape/TZX end first: their callbacks advance the beeper audio
         * to each edge time before beeper_end_slice() flushes the rest. */
        tape_end_slice(&tape, &cpu_z80);
        if (tzx_player) {
            tzx_end_slice(tzx_player, &cpu_z80, &tzx_frame_origin);
        }
        border_end_slice();
        beeper_end_slice();

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
            /* Anchor TZX to the current absolute emulator time so callbacks
             * and beeper_advance_to() share the same time base. */
            tzx_frame_origin = beeper_frame_origin;
            fprintf(stdout, "[F9] TZX REWIND\n");
        } else {
            tape.i_blk = 0; tape.phase = TP_NEXTBLOCK;
            /* Same time-base synchronisation for TAP rewind */
            tape.frame_origin = tape.slice_origin = beeper_frame_origin;
            tape.next_edge_at = 0; tape.pause_end_at = 0;
            tape.ear_level = 1; tape.playing = 1;
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

    /* Audio beeper */
    if (audio_init_sdl(44100) != 0) {
        fprintf(stderr, "Aviso: audio deshabilitado (SDL_OpenAudioDevice falló).\n");
    } else {
        beeper_frame_origin = 0;
        beeper_slice_origin = 0;
        beeper_last_tstate  = 0;
        beeper_level        = 0;
        beeper_frac_acc     = 0.0;
        /* AY-3-8912: present on 128K/+3 only, clocked at CPU_CLK/2. */
        if (model == ZX_128K || model == ZX_PLUS3) {
            ay = ay8912_create((uint32_t)(TSTATES_CPU / 2), (uint32_t)audio_rate);
            if (!ay)
                fprintf(stderr, "Aviso: no se pudo inicializar el AY-3-8912.\n");
        }
    }

    /* TAP (fast) – inyección directa */
    if (tapepath) {
        if (!load_tap_fast(tapepath, /*auto_start=*/1, &cpu_z80, mem_write)) {
            fprintf(stderr, "Fallo al cargar TAP (fast): %s\n", tapepath);
        } else {
            printf("TAP (fast) cargado correctamente.\n");
        }
    }

    /* Register tape EAR-in audio callback for mixing */
    tape_set_ear_notify(on_tape_ear_change);

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
        tzx_set_ear_notify(tzx_player, on_tape_ear_change);
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
        fd = open(divpath, O_RDONLY);
        if (fd == -1) {
            perror(divpath);
            exit(1);
        }
        l = read(fd, divrom, 524288);
        if (l == 8192)
            divide = 1;
        else if (l == 524288)
            divide = 2;
        else {
            fprintf(stderr, "spectrum: divide.rom invalid.\n");
            exit(1);
        }
    }

    while (!emulator_done) {
        /* Hotkeys: F6 (reload TAP & autostart), F7 (list TAP),
                    F8 (play/pause pulses), F9 (rewind pulses) */
        handle_hotkeys(tapepath, tzx_path);

        /*
         * Run one full PAL frame (312 lines) with model-correct t-states/line.
         * Frame layout from INT (t = 0):
         *   lines   0 –  63 : top area  (retrace + BORDER=32 visible rows)
         *   lines  64 – 255 : screen    (192 lines; VRAM rasterised at end)
         *   lines 256 – 311 : bottom area (BORDER=32 visible rows + retrace)
         */
        border_begin_frame();
        run_scanlines(64, 0);
        run_scanlines(192, 1);
        run_scanlines(56, 0);
        spectrum_rasterize();
        spectrum_render();
        Z80INT(&cpu_z80, 0xFF);
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
    ay8912_destroy(ay);
    ay = NULL;
    tzx_destroy(tzx_player);
    exit(0);
}