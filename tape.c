/*
 * tape.c – TAP pulse player and fast loader for ZX Spectrum emulator.
 *
 * TAP format refs:
 *   https://sinclair.wiki.zxnet.co.uk/wiki/TAP_format
 *   https://sinclair.wiki.zxnet.co.uk/wiki/Spectrum_tape_interface
 */

#include "tape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Optional host callback: called before each EAR level transition */
static tape_ear_notify_fn g_tape_ear_notify = NULL;

void tape_set_ear_notify(tape_ear_notify_fn fn)
{
    g_tape_ear_notify = fn;
}

/* Helper: notify and flip level */
#define TAPE_EAR_TOGGLE(t, t_abs) do { \
    int _nl = (t)->ear_level ^ 1; \
    if (g_tape_ear_notify) g_tape_ear_notify((t_abs), _nl); \
    (t)->ear_level = _nl; \
} while (0)

/* ─────────────────────────────────────────────────────────────
 * TAP por pulsos (ROM estándar) - Motor de cinta
 * ───────────────────────────────────────────────────────────── */

static inline uint16_t tape_rd_le16(FILE *f)
{
    int lo = fgetc(f), hi = fgetc(f);
    if (lo == EOF || hi == EOF) return 0;
    return (uint16_t)(lo | (hi << 8));
}

void tape_free(tape_player_t *t)
{
    if (t->blk) {
        for (int i = 0; i < t->nblk; ++i) free(t->blk[i].data);
        free(t->blk);
        t->blk = NULL; t->nblk = 0;
    }
}

int tape_load_tap_pulses(tape_player_t *t, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }

    tape_free(t);

    int count = 0;
    while (1) {
        uint16_t blen = tape_rd_le16(f);
        if (feof(f)) break;
        if (blen == 0 && ferror(f)) { perror("[TAP]"); fclose(f); return -1; }
        fseek(f, blen, SEEK_CUR);
        count++;
    }
    if (count == 0) { fclose(f); fprintf(stderr, "[TAP] vacío.\n"); return -1; }

    t->blk = (tap_block_t *)calloc(count, sizeof(tap_block_t));
    t->nblk = count;

    fseek(f, 0, SEEK_SET);
    for (int i = 0; i < count; ++i) {
        uint16_t blen = tape_rd_le16(f);
        uint8_t *buf = (uint8_t *)malloc(blen);
        if (!buf || fread(buf, 1, blen, f) != blen) {
            fclose(f); tape_free(t); return -1;
        }
        t->blk[i].len  = blen;
        t->blk[i].data = buf;
    }
    fclose(f);

    t->i_blk = 0; t->i_byte = 0; t->bit_mask = 0x80; t->subpulse = 0;
    t->phase = TP_NEXTBLOCK; t->ear_level = 1;
    t->frame_origin = t->slice_origin = 0;
    t->next_edge_at = 0; t->pause_end_at = 0;
    t->playing = 1;
    fprintf(stdout, "TAP (pulsos) cargado: %s (%d bloques)\n", path, t->nblk);
    return 0;
}

int tape_active(const tape_player_t *t)
{
    return t->playing && t->phase != TP_DONE;
}

uint8_t tape_ear_bit6(const tape_player_t *t)
{
    return (tape_active(t) && t->ear_level) ? 0x40 : 0x00;
}

void tape_begin_slice(tape_player_t *t)
{
    t->slice_origin = t->frame_origin;
}

static void tape_advance_to(tape_player_t *t, uint64_t t_now)
{
    if (!tape_active(t)) return;

    while (1) {
        switch (t->phase) {
        case TP_NEXTBLOCK: {
            if (t->i_blk >= t->nblk) { t->phase = TP_DONE; return; }
            uint8_t flag = t->blk[t->i_blk].data[0];
            t->pilot_left = (flag == 0x00) ? PILOT_HDR : PILOT_DATA;
            t->phase = TP_PILOT;
            t->next_edge_at = t_now + T_PILOT;
            TAPE_EAR_TOGGLE(t, t_now);
            t->i_byte = 0; t->bit_mask = 0x80; t->subpulse = 0;
            break;
        }
        case TP_PILOT:
            if (t_now < t->next_edge_at) return;
            TAPE_EAR_TOGGLE(t, t->next_edge_at);
            if (--t->pilot_left > 0) {
                t->next_edge_at += T_PILOT;
            } else {
                t->phase = TP_SYNC1;
                t->next_edge_at += T_SYNC1;
            }
            break;

        case TP_SYNC1:
            if (t_now < t->next_edge_at) return;
            TAPE_EAR_TOGGLE(t, t->next_edge_at);
            t->phase = TP_SYNC2;
            t->next_edge_at += T_SYNC2;
            break;

        case TP_SYNC2:
            if (t_now < t->next_edge_at) return;
            TAPE_EAR_TOGGLE(t, t->next_edge_at);
            t->phase = TP_BITS;
            t->i_byte = 0; t->bit_mask = 0x80; t->subpulse = 0;
            {
                uint8_t b = t->blk[t->i_blk].data[t->i_byte];
                int bit = (b & t->bit_mask) ? 1 : 0;
                t->next_edge_at += bit ? T_BIT1 : T_BIT0;
            }
            break;

        case TP_BITS: {
            if (t_now < t->next_edge_at) return;
            TAPE_EAR_TOGGLE(t, t->next_edge_at);
            uint8_t b = t->blk[t->i_blk].data[t->i_byte];
            int bit = (b & t->bit_mask) ? 1 : 0;
            int tlen = bit ? T_BIT1 : T_BIT0;

            if (t->subpulse == 0) {
                t->subpulse = 1;
                t->next_edge_at += tlen;
            } else {
                t->subpulse = 0;
                if (t->bit_mask == 0x01) {
                    t->bit_mask = 0x80;
                    t->i_byte++;
                } else {
                    t->bit_mask >>= 1;
                }
                if (t->i_byte >= t->blk[t->i_blk].len) {
                    t->phase = TP_PAUSE;
                    t->pause_end_at = t_now + T_MS(T_PAUSE_MS);
                } else {
                    b = t->blk[t->i_blk].data[t->i_byte];
                    bit = (b & t->bit_mask) ? 1 : 0;
                    tlen = bit ? T_BIT1 : T_BIT0;
                    t->next_edge_at += tlen;
                }
            }
            break;
        }

        case TP_PAUSE:
            if (t_now < t->pause_end_at) return;
            t->i_blk++;
            t->phase = TP_NEXTBLOCK;
            break;

        default:
            return;
        }
    }
}

void tape_end_slice(tape_player_t *t, const Z80Context *cpu)
{
    uint64_t t_now = t->slice_origin + (uint64_t)cpu->tstates;
    tape_advance_to(t, t_now);
    t->frame_origin = t_now;
}

/* ─────────────────────────────────────────────────────────────
 * TAP fast loader + listado
 *   - Formato: [len_lo len_hi][flag][payload][checksum XOR]
 *   - Header ROM: 17 bytes -> type, name(10), len_data, p1, p2
 *   - CODE/SCREEN$: carga a p1 (start address) con len_data bytes.
 * Refs: Sinclair Wiki TAP format / Spectrum tape interface
 * ───────────────────────────────────────────────────────────── */

static inline uint16_t rd_le16_file(FILE *f)
{
    int lo = fgetc(f);
    int hi = fgetc(f);
    if (lo == EOF || hi == EOF) return 0;
    return (uint16_t)(lo | (hi << 8));
}

typedef struct {
    uint8_t  type;      /* 0,1,2,3 (PROGRAM, NUM, CHAR, CODE) */
    char     name[11];  /* 10 chars + NUL */
    uint16_t len_data;
    uint16_t p1;        /* para CODE: dirección de carga */
    uint16_t p2;        /* para CODE: normalmente 32768 (0x8000) */
} tap_header_t;

static bool tap_read_header(FILE *f, long block_start_pos, uint16_t blk_len, tap_header_t *out)
{
    (void)block_start_pos;
    uint8_t hdr[17];
    if (fread(hdr, 1, 17, f) != 17) return false;

    /* checksum (1 byte) */
    int chks = fgetc(f); (void)chks;

    out->type = hdr[0];
    memcpy(out->name, &hdr[1], 10);
    out->name[10] = 0;
    out->len_data = (uint16_t)(hdr[11] | (hdr[12] << 8));
    out->p1       = (uint16_t)(hdr[13] | (hdr[14] << 8));
    out->p2       = (uint16_t)(hdr[15] | (hdr[16] << 8));

    if (blk_len != 19) {
        fprintf(stderr, "[TAP] Advertencia: cabecera con len=%u (esperado 19)\n", blk_len);
    }
    return true;
}

/* Lista TAP por consola (no carga) */
void tap_list(const char *path)
{
    if (!path) { fprintf(stderr, "[TAP] No hay fichero (-t)\n"); return; }

    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return; }

    printf("=== TAP LIST: %s ===\n", path);
    int index = 0;
    while (1) {
        uint16_t blk_len = rd_le16_file(f);
        if (feof(f)) break;
        if (blk_len == 0 && ferror(f)) { perror("[TAP]"); break; }

        int flag = fgetc(f);
        if (flag == EOF) break;

        if (flag == 0x00) {
            uint8_t hdr[17];
            if (fread(hdr, 1, 17, f) != 17) { fprintf(stderr, "[TAP] header truncado\n"); break; }
            int ch = fgetc(f); (void)ch;

            uint8_t type = hdr[0];
            char name[11]; memcpy(name, &hdr[1], 10); name[10] = 0;
            uint16_t len_data = (uint16_t)(hdr[11] | (hdr[12] << 8));
            uint16_t p1       = (uint16_t)(hdr[13] | (hdr[14] << 8));
            uint16_t p2       = (uint16_t)(hdr[15] | (hdr[16] << 8));

            const char *tname = (type==0?"PROGRAM":(type==1?"NUMARRAY":(type==2?"CHARARRAY":(type==3?"CODE":"?"))));
            printf(" [%03d] HEADER  len=%u  type=%s  name=\"%.*s\"  data=%u  p1=%u  p2=%u\n",
                   index++, blk_len, tname, 10, name, len_data, p1, p2);
        } else if (flag == 0xFF) {
            size_t toread = (blk_len >= 1) ? (blk_len - 1) : 0;
            if (toread >= 1) {
                fseek(f, (long)(toread - 1), SEEK_CUR);
                int cs = fgetc(f); (void)cs;
            }
            printf(" [%03d] DATA    len=%u\n", index++, blk_len);
        } else {
            size_t skip = (blk_len >= 1) ? (blk_len - 1) : 0;
            if (skip) fseek(f, (long)skip, SEEK_CUR);
            printf(" [%03d] FLAG=0x%02X (saltado) len=%u\n", index++, flag, blk_len);
        }
    }

    fclose(f);
}

/* Carga CODE/SCREEN$ de un TAP. Autostart opcional (PC := start address del último CODE) */
bool load_tap_fast(const char *path, int auto_start,
                   Z80Context *cpu, tape_mem_write_fn mem_write)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return false; }

    long block_start = 0;

    printf("=== TAP: %s ===\n", path);

    tap_header_t last_hdr = {0};
    bool have_hdr = false;
    bool loaded_any_code = false;
    uint16_t last_code_start = 0;

    while (1) {
        uint16_t blk_len = rd_le16_file(f);
        if (feof(f)) break;
        if (blk_len == 0 && ferror(f)) { fclose(f); return false; }

        block_start = ftell(f);
        int flag = fgetc(f);
        if (flag == EOF) break;

        if (flag == 0x00) {
            tap_header_t H;
            if (!tap_read_header(f, block_start, blk_len, &H)) { fclose(f); return false; }
            have_hdr = true; last_hdr = H;

            const char *tname = (H.type==0?"PROGRAM":(H.type==1?"NUMARRAY":(H.type==2?"CHARARRAY":(H.type==3?"CODE":"?"))));
            printf(" - HEADER: type=%s  name=\"%.*s\"  len_data=%u  p1=%u  p2=%u\n",
                   tname, 10, last_hdr.name, last_hdr.len_data, last_hdr.p1, last_hdr.p2);
        }
        else if (flag == 0xFF) {
            if (!have_hdr) {
                size_t toread = (blk_len >= 1) ? (blk_len - 1) : 0; /* payload + checksum */
                if (toread > 0) fseek(f, (long)toread, SEEK_CUR);
                printf(" - DATA sin header previo: saltado (%u bytes)\n", blk_len);
                continue;
            }

            /* payload (dsz = blk_len-2 bytes de datos + 1 checksum al final) */
            uint16_t payload_len = (blk_len >= 2) ? (blk_len - 2) : 0;
            if (payload_len < 1) { fclose(f); fprintf(stderr, "[TAP] DATA tamaño inválido.\n"); return false; }

            uint8_t *buf = (uint8_t *)malloc(payload_len);
            if (!buf) { fclose(f); return false; }

            size_t rd = fread(buf, 1, payload_len, f);
            if (rd != payload_len) { free(buf); fclose(f); return false; }

            uint8_t *data = buf;
            size_t   dsz  = (payload_len >= 1) ? (payload_len - 1) : 0; /* sin checksum final */
            uint16_t declared = last_hdr.len_data;

            if (last_hdr.type == 3) {
                uint16_t start = last_hdr.p1;
                size_t tocopy = (declared <= dsz) ? declared : dsz;
                printf("   · Cargando CODE en 0x%04X (%zu bytes)\n", start, tocopy);

                for (size_t i = 0; i < tocopy; ++i) {
                    mem_write(0, (uint16_t)(start + i), data[i]);
                }
                loaded_any_code = true;
                last_code_start = start;
            } else {
                printf("   · DATA type=%u no cargado (soportado solo CODE por ahora)\n", last_hdr.type);
            }

            free(buf);
            have_hdr = false;
        }
        else {
            size_t skip = (blk_len >= 1) ? (blk_len - 1) : 0;
            if (skip) fseek(f, (long)skip, SEEK_CUR);
            printf(" - FLAG 0x%02X no estándar: saltado (%u bytes)\n", flag, blk_len);
        }
    }

    fclose(f);

    if (loaded_any_code) {
        if (auto_start) {
            cpu->PC = last_code_start;
            printf("AUTO-START: PC := 0x%04X\n", cpu->PC);
        }
    } else {
        fprintf(stderr, "[TAP] No se cargó ningún bloque CODE.\n");
        return false;
    }

    return true;
}
