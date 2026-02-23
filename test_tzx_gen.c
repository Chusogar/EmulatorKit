/*
 * test_tzx_gen.c – Unit tests for TZX block 0x19 (Generalised Data).
 *
 * Tests construct synthetic TZX files, load them via tzx_load_file(), and
 * advance the player in controlled T-state increments while recording every
 * EAR level change via the notify callback.  The recorded edge timestamps are
 * compared against analytically computed expected values.
 *
 * Build & run:
 *   make test_tzx_gen && ./test_tzx_gen
 *
 * TZX 0x19 block layout (all little-endian):
 *   ID    : 1 byte  – 0x19
 *   BLEN  : 4 bytes – number of bytes that follow
 *   PAUSE : 2 bytes – pause after block (ms)
 *   TOTP  : 4 bytes – number of entries in the pilot stream
 *   NPP   : 1 byte  – max pulses per pilot symbol (0 = no pilot)
 *   ASP   : 1 byte  – symbols in pilot alphabet (0 = 256)
 *   TOTD  : 4 bytes – total data symbols to emit
 *   NPD   : 1 byte  – max pulses per data symbol
 *   ASD   : 1 byte  – symbols in data alphabet (0 = 256)
 *   [pilot symbol table: ASP × (1 + NPP×2) bytes]
 *   [pilot stream:       TOTP × 3 bytes (1 sym_idx + 2 rep_count LE)]
 *   [data symbol table:  ASD  × (1 + NPD×2) bytes]
 *   [data stream:        ceil(TOTD × ceil_log2(ASD) / 8) bytes]
 *
 * Symbol flags (bits 1..0) – code convention:
 *   0 = toggle EAR at symbol start
 *   1 = keep current EAR level
 *   2 = force EAR low
 *   3 = force EAR high
 * Then each non-zero pulse in the symbol's pulse array toggles EAR.
 *
 * Timing invariants verified:
 *   - No drift within a multi-pulse symbol: each pulse is relative to the
 *     previously scheduled edge, not to the caller's t_now.
 *   - No drift between symbols: the next symbol starts exactly where the
 *     previous one ended.
 *   - Pause after block is anchored to the last signal edge, not t_now.
 *
 * All tests initialise the block at t=0 by calling step(tp,&origin,0) first,
 * which triggers tzx_advance_to(tp,0) and anchors slice_origin=0.
 */

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>  /* getpid() */

#include "tzx.h"
#include "libz80/z80.h"  /* for Z80Context.tstates */

/* ─── helpers ─────────────────────────────────────────────────────────────── */

static void put_le16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static void put_le32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v       & 0xFF); p[1] = (uint8_t)((v >>  8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* Build TZX header + single 0x19 block into caller-supplied buffer.
 * Returns total byte count written. */
static size_t build_tzx_gen(uint8_t* buf,
    uint16_t pause_ms,
    uint32_t totp, uint8_t npp, uint8_t asp,
    const uint8_t* pilot_sym_table,   /* asp*(1+npp*2) bytes, NULL if totp==0 */
    const uint8_t* pilot_stream,      /* totp*3 bytes, NULL if totp==0 */
    uint32_t totd, uint8_t npd, uint8_t asd,
    const uint8_t* data_sym_table,    /* asd*(1+npd*2) bytes, NULL if totd==0 */
    const uint8_t* data_stream,       /* data_stream_bytes bytes, NULL if totd==0 */
    uint32_t data_stream_bytes)
{
    /* TZX file header */
    memcpy(buf, "ZXTape!", 7);
    buf[7] = 0x1A; buf[8] = 1; buf[9] = 21;
    size_t p = 10;

    buf[p++] = 0x19;  /* block ID */

    uint32_t pilot_sym_size   = totp > 0 ? (uint32_t)asp * (1u + 2u * npp) : 0u;
    uint32_t pilot_stream_sz  = totp * 3u;
    uint32_t data_sym_size    = totd > 0 ? (uint32_t)asd * (1u + 2u * npd) : 0u;

    /* blen = bytes that follow the 4-byte blen field */
    uint32_t blen = 2u + 4u + 1u + 1u + 4u + 1u + 1u   /* fixed 14-byte sub-header */
                  + pilot_sym_size + pilot_stream_sz
                  + data_sym_size  + data_stream_bytes;

    put_le32(buf+p, blen);    p += 4;
    put_le16(buf+p, pause_ms);p += 2;
    put_le32(buf+p, totp);    p += 4;
    buf[p++] = npp;
    buf[p++] = asp;
    put_le32(buf+p, totd);    p += 4;
    buf[p++] = npd;
    buf[p++] = asd;

    if (totp > 0) {
        memcpy(buf+p, pilot_sym_table, pilot_sym_size); p += pilot_sym_size;
        memcpy(buf+p, pilot_stream,    pilot_stream_sz); p += pilot_stream_sz;
    }
    if (totd > 0) {
        memcpy(buf+p, data_sym_table, data_sym_size);       p += data_sym_size;
        memcpy(buf+p, data_stream,    data_stream_bytes);   p += data_stream_bytes;
    }
    return p;
}

/* Write raw bytes to a temp file and return its path (static buffer). */
static const char* write_tmp_tzx(const uint8_t* data, size_t len)
{
    static char path[256];
    snprintf(path, sizeof(path), "/tmp/test_tzx_gen_%d.tzx", (int)getpid());
    FILE* f = fopen(path, "wb");
    assert(f != NULL);
    size_t w = fwrite(data, 1, len, f);
    fclose(f);
    assert(w == len);
    return path;
}

/* ─── edge recording ──────────────────────────────────────────────────────── */

#define MAX_EDGES 4096
static uint64_t g_edge_times[MAX_EDGES];
static int      g_edge_levels[MAX_EDGES];
static int      g_edge_count = 0;

static void record_edge(uint64_t t_abs, int new_level)
{
    assert(g_edge_count < MAX_EDGES);
    g_edge_times[g_edge_count]  = t_abs;
    g_edge_levels[g_edge_count] = new_level;
    g_edge_count++;
}

/*
 * Advance the player by `delta` T-states using the public slice API.
 * *origin accumulates the absolute T-state counter.
 */
static void step(tzx_player_t* tp, uint64_t* origin, uint64_t delta)
{
    Z80Context cpu;
    memset(&cpu, 0, sizeof(cpu));
    cpu.tstates = (unsigned)delta;
    tzx_begin_slice(tp, *origin);
    tzx_end_slice(tp, &cpu, origin);
}

/* ─── test 1: pilot-only block ────────────────────────────────────────────── */
/*
 * Block: totp=1 entry (rep=4), pilot symbol = flags=0 (toggle), pulse=2168.
 * No data, no pause.
 *
 * Expected EAR edges (initial ear_level=1, block anchored at t=0):
 *   Each repetition starts with a flags toggle then fires one pulse edge after
 *   2168 T-states.  Consecutive symbols chain exactly from the previous end.
 *
 *   t=    0, lv=0  (rep1 flags toggle: 1→0)
 *   t= 2168, lv=1  (rep1 pulse end:    0→1)
 *   t= 2168, lv=0  (rep2 flags toggle: 1→0)  ← same timestamp, chained
 *   t= 4336, lv=1  (rep2 pulse end)
 *   t= 4336, lv=0  (rep3 flags toggle)
 *   t= 6504, lv=1  (rep3 pulse end)
 *   t= 6504, lv=0  (rep4 flags toggle)
 *   t= 8672, lv=1  (rep4 pulse end)
 */
static void test_pilot_only(void)
{
    printf("test_pilot_only ... ");
    fflush(stdout);

    uint8_t pilot_sym[3];
    pilot_sym[0] = 0x00;            /* flags = toggle */
    put_le16(pilot_sym+1, 2168);

    uint8_t pilot_stream[3];
    pilot_stream[0] = 0;            /* sym_idx = 0 */
    put_le16(pilot_stream+1, 4);    /* rep = 4 */

    uint8_t buf[256];
    size_t len = build_tzx_gen(buf, 0,
        1, 1, 1, pilot_sym, pilot_stream,
        0, 0, 0, NULL, NULL, 0);

    tzx_player_t* tp = tzx_create();
    assert(tp != NULL);
    tzx_set_ear_notify(tp, record_edge);
    g_edge_count = 0;
    assert(tzx_load_file(tp, write_tmp_tzx(buf, len)) == 0);

    uint64_t origin = 0;

    /* step(0): initialise block at t=0; fires the first flags toggle at t=0. */
    step(tp, &origin, 0);
    assert(tzx_active(tp));

    /* step(10000): t_now=10000; all remaining edges (<8672) are processed. */
    step(tp, &origin, 10000);
    assert(!tzx_active(tp));

    assert(g_edge_count == 8);
    const uint64_t et[] = { 0, 2168, 2168, 4336, 4336, 6504, 6504, 8672 };
    const int      el[] = { 0,    1,    0,    1,    0,    1,    0,    1  };
    for (int i = 0; i < 8; i++) {
        if (g_edge_times[i] != et[i] || g_edge_levels[i] != el[i]) {
            fprintf(stderr,
                "\n  FAIL edge[%d]: got t=%llu lv=%d, expected t=%llu lv=%d\n",
                i, (unsigned long long)g_edge_times[i], g_edge_levels[i],
                   (unsigned long long)et[i], el[i]);
            tzx_destroy(tp); exit(1);
        }
    }

    tzx_destroy(tp);
    printf("OK\n");
}

/* ─── test 2: data block timing ───────────────────────────────────────────── */
/*
 * Block: totp=0 (no pilot); totd=4 data symbols.
 * Data alphabet size=2: sym0={flags=1(no change),p0=855,p1=855},
 *                       sym1={flags=1,p0=1710,p1=1710}.
 * Data stream 0xA0=0b10100000 → sym1,sym0,sym1,sym0 (MSB-first, 1 bit each).
 * No pause.
 *
 * initial ear_level=1; flags=1 so no toggle at symbol start.
 * Edge sequence (each symbol yields 2 pulse toggles):
 *   sym1: t=1710 (lv=0), t=3420 (lv=1)
 *   sym0: t=4275 (lv=0), t=5130 (lv=1)
 *   sym1: t=6840 (lv=0), t=8550 (lv=1)
 *   sym0: t=9405 (lv=0), t=10260(lv=1)
 */
static void test_data_timing(void)
{
    printf("test_data_timing ... ");
    fflush(stdout);

    /* 2 symbols × (1 + npd*2) = 2 × 5 = 10 bytes */
    uint8_t data_sym[10];
    data_sym[0] = 0x01; put_le16(data_sym+1,  855); put_le16(data_sym+3,  855); /* sym0 */
    data_sym[5] = 0x01; put_le16(data_sym+6, 1710); put_le16(data_sym+8, 1710); /* sym1 */

    /* 4 symbols × 1 bit = 4 bits → 1 byte; 0b10100000 = sym1,sym0,sym1,sym0 */
    uint8_t data_stream[1] = { 0xA0 };

    uint8_t buf[256];
    size_t len = build_tzx_gen(buf, 0,
        0, 0, 0, NULL, NULL,
        4, 2, 2, data_sym, data_stream, 1);

    tzx_player_t* tp = tzx_create();
    assert(tp != NULL);
    tzx_set_ear_notify(tp, record_edge);
    g_edge_count = 0;
    assert(tzx_load_file(tp, write_tmp_tzx(buf, len)) == 0);

    uint64_t origin = 0;
    step(tp, &origin, 0);      /* init at t=0; no edges yet (flags=1) */
    assert(tzx_active(tp));

    step(tp, &origin, 15000);  /* process all 8 pulse edges */
    assert(!tzx_active(tp));

    assert(g_edge_count == 8);
    const uint64_t et[] = { 1710, 3420, 4275, 5130, 6840, 8550, 9405, 10260 };
    const int      el[] = {    0,    1,    0,    1,    0,    1,    0,      1 };
    for (int i = 0; i < 8; i++) {
        if (g_edge_times[i] != et[i] || g_edge_levels[i] != el[i]) {
            fprintf(stderr,
                "\n  FAIL edge[%d]: got t=%llu lv=%d, expected t=%llu lv=%d\n",
                i, (unsigned long long)g_edge_times[i], g_edge_levels[i],
                   (unsigned long long)et[i], el[i]);
            tzx_destroy(tp); exit(1);
        }
    }

    tzx_destroy(tp);
    printf("OK\n");
}

/* ─── test 3: pause anchored to last signal edge ─────────────────────────── */
/*
 * Block: 1 pilot stream entry, rep=2, toggle symbol, pulse=1000.  Pause=10ms.
 *
 * Signal ends at t=2000 (last pulse edge).
 * Pause of 10 ms = floor(10 * 3546900/1000) = 35469 T-states.
 * pause_end_at = 2000 + 35469 = 37469.
 *
 * After stepping to t=2001 the player must still be active (in pause).
 * After stepping to t=42001 (>37469) the player must be inactive.
 */
static void test_pause_after_block(void)
{
    printf("test_pause_after_block ... ");
    fflush(stdout);

    uint8_t pilot_sym[3];
    pilot_sym[0] = 0x00;
    put_le16(pilot_sym+1, 1000);

    uint8_t pilot_stream[3];
    pilot_stream[0] = 0;
    put_le16(pilot_stream+1, 2);   /* rep=2 */

    uint8_t buf[256];
    size_t len = build_tzx_gen(buf, 10,   /* 10 ms pause */
        1, 1, 1, pilot_sym, pilot_stream,
        0, 0, 0, NULL, NULL, 0);

    tzx_player_t* tp = tzx_create();
    assert(tp != NULL);
    tzx_set_ear_notify(tp, record_edge);
    g_edge_count = 0;
    assert(tzx_load_file(tp, write_tmp_tzx(buf, len)) == 0);

    uint64_t origin = 0;
    step(tp, &origin, 0);     /* init at t=0 */
    step(tp, &origin, 2001);  /* past last signal edge at t=2000 */
    assert(tzx_active(tp));   /* player is in the 10 ms pause → still active */

    step(tp, &origin, 40000); /* t_now = 2001+40000 = 42001 > pause_end=37469 */
    assert(!tzx_active(tp));  /* pause expired */

    tzx_destroy(tp);
    printf("OK\n");
}

/* ─── test 4: pilot then data ────────────────────────────────────────────── */
/*
 * Block: pilot=2 reps of toggle sym (pulse=500); data=2 symbols
 *        sym0={flags=1,pulse=300}, sym1={flags=1,pulse=300};
 *        stream 0x80→sym1,sym0.  No pause.
 *
 * Edges (initial ear=1):
 *   t=   0 lv=0  (pilot rep1 flags toggle)
 *   t= 500 lv=1  (pilot rep1 pulse end)
 *   t= 500 lv=0  (pilot rep2 flags toggle — chained from rep1 end)
 *   t=1000 lv=1  (pilot rep2 pulse end)
 *   t=1300 lv=0  (data sym1 pulse end; flags=1 so no start toggle)
 *   t=1600 lv=1  (data sym0 pulse end)
 */
static void test_pilot_then_data(void)
{
    printf("test_pilot_then_data ... ");
    fflush(stdout);

    uint8_t pilot_sym[3];
    pilot_sym[0] = 0x00;
    put_le16(pilot_sym+1, 500);

    uint8_t pilot_stream[3];
    pilot_stream[0] = 0;
    put_le16(pilot_stream+1, 2);

    /* 2 symbols × (1 + npd*2) = 2 × 3 = 6 bytes */
    uint8_t data_sym[6];
    data_sym[0] = 0x01; put_le16(data_sym+1, 300);   /* sym0 */
    data_sym[3] = 0x01; put_le16(data_sym+4, 300);   /* sym1 */

    /* totd=2, bits=1, stream=0b10000000 → sym1 then sym0 */
    uint8_t data_stream[1] = { 0x80 };

    uint8_t buf[256];
    size_t len = build_tzx_gen(buf, 0,
        1, 1, 1, pilot_sym, pilot_stream,
        2, 1, 2, data_sym, data_stream, 1);

    tzx_player_t* tp = tzx_create();
    assert(tp != NULL);
    tzx_set_ear_notify(tp, record_edge);
    g_edge_count = 0;
    assert(tzx_load_file(tp, write_tmp_tzx(buf, len)) == 0);

    uint64_t origin = 0;
    step(tp, &origin, 0);
    step(tp, &origin, 5000);
    assert(!tzx_active(tp));

    assert(g_edge_count == 6);
    const uint64_t et[] = {    0,  500,  500, 1000, 1300, 1600 };
    const int      el[] = {    0,    1,    0,    1,    0,    1  };
    for (int i = 0; i < 6; i++) {
        if (g_edge_times[i] != et[i] || g_edge_levels[i] != el[i]) {
            fprintf(stderr,
                "\n  FAIL edge[%d]: got t=%llu lv=%d, expected t=%llu lv=%d\n",
                i, (unsigned long long)g_edge_times[i], g_edge_levels[i],
                   (unsigned long long)et[i], el[i]);
            tzx_destroy(tp); exit(1);
        }
    }

    tzx_destroy(tp);
    printf("OK\n");
}

/* ─── test 5: incremental 1-T-state stepping ─────────────────────────────── */
/*
 * Same pilot as test 1 (4 reps, pulse=2168) but advanced one T-state at a
 * time.  Must produce the same 8 edges at the same timestamps, proving that
 * the symbol-chaining logic is drift-free regardless of step granularity.
 */
static void test_incremental_step(void)
{
    printf("test_incremental_step ... ");
    fflush(stdout);

    uint8_t pilot_sym[3];
    pilot_sym[0] = 0x00;
    put_le16(pilot_sym+1, 2168);

    uint8_t pilot_stream[3];
    pilot_stream[0] = 0;
    put_le16(pilot_stream+1, 4);

    uint8_t buf[256];
    size_t len = build_tzx_gen(buf, 0,
        1, 1, 1, pilot_sym, pilot_stream,
        0, 0, 0, NULL, NULL, 0);

    tzx_player_t* tp = tzx_create();
    assert(tp != NULL);
    tzx_set_ear_notify(tp, record_edge);
    g_edge_count = 0;
    assert(tzx_load_file(tp, write_tmp_tzx(buf, len)) == 0);

    uint64_t origin = 0;

    /* Init at t=0 to anchor the block; fires the first toggle at t=0. */
    step(tp, &origin, 0);

    /* Step 1 T-state at a time; all edges land at or before t=8672. */
    for (int i = 0; i < 20000 && tzx_active(tp); i++)
        step(tp, &origin, 1);

    assert(!tzx_active(tp));
    assert(g_edge_count == 8);

    const uint64_t et[] = { 0, 2168, 2168, 4336, 4336, 6504, 6504, 8672 };
    const int      el[] = { 0,    1,    0,    1,    0,    1,    0,    1  };
    for (int i = 0; i < 8; i++) {
        if (g_edge_times[i] != et[i] || g_edge_levels[i] != el[i]) {
            fprintf(stderr,
                "\n  FAIL edge[%d]: got t=%llu lv=%d, expected t=%llu lv=%d\n",
                i, (unsigned long long)g_edge_times[i], g_edge_levels[i],
                   (unsigned long long)et[i], el[i]);
            tzx_destroy(tp); exit(1);
        }
    }

    tzx_destroy(tp);
    printf("OK\n");
}

/* ─── test 6: tzx_ear_bit6 reflects current EAR level ────────────────────── */
/*
 * Single pilot symbol: flags=0 (toggle), pulse=1000, rep=1.  No pause.
 *
 * Before any step:       active, ear=1 → bit6=0x40
 * After step(0):         first toggle at t=0 → ear=0 → bit6=0x00
 * After step(1100):      pulse edge at t=1000 fires → ear=1, player done
 *                        tzx_ear_bit6 returns 0x00 when player is not active
 */
static void test_ear_bit6(void)
{
    printf("test_ear_bit6 ... ");
    fflush(stdout);

    uint8_t pilot_sym[3];
    pilot_sym[0] = 0x00;
    put_le16(pilot_sym+1, 1000);

    uint8_t pilot_stream[3];
    pilot_stream[0] = 0;
    put_le16(pilot_stream+1, 1);   /* rep=1 */

    uint8_t buf[256];
    size_t len = build_tzx_gen(buf, 0,
        1, 1, 1, pilot_sym, pilot_stream,
        0, 0, 0, NULL, NULL, 0);

    tzx_player_t* tp = tzx_create();
    assert(tp != NULL);
    tzx_set_ear_notify(tp, record_edge);
    g_edge_count = 0;
    assert(tzx_load_file(tp, write_tmp_tzx(buf, len)) == 0);

    /* Before any advancement: player is active, ear_level=1. */
    assert(tzx_ear_bit6(tp) == 0x40);

    uint64_t origin = 0;
    step(tp, &origin, 0);       /* flags toggle at t=0 → ear=0 */
    assert(tzx_active(tp));
    assert(tzx_ear_bit6(tp) == 0x00);

    step(tp, &origin, 1100);    /* pulse edge at t=1000 → ear=1, then player done */
    assert(!tzx_active(tp));
    assert(tzx_ear_bit6(tp) == 0x00);  /* 0 when inactive, regardless of ear_level */

    tzx_destroy(tp);
    printf("OK\n");
}

/* ─── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    printf("=== TZX 0x19 Generalised Data tests ===\n");
    test_pilot_only();
    test_data_timing();
    test_pause_after_block();
    test_pilot_then_data();
    test_incremental_step();
    test_ear_bit6();
    printf("All tests passed.\n");
    return 0;
}
