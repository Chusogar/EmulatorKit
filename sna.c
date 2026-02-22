/*
 * sna.c – SNA snapshot loader (48K & 128K) for ZX Spectrum emulator.
 *
 * 48K:  PC is taken from the stack (implicit RETN) and SP += 2.
 * 128K: PC is explicit after the 48K header; 7FFD and remaining banks follow.
 *
 * SNA format refs:
 *   https://sinclair.wiki.zxnet.co.uk/wiki/SNA_format
 *   https://worldofspectrum.net/zx-modules/fileformats/snaformat.html
 */

#include "sna.h"

#include <stdio.h>
#include <string.h>

/* RAM bank index helpers (must match spectrum.c layout) */
#define SNA_ROM(x)  (x)
#define SNA_RAM(x)  ((x) + 8)

bool load_sna(const char *filename, const sna_context_t *ctx)
{
    FILE *f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "No se pudo abrir .sna: %s\n", filename);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t header[27];
    if (fread(header, 1, 27, f) != 27) {
        fclose(f);
        fprintf(stderr, "Archivo .sna incompleto (header)\n");
        return false;
    }

    Z80Context *cpu = ctx->cpu;

    /* Registers (48K layout) */
    cpu->I          = header[0];
    cpu->R2.wr.HL   = (header[2]  << 8) | header[1];
    cpu->R2.wr.DE   = (header[4]  << 8) | header[3];
    cpu->R2.wr.BC   = (header[6]  << 8) | header[5];
    cpu->R2.wr.AF   = (header[8]  << 8) | header[7];
    cpu->R1.wr.HL   = (header[10] << 8) | header[9];
    cpu->R1.wr.DE   = (header[12] << 8) | header[11];
    cpu->R1.wr.BC   = (header[14] << 8) | header[13];
    cpu->R1.wr.IY   = (header[16] << 8) | header[15];
    cpu->R1.wr.IX   = (header[18] << 8) | header[17];
    /* Byte 19: docs clásicos indican IFF2; usamos bit2 o cualquier no-cero como habilitado */
    cpu->IFF2       = (header[19] & 0x04) ? 1 : (header[19] ? 1 : 0);
    cpu->R          = header[20];
    cpu->R1.wr.AF   = (header[22] << 8) | header[21];
    cpu->R1.wr.SP   = (header[24] << 8) | header[23];
    cpu->IM         = header[25];
    *ctx->border_color = header[26] & 0x07;

    /* RAM 48K */
    uint8_t ram48k[49152];
    if (fread(ram48k, 1, sizeof(ram48k), f) != sizeof(ram48k)) {
        fclose(f);
        fprintf(stderr, "Archivo .sna incompleto (RAM 48K)\n");
        return false;
    }

    bool is128k = (fsz == 131103L || fsz == 147487L);

    if (!is128k) {
        /* 48K pure → copy to 0x4000..0xFFFF */
        for (uint32_t off = 0; off < sizeof(ram48k); ++off)
            ctx->mem_write(0, 0x4000u + off, ram48k[off]);

        /* PC from stack; SP += 2 */
        uint16_t sp = cpu->R1.wr.SP;
        uint8_t pcl = ctx->mem_read(0, sp);
        uint8_t pch = ctx->mem_read(0, sp + 1);
        cpu->PC = ((uint16_t)pch << 8) | pcl;
        cpu->R1.wr.SP = sp + 2;

        cpu->IFF1 = cpu->IFF2;

        fclose(f);
        printf("Snapshot .sna (48K) cargado: %s\n", filename);
        printf("PC=0x%04X  SP=0x%04X  Border=%d  IM=%d\n",
               cpu->PC, cpu->R1.wr.SP, *ctx->border_color, cpu->IM);
        return true;
    }

    /* 128K */
    int pcl = fgetc(f), pch = fgetc(f);
    if (pcl == EOF || pch == EOF) { fclose(f); fprintf(stderr, "SNA 128K: error leyendo PC\n"); return false; }
    cpu->PC = (uint16_t)(pcl | (pch << 8));

    int last7ffd = fgetc(f);
    int trdos    = fgetc(f);
    if (last7ffd == EOF || trdos == EOF) { fclose(f); fprintf(stderr, "SNA 128K: error leyendo 7FFD/TRDOS\n"); return false; }
    uint8_t latch7ffd = (uint8_t)last7ffd;
    uint8_t bank_n = latch7ffd & 0x07;

    /* Initial 48K RAM → banks 5 / n / 2 */
    memcpy(&ctx->ram[SNA_RAM(5)][0],      &ram48k[0x0000], 16384);
    memcpy(&ctx->ram[SNA_RAM(bank_n)][0], &ram48k[0x4000], 16384);
    memcpy(&ctx->ram[SNA_RAM(2)][0],      &ram48k[0x8000], 16384);

    /* Fill remaining banks (5 or 6) in file order */
    int remain[8] = {0,1,2,3,4,5,6,7};
    remain[5] = -1; remain[2] = -1; remain[bank_n] = -1;

    for (int b = 0; b < 8; ++b) {
        if (remain[b] >= 0) {
            if (fread(&ctx->ram[SNA_RAM(remain[b])][0], 1, 16384, f) != 16384) {
                fclose(f);
                fprintf(stderr, "SNA 128K: error leyendo banco %d\n", remain[b]);
                return false;
            }
        }
    }

    /* Update pagination latch / MMU */
    *ctx->mlatch = latch7ffd & 0x3F;
    ctx->recalc_mmu();

    /* Sync visible window with snapshot */
    for (uint32_t off = 0; off < 16384; ++off)
        ctx->mem_write(0, 0x4000u + off, ctx->ram[SNA_RAM(5)][off]);
    for (uint32_t off = 0; off < 16384; ++off)
        ctx->mem_write(0, 0x8000u + off, ctx->ram[SNA_RAM(2)][off]);
    for (uint32_t off = 0; off < 16384; ++off)
        ctx->mem_write(0, 0xC000u + off, ctx->ram[SNA_RAM(bank_n)][off]);

    cpu->IFF1 = cpu->IFF2;

    fclose(f);
    printf("Snapshot .sna (128K) cargado: %s  (7FFD=0x%02X, n=%u, TR-DOS=%u)\n",
           filename, latch7ffd, bank_n, trdos);
    printf("PC=0x%04X  SP=0x%04X  Border=%d  IM=%d\n",
           cpu->PC, cpu->R1.wr.SP, *ctx->border_color, cpu->IM);

    return true;
}