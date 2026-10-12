/* rec_play.c — 把 M4 的 RECnnn.WAV（500 ksps 原始樣本）用機上同一份 ddc.c 解調成音訊。
 *
 *   rec_play REC003.WAV out.wav [bw_hz] [eq] [nogap]
 *
 * 切塊、skip、塊間空檔都照機上的做（空檔從同名 .TXT 的 start_us 算），
 * 輸出 15625 Hz 單聲道 16 位元。eq 預設 2（SPK+，開機預設）；nogap 把空檔當 0，
 * 拿來對照塊邊界的處理。
 *
 * 編譯（repo 根目錄）：
 *   gcc -std=c11 -O2 -Ifirmware -o tools/rec_play.exe tools/rec_play.c firmware/ddc.c firmware/eq.c -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ddc.h"
#include "eq.h"

#define BLOCK 32768
#define SKIP  512


static void put32(FILE *f, uint32_t v) { fwrite(&v, 4, 1, f); }
static void put16(FILE *f, uint16_t v) { fwrite(&v, 2, 1, f); }

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: rec_play REC.WAV out.wav [bw] [eq] [nogap]\n");
        return 1;
    }
    char txt[512];
    snprintf(txt, sizeof txt, "%s", argv[1]);
    strcpy(txt + strlen(txt) - 3, "TXT");

    long tune = 0;
    char mode[8] = "AM";
    static uint32_t start[4096];
    int nstart = 0;
    FILE *ft = fopen(txt, "r");
    if (ft) {
        char line[128];
        int in_list = 0;
        while (fgets(line, sizeof line, ft)) {
            if (in_list) { if (nstart < 4096) start[nstart++] = strtoul(line, 0, 10); }
            else if (!strncmp(line, "tune ", 5)) tune = atol(line + 5);
            else if (!strncmp(line, "mode ", 5)) sscanf(line + 5, "%7s", mode);
            else if (!strncmp(line, "start_us", 8)) in_list = 1;
        }
        fclose(ft);
    }
    int m = DDC_AM;
    for (int i = 0; i < DDC_NMODES; i++)
        if (!strcmp(mode, ddc_mode_name(i))) m = i;
    int bw = argc > 3 ? atoi(argv[3]) : 0;
    if (bw <= 0) bw = ddc_default_bw(m);
    int eqp = argc > 4 ? atoi(argv[4]) : 2;
    int nogap = argc > 5 && !strcmp(argv[5], "nogap");

    FILE *fi = fopen(argv[1], "rb");
    if (!fi) { perror(argv[1]); return 1; }
    fseek(fi, 44, SEEK_SET);
    FILE *fo = fopen(argv[2], "wb");
    if (!fo) { perror(argv[2]); return 1; }
    fwrite("RIFF\0\0\0\0WAVEfmt ", 16, 1, fo);
    put32(fo, 16); put16(fo, 1); put16(fo, 1);
    put32(fo, DDC_AFS_X2 / 2); put32(fo, DDC_AFS_X2); put16(fo, 2); put16(fo, 16);
    fwrite("data\0\0\0\0", 8, 1, fo);

    ddc d;
    ddc_init(&d);
    ddc_set(&d, (int32_t)tune, m, bw);
    eq e;
    eq_set(&e, eqp, DDC_AFS_X2 / 2.0);

    static uint16_t x[BLOCK];
    static int16_t out[BLOCK / DDC_DECIM + DDC_FILL_MAX];
    uint32_t total = 0;
    int gmin = 1 << 30, gmax = 0;
    for (int b = 0; fread(x, 2, BLOCK, fi) == BLOCK; b++) {
        int gap = 0;
        if (b > 0 && b < nstart) {
            /* 上一塊結束到這一塊 x[0]：兩次開始相隔（µs÷2 = 點數）減掉一塊 */
            gap = (int)((start[b] - start[b - 1] + 1) / 2) - BLOCK;
            if (gap < 0) gap = 0;
            if (gap < gmin) gmin = gap;
            if (gap > gmax) gmax = gap;
        }
        if (nogap) gap = 0;
        int n = ddc_block(&d, x, BLOCK, SKIP, gap, out, (int)(sizeof out / sizeof out[0]));
        eq_run(&e, out, n);
        if (getenv("REC_BOUNDS")) fprintf(stderr, "%u\n", total);
        fwrite(out, 2, n, fo);
        total += n;
    }
    fseek(fo, 4, SEEK_SET); put32(fo, 36 + total * 2);
    fseek(fo, 40, SEEK_SET); put32(fo, total * 2);
    fclose(fo);
    fclose(fi);
    printf("%s: tune %ld %s bw %d eq %d%s, %u samples (%.1f s), gap %d..%d pts\n",
           argv[1], tune, mode, bw, eqp, nogap ? " nogap" : "", total,
           total / (DDC_AFS_X2 / 2.0), gmin, gmax);
    return 0;
}
