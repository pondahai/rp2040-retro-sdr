/* PC 上跑整條路徑：合成 ADC 樣本 -> sdr_block -> sdr_key -> ui_line。
 *
 *   build_pc.bat && test_pc.exe
 *
 * 會檢查：
 *   1. dBFS 校正：滿刻度正弦讀到 0 dBFS
 *   2. 雜訊底線：純雜訊的中位數接近理論值
 *   3. 實景：0.41 V 偏壓上疊雜訊、BPC 載波、JJY 載波、PAM8403 的尖峰，
 *      打 "68.5" Enter 之後游標落在 BPC 上，且看得出載波高出雜訊
 *   4. 頻率輸入的解析
 * 並輸出 screen.ppm（整個畫面）與 glyphs.ppm（字表），給人眼看。
 * 有任何一項不過就回傳非 0。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "font5x7.h"
#include "sdr.h"

static int g_fail;

#define CHECK(cond, ...)                                 \
    do {                                                 \
        if (cond) {                                      \
            printf("  ok    ");                          \
        } else {                                         \
            printf("  FAIL  ");                          \
            g_fail++;                                    \
        }                                                \
        printf(__VA_ARGS__);                             \
        printf("\n");                                    \
    } while (0)

/* ---- 合成訊號 ------------------------------------------------------------ */

static uint64_t g_rng = 0x9E3779B97F4A7C15ull;

static double urand(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 7;
    g_rng ^= g_rng << 17;
    return ((g_rng >> 11) + 0.5) / 9007199254740992.0;
}

static double grand(void)
{
    return sqrt(-2.0 * log(urand())) * cos(2.0 * 3.14159265358979323846 * urand());
}

typedef struct { double hz, amp; } tone;

static uint16_t g_block[SDR_BLOCK];
static double g_t;                    /* 跨塊連續的時間，載波相位才不會每塊重來 */

static void synth(double dc, double sigma, const tone *t, int nt)
{
    const double PI2 = 2.0 * 3.14159265358979323846;
    for (int i = 0; i < SDR_BLOCK; i++, g_t += 1.0 / SP_FS) {
        double v = dc + sigma * grand();
        for (int k = 0; k < nt; k++)
            v += t[k].amp * sin(PI2 * t[k].hz * g_t);
        long q = lround(v);
        if (q < 0) q = 0;               /* ADC 量不到負電壓 */
        if (q > 4095) q = 4095;
        g_block[i] = (uint16_t)q;
    }
}

/* ---- 輸出圖檔 ------------------------------------------------------------ */

static void write_screen(const sdr *s, const char *path)
{
    static uint16_t line[UI_W];
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); exit(1); }
    fprintf(f, "P6\n%d %d\n255\n", UI_W, UI_H);
    for (int y = 0; y < UI_H; y++) {
        ui_line(s, y, line);
        for (int x = 0; x < UI_W; x++) {
            uint16_t c = (uint16_t)((line[x] >> 8) | (line[x] << 8));  /* 翻回原生 */
            uint8_t rgb[3] = {
                (uint8_t)(((c >> 11) & 31) * 255 / 31),
                (uint8_t)(((c >> 5) & 63) * 255 / 63),
                (uint8_t)((c & 31) * 255 / 31),
            };
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
}

static void write_glyphs(const char *path)
{
    enum { COLS = 16, ROWS = 6, SC = 4 };
    int w = COLS * FONT_CW * SC, h = ROWS * FONT_CH * SC;
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); exit(1); }
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int cx = x / SC / FONT_CW, cy = y / SC / FONT_CH;
            int gx = x / SC % FONT_CW, gy = y / SC % FONT_CH;
            char c = (char)(0x20 + cy * COLS + cx);
            int on = gx < 5 && gy < 7 && (font5x7_row(c, gy) & (0x10 >> gx));
            uint8_t v = on ? 255 : ((cx + cy) & 1 ? 40 : 20);
            uint8_t rgb[3] = { v, v, v };
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
}

static void press(sdr *s, uint8_t code)
{
    key_event e = { code, 0, 0 };
    sdr_key(s, &e);
}

static void type(sdr *s, const char *str)
{
    for (; *str; str++)
        press(s, (uint8_t)*str);
}

/* ---- 測試 ---------------------------------------------------------------- */

static sdr g_s;                       /* 140 KB 左右，別放在堆疊上 */

int main(void)
{
    sdr *s = &g_s;

    printf("[1] dBFS calibration\n");
    {
        /* 68.5 kHz 不在 bin 正中央；挑 bin 561 正中央的頻率，才量得到相干增益 */
        int b = 561;
        tone t = { (double)b * SP_FS / SP_N, 2047.0 };
        sdr_init(s);
        synth(2048.0, 0.0, &t, 1);
        sdr_block(s, g_block, SDR_BLOCK, SDR_SKIP);
        int v = s->sp.bin_db[b];
        CHECK(v >= -3 && v <= 3, "full-scale sine at bin %d reads %d.%d dBFS (want 0 +-0.3)",
              b, v / 10, abs(v % 10));
        /* 正弦以外的地方應該很乾淨：只有 Hann 主瓣（±2 bin）會高 */
        int worst = -1600, wb = 0;
        for (int i = 4; i < SP_BINS; i++)
            if ((i < b - 3 || i > b + 3) && s->sp.bin_db[i] > worst)
                worst = s->sp.bin_db[i], wb = i;
        CHECK(worst < -680, "worst spur away from the tone: %d.%d dBFS at bin %d (want < -68)",
              worst / 10, abs(worst % 10), wb);
    }

    printf("[2] noise floor\n");
    {
        double sigma = 2.0;
        sdr_init(s);
        for (int i = 0; i < 4; i++) {
            synth(508.0, sigma, NULL, 0);
            sdr_block(s, g_block, SDR_BLOCK, SDR_SKIP);
        }
        /* 每 bin 的雜訊功率相對滿刻度正弦：6σ²/(A²N)，σ² 含量化雜訊 1/12。
         * dB 域平均 K 段再取中位數，比線性平均低約 1.5–2.5 dB。 */
        double var = sigma * sigma + 1.0 / 12.0;
        double theory = 10.0 * log10(6.0 * var / (2048.0 * 2048.0 * SP_N));
        double got = s->sp.nf / 10.0;
        CHECK(got > theory - 4.0 && got < theory + 1.0,
              "NF %.1f dBFS/bin, theory %.1f (linear mean), want within -4..+1", got, theory);
    }

    printf("[3] realistic scene + frequency entry\n");
    {
        tone t[] = {
            { 68500.0, 3.0 },         /* BPC：只有 3 LSB */
            { 40000.0, 20.0 },        /* JJY 40k：強一點 */
            { 240000.0, 10.0 },       /* PAM8403 摺疊回來的尖峰（假設的位置） */
        };
        sdr_init(s);
        for (int i = 0; i < 40; i++) {
            synth(508.0, 2.0, t, 3);
            sdr_block(s, g_block, SDR_BLOCK, SDR_SKIP);
        }
        type(s, "68.5");
        CHECK(s->entering, "typing digits enters frequency mode");
        sdr_prepare(s);               /* 輸入中的畫面也要畫得出來 */
        press(s, KEY_ENTER);
        CHECK(!s->entering && s->cursor == 87, "after ENTER cursor = %d (want 87)", s->cursor);

        int cb = sdr_cursor_bin(s);
        int32_t hz = spectrum_bin_hz(cb);
        CHECK(labs(hz - 68500) <= 250, "cursor peak bin %d = %ld Hz (want 68500 +-250)",
              cb, (long)hz);
        int snr = s->sp.bin_db[cb] - s->sp.nf;
        CHECK(snr >= 100, "BPC (3 LSB) stands %d.%d dB above NF (want >= 10)",
              snr / 10, snr % 10);

        int jb = (int)lround(40000.0 * SP_N / SP_FS);
        int jsnr = s->sp.bin_db[jb] - s->sp.nf;
        CHECK(jsnr >= 250, "JJY 40k (20 LSB) stands %d.%d dB above NF (want >= 25)",
              jsnr / 10, jsnr % 10);

        s->proc_ms = 31;
        s->scan_us = 1210;
        sdr_prepare(s);
        write_screen(s, "screen.ppm");
        printf("        wrote screen.ppm\n");
    }

    printf("[4] keys\n");
    {
        sdr_init(s);
        type(s, "250");
        press(s, KEY_ENTER);
        CHECK(s->cursor == 88, "out-of-range entry leaves cursor alone (cursor %d, want 88)",
              s->cursor);
        type(s, "1.2.3");
        press(s, KEY_ENTER);
        CHECK(s->cursor == 88, "malformed entry is ignored (cursor %d)", s->cursor);
        type(s, "200");
        press(s, KEY_ESC);
        CHECK(!s->entering && s->cursor == 88, "ESC cancels entry");
        press(s, 'L');
        CHECK(s->cursor == 98, "L moves +10 (cursor %d)", s->cursor);
        press(s, KEY_LEFT);
        CHECK(s->cursor == 97, "LEFT moves -1 (cursor %d)", s->cursor);
        for (int i = 0; i < 40; i++)
            press(s, KEY_DOWN);
        CHECK(s->ref_db == -140, "REF floors at -140 (got %d)", s->ref_db);
        for (int i = 0; i < 10; i++)
            press(s, KEY_PGUP);
        CHECK(s->range_db == 120, "RANGE caps at 120 (got %d)", s->range_db);
    }

    write_glyphs("glyphs.ppm");
    printf("        wrote glyphs.ppm\n");

    printf("\n%s (%d failed)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail ? 1 : 0;
}
