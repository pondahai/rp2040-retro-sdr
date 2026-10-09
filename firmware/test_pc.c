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

#include "ddc.h"
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

    printf("[1b] real-FFT split: no mirror image\n");
    {
        /* 拆分寫錯的典型症狀：在 M−k 冒出一個鏡像。挑高頻的 bin 1500，
         * 鏡像會落在 548，兩邊都看。 */
        int b = 1500, img = FFT_M - b;
        tone t = { (double)b * SP_FS / SP_N, 2047.0 };
        sdr_init(s);
        synth(2048.0, 0.0, &t, 1);
        sdr_block(s, g_block, SDR_BLOCK, SDR_SKIP);
        int v = s->sp.bin_db[b], m = s->sp.bin_db[img];
        CHECK(v >= -3 && v <= 3, "sine at bin %d reads %d.%d dBFS (want 0 +-0.3)",
              b, v / 10, abs(v % 10));
        CHECK(m < -680, "mirror bin %d reads %d.%d dBFS (want < -68)",
              img, m / 10, abs(m % 10));
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
        CHECK(s->tune_hz == 68500 && s->cursor == 87, "starts on BPC (tune %ld, cursor %d)",
              (long)s->tune_hz, s->cursor);
        s->ddc_dirty = 0;
        type(s, "250");
        press(s, KEY_ENTER);
        CHECK(s->tune_hz == 68500 && !s->ddc_dirty,
              "out-of-range entry is ignored (tune %ld)", (long)s->tune_hz);
        type(s, "1.2.3");
        press(s, KEY_ENTER);
        CHECK(s->tune_hz == 68500, "malformed entry is ignored (tune %ld)", (long)s->tune_hz);
        type(s, "200");
        press(s, KEY_ESC);
        CHECK(!s->entering && s->tune_hz == 68500, "ESC cancels entry");
        type(s, "40.0125");
        press(s, KEY_ENTER);
        CHECK(s->tune_hz == 68500, "more than 3 decimals is rejected (tune %ld)",
              (long)s->tune_hz);
        type(s, "40.012");
        press(s, KEY_ENTER);
        CHECK(s->tune_hz == 40012 && s->ddc_dirty, "40.012 -> 40012 Hz (got %ld)",
              (long)s->tune_hz);
        press(s, 'L');
        CHECK(s->tune_hz == 41012, "L tunes +10 steps (got %ld)", (long)s->tune_hz);
        press(s, KEY_LEFT);
        CHECK(s->tune_hz == 40912, "LEFT tunes -1 step (got %ld)", (long)s->tune_hz);
        press(s, 's');
        CHECK(s->step_hz == 1000 && s->tune_hz == 41000,
              "S cycles step to 1000 and snaps (step %d, tune %ld)", s->step_hz,
              (long)s->tune_hz);
        for (int i = 0; i < 100; i++)
            press(s, KEY_LEFT);
        CHECK(s->tune_hz == 0 && s->cursor == 0, "tuning clamps at 0 Hz");
        press(s, 'm');
        CHECK(s->mode == DDC_USB && s->bw_hz == 2400, "M cycles CW -> USB with its default BW");
        press(s, 'b');
        CHECK(s->bw_hz == 2700, "B cycles bandwidth (got %d)", s->bw_hz);
        for (int i = 0; i < 20; i++)
            press(s, KEY_PGUP);
        CHECK(s->vol == SDR_VOL_MAX, "volume caps at %d", SDR_VOL_MAX);
        for (int i = 0; i < 40; i++)
            press(s, KEY_DOWN);
        CHECK(s->ref_db == -140, "REF floors at -140 (got %d)", s->ref_db);
        for (int i = 0; i < 10; i++)
            press(s, ']');
        CHECK(s->range_db == 120, "RANGE caps at 120 (got %d)", s->range_db);
    }

    printf("[5] DDC / demodulation\n");
    {
        static ddc d;
        static int16_t audio[1024];
        struct {
            const char *name;
            int mode;
            tone t[3];
            int nt;
            double want_hz;       /* 音訊裡應該出現的頻率 */
            double image_hz;      /* 反邊帶若沒濾掉會出現在這裡（0 = 不檢查） */
        } cases[] = {
            { "CW  carrier at tune      -> 800 Hz", DDC_CW,
              { { 68500, 20 } }, 1, 800, 0 },
            { "USB tone at tune+1000    -> 1000 Hz", DDC_USB,
              { { 69500, 20 } }, 1, 1000, 0 },
            { "USB rejects tune-1000 (LSB side)", DDC_USB,
              { { 67500, 20 }, { 69700, 2 } }, 2, 1200, 1000 },
            { "LSB tone at tune-1000    -> 1000 Hz", DDC_LSB,
              { { 67500, 20 } }, 1, 1000, 0 },
            { "AM  carrier + 1 kHz, m=0.5 -> 1000 Hz", DDC_AM,
              { { 68500, 40 }, { 67500, 10 }, { 69500, 10 } }, 3, 1000, 0 },
        };
        const double afs = DDC_AFS_X2 / 2.0;
        for (unsigned c = 0; c < sizeof cases / sizeof cases[0]; c++) {
            ddc_init(&d);
            ddc_set(&d, 68500, cases[c].mode, ddc_default_bw(cases[c].mode));
            g_t = 0;
            int got = 0;
            for (int blk = 0; blk < 6; blk++) {   /* 前幾塊讓 FIR、AGC 穩下來 */
                synth(508.0, 2.0, cases[c].t, cases[c].nt);
                got = ddc_block(&d, g_block, SDR_BLOCK, 0, 0, audio, 1024);
            }
            /* Goertzel：目標頻率的功率 vs 300..3500 Hz 其他地方的最大值 */
            double pw_want = 0, pw_other = 0, pw_img = 0;
            for (int f = 300; f <= 3500; f += 50) {
                double w = 2 * 3.14159265358979323846 * f / afs, cw = 2 * cos(w);
                double s0 = 0, s1 = 0, s2 = 0;
                for (int i = 0; i < got; i++) {
                    s0 = audio[i] + cw * s1 - s2;
                    s2 = s1; s1 = s0;
                }
                double pw = s1 * s1 + s2 * s2 - cw * s1 * s2;
                if (fabs(f - cases[c].want_hz) < 1)
                    pw_want = pw;
                else if (cases[c].image_hz && fabs(f - cases[c].image_hz) < 1)
                    pw_img = pw;
                else if (fabs(f - cases[c].want_hz) > 150 && pw > pw_other)
                    pw_other = pw;
            }
            double ratio = 10 * log10((pw_want + 1e-9) / (pw_other + 1e-9));
            CHECK(got == SDR_BLOCK / DDC_DECIM && ratio > 20,
                  "%s: %d samples, %.1f dB above anything else", cases[c].name, got, ratio);
            if (cases[c].image_hz) {
                double rej = 10 * log10((pw_want + 1e-9) / (pw_img + 1e-9));
                /* 想要的那個只有 2 LSB、不要的有 20 LSB（強 20 dB）：
                 * 濾掉之後想要的仍比較大，才算真的拒斥了 */
                CHECK(rej > 10, "    image at %.0f Hz is %.1f dB below the wanted tone",
                      cases[c].image_hz, rej);
            }
        }
    }

    printf("[6] DDC across block boundaries (scan gap + bias transient)\n");
    {
        /* 上機實況：塊與塊之間空了約 1 ms（鍵盤掃描），每塊開頭偏壓從 0 V
         * 爬回 0.41 V（τ ≈ 41 µs ≈ 20 點）。把好幾塊的音訊接起來看 CW 嗶聲
         * 乾不乾淨。gap 給對 vs 給 0 各跑一次：給 0 應該明顯比較髒。 */
        static ddc d;
        static int16_t audio[4096];
        const int GAP = 480;
        tone t = { 68500, 20 };
        const double afs = DDC_AFS_X2 / 2.0;
        double purity[2];
        for (int pass = 0; pass < 2; pass++) {
            ddc_init(&d);
            ddc_set(&d, 68500, DDC_CW, 500);
            g_t = 0;
            int got = 0;
            for (int blk = 0; blk < 10; blk++) {
                g_t += (double)GAP / SP_FS;                  /* 空檔：真實時間照走 */
                synth(508.0, 2.0, &t, 1);
                for (int i = 0; i < SDR_SKIP; i++)          /* 偏壓回穩的暫態 */
                    g_block[i] = (uint16_t)lround(g_block[i] - 508.0 * exp(-i / 20.5));
                int n = ddc_block(&d, g_block, SDR_BLOCK, SDR_SKIP,
                                  pass == 0 ? GAP : 0, audio + (blk >= 4 ? got : 0),
                                  (int)(sizeof audio / sizeof audio[0]) - got);
                if (blk >= 4)
                    got += n;
            }
            double pw_want = 0, pw_other = 0;
            for (int f = 300; f <= 3500; f += 25) {
                double w = 2 * 3.14159265358979323846 * f / afs, cw = 2 * cos(w);
                double s0 = 0, s1 = 0, s2 = 0;
                for (int i = 0; i < got; i++) {
                    s0 = audio[i] + cw * s1 - s2;
                    s2 = s1; s1 = s0;
                }
                double pw = s1 * s1 + s2 * s2 - cw * s1 * s2;
                if (f == DDC_CW_PITCH)
                    pw_want = pw;
                else if (abs(f - DDC_CW_PITCH) > 100 && pw > pw_other)
                    pw_other = pw;
            }
            purity[pass] = 10 * log10((pw_want + 1e-9) / (pw_other + 1e-9));
        }
        CHECK(purity[0] > 25, "6 blocks stitched, gap compensated: tone %.1f dB above junk",
              purity[0]);
        printf("        (same without gap compensation: %.1f dB)\n", purity[1]);
        CHECK(purity[0] > purity[1] + 3, "gap compensation makes a difference");
    }

    write_glyphs("glyphs.ppm");
    printf("        wrote glyphs.ppm\n");

    printf("\n%s (%d failed)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail ? 1 : 0;
}
