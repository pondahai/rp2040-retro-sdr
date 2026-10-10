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
#include "jjy.h"
#include "sdr.h"
#include "preset.h"
#include "wav.h"

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

        /* 統計頁（鍵盤 I）：內容是平台填的，這裡放一份上機時的樣子 */
        static const char *demo[UI_STAT_LINES] = {
            "STATS                          uptime 00:12:34",
            "CORE0  proc 62 ms = scan 1.0 + dsp 37 + draw 24",
            "       fft 25 ms  db 8 ms   blocks 11234  drop 0",
            "CORE1  ddc 31 ms of 65  (fir 15  cic 16)",
            "AUDIO  fill 726  underruns 828  vol 5",
            "CLOCK  sys 250 MHz  peri 250 MHz  spi 62.5 MHz",
            "SIGNAL NF -97.0 dBFS  tune -88.1 dBFS  S/N 8.9",
            "JJY    span 12 dB  sym 133  frames 2  err 0",
            "TX     off",
            "",
            "I = back to waterfall",
        };
        press(s, 'i');
        for (int i = 0; i < UI_STAT_LINES; i++)
            snprintf(s->stats[i], sizeof s->stats[i], "%s", demo[i]);
        sdr_prepare(s);
        write_screen(s, "stats.ppm");
        CHECK(s->show_stats && s->ntext <= UI_MAX_TEXT, "stats page fits (%d text items)", s->ntext);
        press(s, 'i');
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

        /* 空檔有補樣本：產出平均要剛好是 15625 Hz，播放端才能固定 64 µs 播 */
        ddc_init(&d);
        long total = 0;
        for (int blk = 0; blk < 20; blk++) {
            synth(508.0, 2.0, &t, 1);
            total += ddc_block(&d, g_block, SDR_BLOCK, SDR_SKIP, GAP, audio,
                               (int)(sizeof audio / sizeof audio[0]));
        }
        long want = 20L * (SDR_BLOCK + GAP) / DDC_DECIM;
        CHECK(labs(total - want) <= 1, "20 blocks with gap: %ld audio samples (want %ld = 15625 Hz)",
              total, want);
    }

    printf("[7] JJY frame format\n");
    {
        uint8_t f[60];
        jjy_time t = { 26, 282, 22, 47, 5 }, u;   /* 2026 年第 282 天 22:47，週五 */

        /* 手算的一幀：不靠 jjy_encode，直接對位元 */
        jjy_encode(&t, f);
        static const uint8_t MIN[8] = { 1, 0, 0, 0, 0, 1, 1, 1 };     /* 秒 1–8：40 20 10 0 8 4 2 1 -> 47 */
        static const uint8_t HOUR[7] = { 1, 0, 0, 0, 0, 1, 0 };       /* 秒 12–18：20 10 0 8 4 2 1 -> 22 */
        int ok = !memcmp(f + 1, MIN, 8) && !memcmp(f + 12, HOUR, 7);
        /* 282：秒 22–23 = 200,100 -> 1,0；25–28 = 80..10 -> 8 = 1,0,0,0；30–33 = 2 -> 0,0,1,0 */
        ok = ok && f[22] == 1 && f[23] == 0 && f[25] == 1 && f[26] == 0 && f[27] == 0 && f[28] == 0
                && f[30] == 0 && f[31] == 0 && f[32] == 1 && f[33] == 0;
        /* 同位：時 22 -> 位元 1+1 = 偶 -> PA1 = 0；分 47 -> 1+1+1+1 = 偶 -> PA2 = 0 */
        ok = ok && f[36] == 0 && f[37] == 0;
        /* 年 26 -> 秒 41–48：0,0,1,0, 0,1,1,0；週五 = 5 -> 秒 50–52：1,0,1 */
        static const uint8_t YEAR[8] = { 0, 0, 1, 0, 0, 1, 1, 0 };
        ok = ok && !memcmp(f + 41, YEAR, 8) && f[50] == 1 && f[51] == 0 && f[52] == 1;
        ok = ok && f[0] == JJY_M && f[9] == JJY_M && f[19] == JJY_M && f[29] == JJY_M
                && f[39] == JJY_M && f[49] == JJY_M && f[59] == JJY_M;
        CHECK(ok, "hand-computed frame for 2026/282 22:47 Fri matches bit by bit");

        CHECK(jjy_decode(f, &u) == 0 && u.year == 26 && u.yday == 282 && u.hour == 22 &&
              u.min == 47 && u.wday == 5, "decode(encode(t)) == t");

        int bad = 0;
        for (int i = 0; i < 2000; i++) {
            jjy_time r = { (int)(urand() * 100), 1 + (int)(urand() * 366), (int)(urand() * 24),
                           (int)(urand() * 60), (int)(urand() * 7) };
            jjy_encode(&r, f);
            int cs = (r.min == 15 || r.min == 45);
            if (jjy_decode(f, &u) != 0 || u.yday != r.yday || u.hour != r.hour ||
                u.min != r.min || (!cs && (u.year != r.year || u.wday != r.wday)))
                bad++;
        }
        CHECK(bad == 0, "2000 random times round-trip (%d bad)", bad);

        jjy_encode(&t, f); f[36] ^= 1;
        CHECK(jjy_decode(f, &u) == JJY_E_PARITY, "flipped PA1 -> parity error");
        jjy_encode(&t, f); f[19] = JJY_0;
        CHECK(jjy_decode(f, &u) == JJY_E_MARKER, "missing P2 -> marker error");
        jjy_encode(&t, f); f[15] = 1; f[16] = 1;   /* 時個位 2 -> 14；多兩個 1，同位不變 */
        CHECK(jjy_decode(f, &u) == JJY_E_RANGE, "hour units > 9 -> range error");
        jjy_encode(&t, f); f[30] = JJY_ERR;
        CHECK(jjy_decode(f, &u) == JJY_E_SYMBOL, "unreadable symbol -> error");

        jjy_time c = { 26, 282, 22, 45, 5 };
        jjy_encode(&c, f);
        for (int k = 40; k <= 48; k++) f[k] = JJY_ERR;   /* 呼號：這幾秒不是 0/1 */
        CHECK(jjy_decode(f, &u) == 0 && u.min == 45 && u.year == -1,
              "minute 45 with call sign in 40-48 still decodes (year = -1)");
    }

    printf("[8] JJY from a noisy envelope\n");
    {
        /* 3.5 分鐘的包絡：高 = 1.0，低 = 0.01（-20 dB），乘上慢衰落與雜訊，
         * 時間戳每 8.19 ms 一點、每 66.5 ms 跳過 2 ms（鍵盤掃描）。
         * 起點故意落在一分鐘的第 23.4 秒。 */
        static jjy j;
        jjy_init(&j);
        jjy_time t0 = { 26, 282, 22, 47, 5 };
        uint8_t f[3][60];
        for (int m = 0; m < 3; m++) {
            jjy_time t = t0;
            t.min += m;
            jjy_encode(&t, f[m]);
        }
        double t_ms = 23400.0, blk = 0;
        int locked_at = -1;
        while (t_ms < 3 * 60000.0 + 59000.0) {
            int m = (int)(t_ms / 60000), sec = (int)(t_ms / 1000) % 60;
            double in_sec = fmod(t_ms, 1000.0);
            uint8_t sy = m < 3 ? f[m][sec] : JJY_M;
            double width = sy == JJY_M ? 200 : sy == JJY_1 ? 500 : 800;
            double p = in_sec < width ? 1.0 : 0.01;
            double fade = 0.5 + 0.4 * sin(t_ms / 9000.0);     /* 約 1 分鐘一個週期的衰落 */
            double noise = -log(urand()) * 0.004;              /* 指數分布：功率雜訊 */
            jjy_push(&j, (float)(p * fade + noise), (uint32_t)t_ms);
            if (jjy_locked(&j) && locked_at < 0)
                locked_at = (int)(t_ms / 1000);
            t_ms += 8.192;
            blk += 8.192;
            if (blk >= 65.5) { blk = 0; t_ms += 2.0; }
        }
        printf("        symbols %lu, frames %lu, last err %d, history: %.30s...\n",
               (unsigned long)j.symbols, (unsigned long)j.frames, j.last_err, j.hist);
        CHECK(jjy_locked(&j) && j.t.hour == 22 && j.t.min == 49 && j.t.yday == 282,
              "locked, last decoded 22:%02d (want 22:49), locked at t=%d s", j.t.min, locked_at);
        CHECK(labs((long)j.t_ms - 2 * 60000L) < 60,
              "frame timestamp %lu ms (want 120000, the real start of 22:49)", (unsigned long)j.t_ms);
    }

    printf("[9] JJY end to end: 40 kHz carrier -> DDC -> decoder\n");
    {
        /* 真的合成 40 kHz 載波（100% / 10%），加雜訊，切成塊、每塊之間空 1 ms、
         * 開頭有偏壓暫態，丟進 DDC，再把包絡餵給解碼器。約 2.5 分鐘。 */
        static ddc d;
        static jjy j;
        static int16_t audio[1024];
        ddc_init(&d);
        ddc_set(&d, 40000, DDC_CW, 500);
        jjy_init(&j);
        jjy_time t0 = { 26, 282, 22, 47, 5 };
        uint8_t f[3][60];
        for (int m = 0; m < 3; m++) {
            jjy_time t = t0;
            t.min += m;
            jjy_encode(&t, f[m]);
        }
        const int GAP = 480;
        double t = 50.0;                        /* 從第 50 秒開始 */
        const double PI2 = 2 * 3.14159265358979323846;
        while (t < 3 * 60.0 + 3.0) {                /* 22:49 那一幀收到第 180 秒 */
            t += (double)GAP / SP_FS;
            for (int i = 0; i < SDR_BLOCK; i++, t += 1.0 / SP_FS) {
                int m = (int)(t / 60), sec = (int)t % 60;
                double in_sec = fmod(t, 1.0);
                uint8_t sy = m < 3 ? f[m][sec] : JJY_M;
                double width = sy == JJY_M ? 0.2 : sy == JJY_1 ? 0.5 : 0.8;
                double a = in_sec < width ? 6.0 : 0.6;            /* 6 LSB / 0.6 LSB */
                double v = 508.0 + 2.0 * grand() + a * sin(PI2 * 40000.0 * t);
                if (i < SDR_SKIP)
                    v -= 508.0 * exp(-i / 20.5);
                long q = lround(v);
                g_block[i] = (uint16_t)(q < 0 ? 0 : q > 4095 ? 4095 : q);
            }
            ddc_block(&d, g_block, SDR_BLOCK, SDR_SKIP, GAP, audio, 1024);
            for (int k = 0; k < d.env_n; k++)
                jjy_push(&j, d.env[k], d.env_ms[k] + 50000u);
        }
        printf("        symbols %lu, frames %lu, last err %d, history: %.30s...\n",
               (unsigned long)j.symbols, (unsigned long)j.frames, j.last_err, j.hist);
        CHECK(jjy_locked(&j) && j.t.hour == 22 && j.t.min == 49,
              "6 LSB carrier in 2 LSB noise: locked on 22:%02d (want 22:49)", j.t.min);
        CHECK(labs((long)j.t_ms - 120000L) < 60,
              "frame timestamp %lu ms (want 120000)", (unsigned long)j.t_ms);

        /* 記錄檔的兩種行（jjy.h）。鎖定之後格式該有的欄位都要在 */
        char line[200];
        jjy_log_frame(&j, 3725, 40000, -632, -771, line, sizeof line);
        printf("        %.90s...\n", line);
        CHECK(!strncmp(line, "F up=01:02:05 tune=40000 sig=-63.2 nf=-77 err=0 good=", 52) &&
              strstr(line, " jst=22:49 yday=") && strstr(line, " sym=M"),
              "frame log line has uptime, level, decoded time and the 60 symbols");
        CHECK(strlen(strstr(line, " sym=") + 5) == 60, "sym= carries exactly 60 symbols");
        jjy_log_status(&j, 59, 40000, -5, -771, line, sizeof line);
        printf("        %.90s...\n", line);
        CHECK(!strncmp(line, "S up=00:00:59 tune=40000 sig=-0.5 nf=-77 sn=76.6 span=", 54) &&
              strstr(line, " frames=") && strstr(line, " hist="),
              "status log line (sig between 0 and -1 dB keeps its sign)");
        jjy fresh;
        jjy_init(&fresh);
        fresh.last_err = JJY_E_PARITY;
        jjy_log_frame(&fresh, 0, 60000, -900, -900, line, sizeof line);
        CHECK(strstr(line, "err=-3 good=0 sym=") && !strstr(line, "jst=") &&
              line[strlen(line) - 1] == '=',
              "failed frame: no decoded-time fields ('%.60s')", line);
    }

    printf("[10] recording: R key and wav header\n");
    {
        sdr_init(s);
        CHECK(!s->rec_on && !s->rec[0], "starts not recording");
        press(s, 'r');
        CHECK(s->rec_on, "R starts recording");
        strcpy(s->rec, "REC000.WAV  3 s  1.5 MB  drop 0");
        sdr_prepare(s);
        int seen = 0;
        for (int i = 0; i < s->ntext; i++)
            seen |= (strstr(s->text[i].s, "REC000.WAV") ? 1 : 0) | (!strcmp(s->text[i].s, "  REC") ? 2 : 0);
        CHECK(seen == 3, "status line and REC tag are on screen (seen %d)", seen);
        press(s, 'r');
        CHECK(!s->rec_on, "R again stops");

        uint8_t h[WAV_HDR];
        wav_header(h, 500000, 1000);
        #define LE32(o) ((uint32_t)h[o] | (uint32_t)h[o+1] << 8 | (uint32_t)h[o+2] << 16 | (uint32_t)h[o+3] << 24)
        #define LE16(o) ((uint32_t)h[o] | (uint32_t)h[o+1] << 8)
        CHECK(!memcmp(h, "RIFF", 4) && !memcmp(h + 8, "WAVEfmt ", 8) && !memcmp(h + 36, "data", 4),
              "wav tags");
        CHECK(LE32(4) == 2036 && LE32(40) == 2000, "riff %lu / data %lu bytes (want 2036 / 2000)",
              (unsigned long)LE32(4), (unsigned long)LE32(40));
        CHECK(LE16(20) == 1 && LE16(22) == 1 && LE32(24) == 500000 && LE32(28) == 1000000 &&
              LE16(32) == 2 && LE16(34) == 16, "PCM mono 16-bit 500 kHz");
    }

    printf("[11] presets: parse, format, f / F keys\n");
    {
        preset p;
        CHECK(preset_parse("40.000 CW 500 JJY Fukushima\r\n", &p) && p.hz == 40000 &&
              p.mode == DDC_CW && p.bw_hz == 500 && !strcmp(p.name, "JJY Fukushima"),
              "full line (hz %ld, mode %d, bw %d, name '%s')", (long)p.hz, p.mode, p.bw_hz, p.name);
        CHECK(preset_parse("  68.5 cw BPC", &p) && p.hz == 68500 && p.bw_hz == ddc_default_bw(DDC_CW) &&
              !strcmp(p.name, "BPC"), "no bandwidth, lower-case mode -> default bw (%d)", p.bw_hz);
        CHECK(preset_parse("26 AM 7000 alias", &p) && p.bw_hz == ddc_default_bw(DDC_AM),
              "bandwidth not in the AM table -> default (%d)", p.bw_hz);
        CHECK(preset_parse("26.000 AM 8000 1026 kHz alias", &p) && p.bw_hz == 8000 &&
              !strcmp(p.name, "1026 kHz alias"), "name may start with a digit after bw ('%s')", p.name);
        CHECK(preset_parse("123.456 LSB", &p) && p.hz == 123456 && p.mode == DDC_LSB && !p.name[0],
              "no name");
        CHECK(!preset_parse("# comment", &p) && !preset_parse("   ", &p) && !preset_parse("", &p),
              "comment and blank lines are skipped");
        CHECK(!preset_parse("300 AM", &p) && !preset_parse("40.0001 CW", &p) &&
              !preset_parse("40 FM", &p) && !preset_parse("abc CW", &p) && !preset_parse("40x CW", &p),
              "out of range, 4 decimals, unknown mode, garbage are rejected");

        char line[64];
        preset q = { 59998, DDC_USB, 2700, "maybe JJY" }, r;
        preset_format(&q, line, sizeof line);
        CHECK(!strcmp(line, "59.998 USB 2700 maybe JJY"), "format: '%s'", line);
        CHECK(preset_parse(line, &r) && r.hz == q.hz && r.mode == q.mode && r.bw_hz == q.bw_hz &&
              !strcmp(r.name, q.name), "format -> parse round trip");

        sdr_init(s);
        int nb = s->presets.n;
        CHECK(nb == s->presets.n_builtin && nb >= 3, "%d built-in presets", nb);
        press(s, 'f');
        CHECK(s->preset_req && s->tune_hz == 68500, "f only raises a request (platform loads SD first)");
        s->preset_req = 0;
        s->ddc_dirty = 0;
        sdr_preset_next(s);
        CHECK(s->tune_hz == 40000 && s->mode == DDC_CW && s->ddc_dirty && s->msg_ttl > 0 &&
              strstr(s->msg, "PRESET 1/"), "first preset: JJY 40 ('%s')", s->msg);
        for (int i = 1; i < nb; i++)
            sdr_preset_next(s);
        sdr_preset_next(s);
        CHECK(s->preset_idx == 0 && s->tune_hz == 40000, "wraps around after the last one");

        sdr_tune(s, 26000);
        s->mode = DDC_AM;
        s->bw_hz = 8000;
        press(s, 'F');
        CHECK(s->preset_save_req, "F raises a save request");
        sdr_current_preset(s, &q);
        CHECK(preset_add(&s->presets, &q) && s->presets.n == nb + 1, "added (n %d)", s->presets.n);
        CHECK(preset_add(&s->presets, &q) && s->presets.n == nb + 1, "same freq + mode is not added twice");
        for (int i = s->presets.n; i < PRESET_MAX; i++) {
            q.hz = 100000 + i;
            preset_add(&s->presets, &q);
        }
        q.hz = 200000;
        CHECK(!preset_add(&s->presets, &q) && s->presets.n == PRESET_MAX, "full list refuses (n %d)",
              s->presets.n);

        sdr_msg(s, "SAVED 26.000 AM");
        strcpy(s->rec, "REC003.WAV 2 s");
        sdr_prepare(s);
        int seen = 0;
        for (int i = 0; i < s->ntext; i++)
            seen |= strstr(s->text[i].s, "SAVED 26.000") ? 1 : strstr(s->text[i].s, "REC003") ? 2 : 0;
        CHECK(seen == 1, "message wins over the recording line while it lasts (seen %d)", seen);
        s->msg_ttl = 0;
        sdr_prepare(s);
        seen = 0;
        for (int i = 0; i < s->ntext; i++)
            seen |= strstr(s->text[i].s, "REC003") ? 2 : 0;
        CHECK(seen == 2, "then the recording line comes back");
    }

    write_glyphs("glyphs.ppm");
    printf("        wrote glyphs.ppm\n");

    printf("\n%s (%d failed)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail ? 1 : 0;
}
