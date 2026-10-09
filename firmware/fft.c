#include "fft.h"

#include <math.h>

static int16_t s_cos[FFT_N / 2];
static int16_t s_sin[FFT_N / 2];
static uint16_t s_log2[256];         /* log2(1 + i/256)，Q16 */

/* 蝴蝶運算的輸出最多是輸入的 1+√2 ≈ 2.414 倍（a ± w·b，w 轉 45° 時最糟）。
 * 輸入的最大絕對值壓在這之下，輸出就不會超過 int16：12287 × 2.414 ≈ 29660。 */
#define FFT_HEADROOM 12288

void fft_init(void)
{
    const double PI = 3.14159265358979323846;
    for (int t = 0; t < FFT_N / 2; t++) {
        double a = 2.0 * PI * t / FFT_N;
        s_cos[t] = (int16_t)lround(cos(a) * 32767.0);
        s_sin[t] = (int16_t)lround(sin(a) * 32767.0);
    }
    for (int i = 0; i < 256; i++)
        s_log2[i] = (uint16_t)lround(log2(1.0 + i / 256.0) * 65536.0);
}

static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

int fft_run(int16_t *re, int16_t *im)
{
    int i, j, shifts = 0;
    int32_t mx = 0;

    /* 位元反轉重排 */
    for (i = 1, j = 0; i < FFT_N; i++) {
        int bit = FFT_N >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            int16_t t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    for (i = 0; i < FFT_N; i++) {
        int32_t a = iabs(re[i]), b = iabs(im[i]);
        if (a > mx) mx = a;
        if (b > mx) mx = b;
    }

    for (int len = 2, tstep = FFT_N / 2; len <= FFT_N; len <<= 1, tstep >>= 1) {
        int half = len >> 1;
        int32_t nmx = 0;

        /* 區塊浮點：只有快溢位才整體右移。最大值在上一級順手記好了，
         * 不必另外掃一遍。 */
        while (mx >= FFT_HEADROOM) {
            for (i = 0; i < FFT_N; i++) {
                re[i] = (int16_t)(re[i] >> 1);
                im[i] = (int16_t)(im[i] >> 1);
            }
            mx >>= 1;
            shifts++;
        }

        for (i = 0; i < FFT_N; i += len) {
            for (j = 0; j < half; j++) {
                int t = j * tstep;
                int32_t c = s_cos[t], s = s_sin[t];
                int k = i + j, m = k + half;
                /* (re + j·im)(cos − j·sin)，也就是乘上 e^{-j2πt/N} */
                int32_t tr = (re[m] * c + im[m] * s + 0x4000) >> 15;
                int32_t ti = (im[m] * c - re[m] * s + 0x4000) >> 15;
                int32_t ar = re[k], ai = im[k];
                int32_t v0 = ar + tr, v1 = ai + ti, v2 = ar - tr, v3 = ai - ti;
                re[k] = (int16_t)v0; im[k] = (int16_t)v1;
                re[m] = (int16_t)v2; im[m] = (int16_t)v3;
                v0 = iabs(v0); v1 = iabs(v1); v2 = iabs(v2); v3 = iabs(v3);
                if (v0 > nmx) nmx = v0;
                if (v1 > nmx) nmx = v1;
                if (v2 > nmx) nmx = v2;
                if (v3 > nmx) nmx = v3;
            }
        }
        mx = nmx;
    }
    return shifts;
}

/* 10·log10(p)·10 = 30.103·log2(p)。log2 用「最高位元位置 + 尾數查表」，
 * 尾數取 8 位元，誤差 < 0.02 dB。
 *
 * 參考點 FFT_DB10_FS 的來歷：振幅 A = 2048 的正弦、輸入先 <<3、Hann 窗
 * 的相干增益是 N/2 再乘 1/2（正弦拆成正負頻各一半）——
 *     |X| = A · 8 · N/4 = 2048 · 8 · 1024 = 2^24，p = 2^48
 *     30.103 · 48 = 1444.9 -> 1445
 * test_pc.c 用實際的正弦驗證這個數字。 */
int fft_db10(uint32_t p, int shifts)
{
    if (p == 0)
        return FFT_DB10_MIN;

    int n = 31 - __builtin_clz(p);
    uint32_t m = n >= 8 ? (p >> (n - 8)) & 0xFF : (p << (8 - n)) & 0xFF;
    int32_t l2 = ((int32_t)n << 16) + s_log2[m] + ((int32_t)shifts << 17);
    /* 30.103 / 65536 換成 Q12 輸入 × 7707 / 2^20（相對誤差 8e-5）。
     * 不用 64 位元除法：M0+ 上那是函式庫呼叫，每塊要做上萬次。
     * l2 >> 4 最大約 2.6e5，× 7707 仍在 int32 之內。 */
    int v = (int)(((l2 >> 4) * 7707) >> 20) - FFT_DB10_FS;
    return v < FFT_DB10_MIN ? FFT_DB10_MIN : v;
}
