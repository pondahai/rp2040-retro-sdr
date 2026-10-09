/* 板子上：整個檔案用 -O3（arduino-pico 預設 -Os），fft_run 放進 RAM。
 * 實測這兩個只快了約 5%：反組譯看內迴圈每個蝴蝶約 70 條指令，那已經是
 * M0+ 的本事，要快只能少做 —— 所以才改成實數 FFT（見 fft.h）。
 * PC 上這兩個都沒有作用。 */
#if defined(ARDUINO_ARCH_RP2040)
#pragma GCC optimize("O3")
#define FFT_RAM_FUNC __attribute__((noinline, section(".time_critical.fft_run")))
#else
#define FFT_RAM_FUNC
#endif

#include "fft.h"

#include <math.h>

/* cos/sin(2πt/N)，t < N/2。複數 FFT（M 點）用偶數索引，拆分用全部。 */
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

FFT_RAM_FUNC int fft_run(int16_t *re, int16_t *im)
{
    int i, j, shifts = 0;
    int32_t mx = 0;

    /* 位元反轉重排 */
    for (i = 1, j = 0; i < FFT_M; i++) {
        int bit = FFT_M >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            int16_t t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    for (i = 0; i < FFT_M; i++) {
        int32_t a = iabs(re[i]), b = iabs(im[i]);
        if (a > mx) mx = a;
        if (b > mx) mx = b;
    }

    for (int len = 2, tstep = FFT_N / 2; len <= FFT_M; len <<= 1, tstep >>= 1) {
        int half = len >> 1;
        int32_t nmx = 0;

        /* 區塊浮點：只有快溢位才整體右移。最大值在上一級順手記好了，
         * 不必另外掃一遍。 */
        while (mx >= FFT_HEADROOM) {
            for (i = 0; i < FFT_M; i++) {
                re[i] = (int16_t)(re[i] >> 1);
                im[i] = (int16_t)(im[i] >> 1);
            }
            mx >>= 1;
            shifts++;
        }

        for (i = 0; i < FFT_M; i += len) {
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

    /* 收尾再壓一次到 16384 以下：拆分（fft_real_db_acc）裡要把兩個 bin
     * 相加再乘 cos/sin，|a+b|·32767·√2 要留在 int32 之內。 */
    while (mx >= 16384) {
        for (i = 0; i < FFT_M; i++) {
            re[i] = (int16_t)(re[i] >> 1);
            im[i] = (int16_t)(im[i] >> 1);
        }
        mx >>= 1;
        shifts++;
    }
    return shifts;
}

/* 實數 FFT 的拆分。z[k] = x[2k] + j·x[2k+1] 做完 M 點 FFT 得到 Z，則
 *
 *     E[k] = (Z[k] + Z*[M-k]) / 2          偶數點的頻譜
 *     O[k] = (Z[k] − Z*[M-k]) / (2j)       奇數點的頻譜
 *     X[k] = E[k] + W^k · O[k]，W = e^{-j2π/N}
 *
 * 為了不丟掉最低位元，這裡算的是 2X（不除 2），dB 再扣掉 6.02。
 * 2E = (ar+br) + j(ai−bi)，2O = (ai+bi) + j(br−ar)。 */
FFT_RAM_FUNC void fft_real_db_acc(const int16_t *re, const int16_t *im,
                                  int shifts, int32_t *acc)
{
    for (int k = 0; k < FFT_M; k++) {
        int kk = k ? FFT_M - k : 0;
        int32_t ar = re[k], ai = im[k], br = re[kk], bi = im[kk];
        int32_t er = ar + br, ei = ai - bi;
        int32_t orr = ai + bi, oi = br - ar;
        int32_t c = s_cos[k], s = s_sin[k];
        int32_t xr = er + ((orr * c + oi * s + 0x4000) >> 15);
        int32_t xi = ei + ((oi * c - orr * s + 0x4000) >> 15);
        /* |2X| 最大約 8 萬，平方要 64 位元 */
        uint64_t p = (uint64_t)((int64_t)xr * xr) + (uint64_t)((int64_t)xi * xi);
        acc[k] += fft_db10(p, shifts) - 60;      /* 2X -> X：−6.02 dB */
    }
}

/* 10·log10(p)·10 = 30.103·log2(p)。log2 用「最高位元位置 + 尾數查表」，
 * 尾數取 8 位元，誤差 < 0.02 dB。
 *
 * 參考點 FFT_DB10_FS 的來歷：振幅 A = 2048 的正弦、輸入先 <<3、Hann 窗
 * 的相干增益是 N/2 再乘 1/2（正弦拆成正負頻各一半）——
 *     |X| = A · 8 · N/4 = 2048 · 8 · 1024 = 2^24，p = 2^48
 *     30.103 · 48 = 1444.9 -> 1445
 * test_pc.c 用實際的正弦驗證這個數字。 */
int fft_db10(uint64_t p, int shifts)
{
    if (p == 0)
        return FFT_DB10_MIN;

    int n = 63 - __builtin_clzll(p);
    uint32_t m = (uint32_t)(n >= 8 ? (p >> (n - 8)) & 0xFF : (p << (8 - n)) & 0xFF);
    uint32_t l2 = ((uint32_t)n << 16) + s_log2[m] + ((uint32_t)shifts << 17);
    /* 30.103 / 65536 換成 Q12 輸入 × 7707 / 2^20（相對誤差 8e-5）。
     * 不用 64 位元除法：M0+ 上那是函式庫呼叫，每塊要做上萬次。
     * l2 >> 4 最大約 4e5，× 7707 約 3e9 —— 超過 int32，所以用 uint32。 */
    int v = (int)(((l2 >> 4) * 7707u) >> 20) - FFT_DB10_FS;
    return v < FFT_DB10_MIN ? FFT_DB10_MIN : v;
}
