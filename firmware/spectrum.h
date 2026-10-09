/* 寬頻頻譜：一塊 ADC 樣本 -> 每個 bin 的 dBFS -> 320 個像素的軌跡。
 *
 * 一塊裡取 K 段、每段 4096 點，各做一次 FFT 再平均（dB 域平均）。
 * 段與段不重疊；塊開頭的 skip 個樣本丟掉，那是 GPIO 26 剛從鍵盤掃描切回
 * ADC、偏壓網路還在回穩的暫態（DESIGN.md §2.3）。
 *
 * 所有位準都是 dBFS × 10 的整數。
 */
#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <stdint.h>

#include "fft.h"

#define SP_N        FFT_N               /* 4096 點 */
#define SP_BINS     (SP_N / 2)          /* 0 .. fs/2 */
#define SP_W        320                 /* 螢幕寬 */
#define SP_FS       500000              /* ADC 取樣率 */

typedef struct {
    int16_t bin_db[SP_BINS];            /* 這一塊的平均，dBFS×10 */
    int16_t raw[SP_W];                  /* 每像素取所屬 bin 的最大值（窄載波才不會被平均掉） */
    int16_t trace[SP_W];                /* raw 再經指數平均，畫頻譜曲線用 */
    int16_t peak[SP_W];                 /* 峰值保持 */
    int16_t nf;                         /* 雜訊底線：所有 bin 的中位數，dBFS×10 */
    int     ema_shift;                  /* 0 = 不做指數平均 */
    int     k_used;                     /* 這一塊實際做了幾段 FFT */
    int     primed;                     /* 第一塊直接拷貝，不從 0 慢慢爬上來 */

    /* 上一塊各階段花的時間（µs）。sp_clock_us 沒設就全是 0。 */
    uint32_t t_win, t_fft, t_db, t_post;
} spectrum;

/* 量時間用的時鐘。板子上設成 time_us_32，PC 上留 NULL。
 * 放成函式指標而不是直接呼叫，這一層才不必知道 SDK 存在。 */
extern uint32_t (*sp_clock_us)(void);

void spectrum_init(spectrum *sp);

/* 處理一塊樣本（12 位元，0..4095）。回傳實際做了幾段 FFT。 */
int spectrum_block(spectrum *sp, const uint16_t *x, int n, int skip, int k);

/* 峰值保持歸零（設成目前的 raw）。 */
void spectrum_peak_reset(spectrum *sp);

/* 像素 px 涵蓋的 bin 範圍 [b0, b1)。 */
void spectrum_px_bins(int px, int *b0, int *b1);

/* bin -> Hz（×1，整數）。bin 寬 = 500000/4096 = 122.0703125 Hz。 */
static inline int32_t spectrum_bin_hz(int b)
{
    return (int32_t)(((int64_t)b * SP_FS) / SP_N);
}

#endif /* SPECTRUM_H */
