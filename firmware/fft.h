/* 4096 點複數 FFT，int16 區塊浮點（block floating point）。
 *
 * 為什麼不是 q15 每級固定除 2：ADC 只有 12 位元，雜訊底線只有幾個 LSB。
 * 每級都 >>1 的話，12 級做完等於除以 4096，雜訊整個被截成 0，頻譜底部
 * 一片平坦 —— 看起來很乾淨，其實是量不到。所以這裡**只在快要溢位時才
 * 右移**，並記下移了幾次，換算 dB 時再補回去。
 *
 * M0+ 沒有 FPU，但有單週期 32x32 乘法：蝴蝶運算全在 int32 裡做。
 * 浮點只出現在 fft_init() 建表，開機做一次。
 *
 * 純 C，PC 與板子共用（test_pc.c 驗證）。
 */
#ifndef FFT_H
#define FFT_H

#include <stdint.h>

#define FFT_LOG2N 12
#define FFT_N     (1 << FFT_LOG2N)

/* dBFS 的參考點：振幅 ±2048 counts（12 位元滿刻度）的正弦波、Hann 窗、
 * 落在 bin 正中央時讀到 0 dBFS。推導見 fft.c。 */
#define FFT_DB10_FS 1445

void fft_init(void);

/* 原地運算。輸入順序、輸出順序都是自然順序。
 * 回傳右移的次數 —— 真正的結果 = 計算結果 × 2^shifts。 */
int fft_run(int16_t *re, int16_t *im);

/* 功率 p（re²+im²）與 fft_run 的右移次數 -> dBFS × 10。
 * p = 0 時回傳 FFT_DB10_MIN。 */
#define FFT_DB10_MIN (-1600)
int fft_db10(uint32_t p, int shifts);

#endif /* FFT_H */
