/* 4096 點實數 FFT：2048 點複數 FFT ＋ 一次拆分。int16 區塊浮點。
 *
 * 為什麼是「實數」：ADC 樣本沒有虛部。直接丟進 4096 點複數 FFT，有一半
 * 的運算在算零。把偶數點放實部、奇數點放虛部，做 2048 點複數 FFT，
 * 再用一次 O(N) 的拆分還原成 4096 點的頻譜 —— 運算量約減半。
 * 上機實測 4096 點複數 FFT 一次 13 ms（M0+、250 MHz），是 DSP 的九成。
 *
 * 為什麼不是 q15 每級固定除 2：ADC 只有 12 位元，雜訊底線只有幾個 LSB。
 * 每級都 >>1 的話雜訊整個被截成 0，頻譜底部一片平坦 —— 看起來很乾淨，
 * 其實是量不到。所以這裡**只在快要溢位時才右移**，記下移了幾次，換算
 * dB 時再補回去。
 *
 * 純 C，PC 與板子共用（test_pc.c 驗證）。
 */
#ifndef FFT_H
#define FFT_H

#include <stdint.h>

#define FFT_N     4096              /* 實數點數 */
#define FFT_M     (FFT_N / 2)       /* 內部複數 FFT 的點數 */
#define FFT_LOG2M 11

/* dBFS 的參考點：振幅 ±2048 counts（12 位元滿刻度）的正弦波、Hann 窗、
 * 落在 bin 正中央時讀到 0 dBFS。推導見 fft.c。 */
#define FFT_DB10_FS 1445
#define FFT_DB10_MIN (-1600)

void fft_init(void);

/* M 點複數 FFT，原地運算，自然順序進出。
 * 回傳右移的次數 —— 真正的結果 = 計算結果 × 2^shifts。 */
int fft_run(int16_t *re, int16_t *im);

/* 實數 FFT 的後半段。呼叫前：re[k] = x[2k]、im[k] = x[2k+1]（已乘窗），
 * 再做過 fft_run()。這裡把 bin 0..M-1 的 dBFS×10 **加**進 acc[]
 * （多段平均用）。re/im 不會被改。 */
void fft_real_db_acc(const int16_t *re, const int16_t *im, int shifts,
                     int32_t *acc);

/* 功率 p 與右移次數 -> dBFS × 10。p = 0 時回傳 FFT_DB10_MIN。 */
int fft_db10(uint64_t p, int shifts);

#endif /* FFT_H */
