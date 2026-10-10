/* M4 錄音的 .wav 檔頭。
 *
 * 錄的是**寬頻原始樣本**：500 ksps、單聲道、16 位元，一個樣本就是 ADC 讀到的
 * 0..4095 原值，不減偏壓、不放大。當成有號 16 位元讀也一樣（都 < 32768），
 * 只是帶著約 500 的直流、振幅只用到 12 位元 —— 好處是板子上不用轉換，
 * 檔案裡就是 ADC 真正看到的東西，分析時再自己去直流。
 *
 * 純 C，不知道板子存在（test_pc.c 驗證）。
 */
#ifndef WAV_H
#define WAV_H

#include <stdint.h>

#define WAV_HDR 44

/* 寫 44 位元組的 PCM 檔頭。n = 樣本數。錄音開始時先填 0，結束再回頭補。 */
void wav_header(uint8_t h[WAV_HDR], uint32_t rate, uint32_t n);

#endif /* WAV_H */
