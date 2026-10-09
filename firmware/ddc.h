/* 窄頻 DDC 與解調：一塊 500 ksps 的 ADC 樣本進來，7812.5 Hz 的音訊出去。
 *
 *   x ─→ NCO 混頻 ─→ CIC³ ÷64 ─→ FIR（通道濾波）─→ 解調 ─→ AGC ─→ 音訊
 *        (e^{-j2πf_LO t})   int64 積分器     127 階，複數
 *
 * 四種模式用同一條鏈，差別只在本振放哪裡、濾波器多寬、最後怎麼取：
 *
 *   CW   本振 = 調諧點；±bw/2 濾波；再乘 e^{+j2π·800t} 取實部 -> 800 Hz 嗶聲
 *   USB  本振 = 調諧點 + 中心；濾波；乘 e^{+j2π·中心·t} 取實部
 *   LSB  本振 = 調諧點 − 中心；濾波；取共軛後同上（頻率翻轉）
 *   AM   本振 = 調諧點；濾波；取振幅 |z|，去 DC
 *
 * 為什麼 ÷64 而不是 ÷62.5 到 8 kHz：整數抽取，一塊 32768 點剛好 512 個音訊樣本。
 *
 * 為什麼積分器用 int64：12 位元輸入 × q14 本振 = 26 位元，三階 CIC ÷64 再長
 * 18 位元 = 44 位元。CIC 靠模數運算吃溢位，但前提是暫存器夠寬裝得下輸出。
 * M0+ 的 64 位元加法是 adds/adcs 兩條指令，不貴。
 *
 * 純 C，不知道板子存在（test_pc.c 驗證）。板子上跑在 Core 1。
 */
#ifndef DDC_H
#define DDC_H

#include <stdint.h>

#define DDC_FS      500000
#define DDC_DECIM   64
#define DDC_AFS_X2  15625               /* 音訊取樣率 × 2 = 7812.5 Hz × 2 */
#define DDC_TAPS    127

enum { DDC_AM = 0, DDC_CW, DDC_USB, DDC_LSB, DDC_NMODES };

#define DDC_CW_PITCH 800                /* CW 嗶聲的音高 */

typedef struct {
    /* 參數（ddc_set 設定） */
    int32_t  tune_hz;
    int      mode;
    int      bw_hz;                     /* 整個通帶的寬度 */

    /* 本振 */
    uint32_t lo_phase, lo_step;         /* 500 kHz 下 */
    uint32_t bfo_phase, bfo_step;       /* 7812.5 Hz 下，把基頻搬回音訊 */
    int      conj;                      /* LSB 要翻頻 */

    /* CIC */
    int64_t  integ[2][3];
    int64_t  comb[2][3];
    int      dec;

    /* FIR：雙份環形緩衝，讀的時候不用折返 */
    int16_t  taps[DDC_TAPS];
    int32_t  hist[2][DDC_TAPS * 2];
    int      hpos;

    /* 解調之後 */
    float    am_dc;                     /* AM 去 DC 的慢平均 */
    float    agc_env;                   /* AGC 的包絡 */
    float    level;                     /* 通帶內的訊號強度（給 S 表），線性 */
} ddc;

void ddc_init(ddc *d);

/* 改調諧點、模式、頻寬。會重算濾波器（用浮點，只有改參數時才跑）。 */
void ddc_set(ddc *d, int32_t tune_hz, int mode, int bw_hz);

/* 處理一塊樣本，音訊寫進 out（最多 max 個），回傳寫了幾個。
 * out 是 int16，±32767 為滿刻度（AGC 之後）。
 *
 * skip：開頭丟掉幾點。GPIO 26 剛從鍵盤掃描切回 ADC，偏壓要從 0 V 爬回
 *       0.41 V（約 500 counts 的階躍）。不丟的話每塊都是一聲「噠」——
 *       M2 第一次上機就是這樣。
 * gap： 上一塊結束到這一塊 x[0] 之間，真實時間過了幾個取樣點（鍵盤掃描的
 *       空檔）。本振的相位會補上 gap + skip，載波在塊與塊之間才接得起來；
 *       不補的話每塊都有一次相位跳躍，CW 嗶聲會喀喀響。 */
int ddc_block(ddc *d, const uint16_t *x, int n, int skip, int gap,
              int16_t *out, int max);

/* 每種模式的預設頻寬與可選頻寬 */
int ddc_default_bw(int mode);
int ddc_next_bw(int mode, int bw_hz);
const char *ddc_mode_name(int mode);

#endif /* DDC_H */
