/* 窄頻 DDC 與解調：一塊 500 ksps 的 ADC 樣本進來，7812.5 Hz 的音訊出去。
 *
 *   x ─→ NCO 混頻 ─→ CIC³ ÷8 ─→ CIC³ ÷8 ─→ FIR（通道濾波）─→ 解調 ─→ AGC ─→ 音訊
 *        (e^{-j2πf_LO t})  int32       int64      127 階，對稱
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
 * 為什麼拆兩級 ÷8：CIC 靠模數運算吃溢位，前提是暫存器裝得下**輸出**。
 * 單級 ÷64 要 44 位元，只能用 int64，而 M0+ 只有 8 個低位暫存器，六個 int64
 * 積分器一直搬進搬出 —— 上機實測每個輸入樣本 135 週期，18 ms/塊。
 * 拆成兩級之後，跑在 500 kHz 的第一級只長 9 位元：樣本（扣 DC 後最大約
 * ±3587，12 位元）× q10 本振 × 2^9 < 2^31，int32 就夠；int64 只留給
 * 62.5 kHz 的第二級，次數少 8 倍。
 * 兩級 CIC³(÷8) 串接的響應與單級 CIC³(÷64) **完全相同**（sinc 比值相乘，
 * 中間項互消），濾波特性不變。
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
#define DDC_ENV_DECIM 64                /* 7812.5 / 64 ≈ 122 Hz，每 8.2 ms 一點 */
#define DDC_ENV_MAX   16

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

    /* CIC 第一級（500 kHz，int32；用 uint32 讓溢位有定義） */
    uint32_t integ_a[2][3];
    uint32_t comb_a[2][3];
    int      dec_a;
    /* CIC 第二級（62.5 kHz，int64） */
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

    /* 包絡輸出（給授時碼解碼）：每 DDC_ENV_DECIM 個音訊樣本取一次通帶內的
     * 平均功率（AGC 之前），附帶真實時間戳。時間戳把鍵盤掃描的空檔也算進去，
     * 不然串起來的「一秒」會短 3%。 */
    float    env[DDC_ENV_MAX];
    uint32_t env_ms[DDC_ENV_MAX];
    int      env_n;                     /* 這一塊產生了幾個 */
    float    env_acc;
    int      env_cnt;
    uint64_t t_samples;                 /* 從開機算起，經過的真實 500 kHz 取樣數 */

    /* 上一塊的耗時（µs）：整塊、其中 FIR＋解調的部分。ddc_clock_us 沒設就是 0。 */
    uint32_t t_total, t_post;
} ddc;

/* 量時間用的時鐘，同 spectrum.h 的 sp_clock_us。 */
extern uint32_t (*ddc_clock_us)(void);

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
