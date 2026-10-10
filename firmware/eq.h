/* eq.h — 喇叭音色：音訊輸出前的幾個 biquad（定點）
 *
 * 機上的小喇叭（PAM8403 推）放不出 300 Hz 以下，低音只會吃掉功率、讓功放
 * 削頂；語音好不好辨認主要靠 1.5–3.5 kHz。耳機聽起來比較清楚就是這個差別。
 * 這裡提供幾組預設，鍵盤 e 輪流切：
 *
 *   FLAT   不處理
 *   SPK    高通 300 Hz ＋ 2 kHz +6 dB
 *   SPK+   高通 400 Hz ＋ 2.5 kHz +9 dB（開機預設，上機實測最清楚）
 *   VOICE  高通 300 Hz ＋ 2 kHz +6 dB ＋ 低通 3.2 kHz（削掉嘶嘶聲）
 *
 * 係數 Q13（|係數| < 4），狀態是 int16 範圍，乘積加總不超過 int32。
 * Core 1 每塊約 1039 個樣本 × 最多 3 節，約 1 ms。 */
#ifndef EQ_H
#define EQ_H

#include <stdint.h>

#define EQ_NPRESETS 4
#define EQ_MAX_BQ   3

typedef struct {
    int32_t b0, b1, b2, a1, a2;     /* Q13，a0 已除掉 */
    int32_t x1, x2, y1, y2;
} eq_bq;

typedef struct {
    int   preset;
    int   nbq;
    eq_bq bq[EQ_MAX_BQ];
} eq;

/* 換預設（會清狀態）。fs = 音訊取樣率。用浮點，只有換預設時才跑。 */
void eq_set(eq *e, int preset, double fs);

/* 就地處理 n 個樣本 */
void eq_run(eq *e, int16_t *x, int n);

const char *eq_name(int preset);

#endif /* EQ_H */
