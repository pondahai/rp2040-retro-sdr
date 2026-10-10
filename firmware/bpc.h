/* BPC 授時碼解碼（中國，68.5 kHz，河南商丘，國家授時中心）。
 *
 * 官方格式沒有公開，以下依 Wikipedia 的 BPC 條目與 libdriver/bpc（實際能用的
 * 解碼器）的解析順序整理，兩者一致（見 docs/DEVLOG.md）：
 *
 *   - 每秒一個符號、2 位元（四進位）。**每秒開頭**載波降到約 10%，
 *     0.1／0.2／0.3／0.4 s 後恢復 = 0／1／2／3。跟 JJY 相反（JJY 是每秒開頭高）。
 *   - 20 秒一幀，一分鐘三幀，從第 0、20、40 秒開始。
 *   - P0 那一秒載波不降 = 幀首標記。時間碼代表 **P0 那一刻**，北京時間（UTC+8）。
 *
 *     P0  標記          P10 高位 = 下午、低位 = P1–P9 的偶同位
 *     P1  秒 / 20（0–2）  P11–P13 日（6 位元）
 *     P2  未用           P14–P15 月（4 位元）
 *     P3–P4  時（12 小時制，4 位元）   P16–P18 年（6 位元，再加 P19 高位 × 64）
 *     P5–P7  分（6 位元）              P19 高位 = 年 +64、低位 = P11–P18 的偶同位
 *     P8–P9  星期（1 = 週一 … 7 = 週日）
 *
 *   數值都是純二進位（不是 BCD），每個符號高位在前。
 *
 * 跟 jjy.h 一樣分三層，純 C，test_pc.c 驗證。 */
#ifndef BPC_H
#define BPC_H

#include <stdint.h>

enum { BPC_M = 4, BPC_ERR = 5 };          /* 0–3 是資料符號 */

typedef struct {
    int year;          /* 0–127（加 2000） */
    int month, day;
    int wday;          /* 1 = 週一 … 7 = 週日 */
    int hour, min;     /* 北京時間，24 小時制 */
    int sec;           /* 0、20、40：這一幀 P0 的秒 */
} bpc_time;

#define BPC_E_MARKER  (-1)   /* P0 不是標記，或資料位置出現標記 */
#define BPC_E_PARITY  (-3)
#define BPC_E_RANGE   (-4)
#define BPC_E_SYMBOL  (-5)   /* 有符號沒收到 */
int bpc_decode(const uint8_t sym[20], bpc_time *out);
void bpc_encode(const bpc_time *t, uint8_t sym[20]);

#define BPC_HIST 60

typedef struct {
    /* 準位追蹤（dB），同 jjy */
    float  hi, lo, smooth;
    int    primed;
    int    low;                  /* 目前判為低（載波降下來了） */
    uint32_t t_cand;
    int      has_cand;
    uint32_t t_fall;             /* 上一次下降邊緣（每秒的開頭） */
    int    have_fall;

    uint8_t frame[20];
    int     pos;                 /* -1 = 還沒對齊 */
    uint32_t t_frame;            /* 這一幀 P0 的時間戳 */

    bpc_time t;                  /* 最近一次解出來的 */
    uint32_t t_ms;               /* 它的 P0 的時間戳（ms，與 bpc_push 同一時基） */
    int      good;               /* 連續解成功、且彼此差 20 秒的幀數 */
    int      last_err;
    uint32_t frames, symbols;

    char   hist[BPC_HIST + 1];   /* 最近 60 個符號：'0'–'3' 'M' '?' */
    char   last_frame[21];
} bpc;

void bpc_init(bpc *b);
void bpc_push(bpc *b, float power, uint32_t t_ms);

/* 記錄檔（SD 卡 BPCLOG.TXT）的一行，格式同 jjy_log_*：
 *   S up=… tune=68500 sig=… nf=… sn=… span=… sym=… frames=… good=… err=… hist=…
 *   F up=… tune=68500 sig=… nf=… err=0 good=2 cst=22:49:20 date=2026-10-10 wday=6 sym=M0…
 * 鎖定前一幀一行；20 秒一幀，一分鐘三行。 */
void bpc_log_status(const bpc *b, uint32_t up_s, int32_t tune_hz, int sig10, int nf10,
                    char *buf, int size);
void bpc_log_frame(const bpc *b, uint32_t up_s, int32_t tune_hz, int sig10, int nf10,
                   char *buf, int size);

/* 鎖定 = 連續兩幀都解成功，而且第二幀剛好是第一幀加 20 秒。 */
static inline int bpc_locked(const bpc *b) { return b->good >= 2; }

#endif /* BPC_H */
