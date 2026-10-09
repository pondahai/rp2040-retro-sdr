/* JJY 授時碼解碼（日本，40 / 60 kHz）。
 *
 * 格式（NICT 官方，見 docs/DEVLOG.md 的來源）：
 *   - 每秒一個符號。每秒開頭載波 100%，經過「脈衝寬度」後降到 10%：
 *       0.8 s = 0，0.5 s = 1，0.2 s = 標記
 *   - 60 秒一幀。標記在第 0（M）、9、19、29、39、49、59（P1–P5、P0）秒；
 *     P0 緊接著 M = 一分鐘的開頭。
 *   - 時間碼代表 **M 那一刻**（這一分鐘的第 0 秒），JST。
 *   - 第 15、45 分鐘的第 40–48 秒改送呼號，年份不能用。
 *
 * 輸入是 ddc.c 的包絡：通帶內的平均功率＋真實時間戳（ms）。
 *
 * 三層，每一層都能單獨測：
 *   jjy_push()    包絡 -> 判斷強弱 -> 找邊緣 -> 每秒一個符號
 *   （內部）       符號 -> 對齊幀 -> 60 個一組
 *   jjy_decode()  60 個符號 -> 時間（純函式）
 *
 * 純 C，不知道板子存在（test_pc.c 驗證）。
 */
#ifndef JJY_H
#define JJY_H

#include <stdint.h>

enum { JJY_0 = 0, JJY_1 = 1, JJY_M = 2, JJY_ERR = 3 };

typedef struct {
    int year;        /* 兩位數 0–99；第 15、45 分鐘沒有（-1） */
    int yday;        /* 1–366 */
    int hour, min;   /* JST */
    int wday;        /* 0 = 週日 */
} jjy_time;

/* 解一幀。成功回 0；否則回負值，見 JJY_E_*。 */
#define JJY_E_MARKER  (-1)   /* 標記不在該在的位置 */
#define JJY_E_ZERO    (-2)   /* 固定為 0 的位元不是 0 */
#define JJY_E_PARITY  (-3)
#define JJY_E_RANGE   (-4)   /* 數字超出範圍（例如 25 點） */
#define JJY_E_SYMBOL  (-5)   /* 有符號沒收到 */
int jjy_decode(const uint8_t sym[60], jjy_time *out);

/* 反過來：時間 -> 60 個符號。測試用，也拿來當格式的第二份實作。 */
void jjy_encode(const jjy_time *t, uint8_t sym[60]);

#define JJY_HIST 60

typedef struct {
    /* 準位追蹤（dB） */
    float  hi, lo, smooth;
    int    primed;
    int    high;                 /* 目前判為高（載波 100%） */
    uint32_t t_cand;             /* 候選邊緣的時間（要維持 80 ms 才算數） */
    int      has_cand;
    uint32_t t_rise;             /* 上一次上升邊緣（每秒的開頭） */
    int    have_rise;

    /* 幀 */
    uint8_t frame[60];
    int     pos;                 /* 下一個符號放在 frame 的哪裡；-1 = 還沒對齊 */
    uint8_t last_sym;
    uint32_t t_frame;            /* 這一幀的 M 的時間戳 */

    /* 結果 */
    jjy_time t;                  /* 最近一次解出來的 */
    uint32_t t_ms;               /* 它對應的 M 的時間戳（ms，與 jjy_push 同一時基） */
    int      good;               /* 連續解成功、且彼此一致的幀數 */
    int      last_err;           /* 最近一次解碼的結果（0 或 JJY_E_*） */
    uint32_t frames, symbols;

    /* 給畫面：最近 60 個符號，'0' '1' 'M' '?' */
    char   hist[JJY_HIST + 1];
} jjy;

void jjy_init(jjy *j);

/* 餵一個包絡點：功率（線性）與時間戳（ms，單調遞增）。 */
void jjy_push(jjy *j, float power, uint32_t t_ms);

/* 鎖定 = 連續兩幀都解成功，而且第二幀剛好是第一幀加一分鐘。 */
static inline int jjy_locked(const jjy *j) { return j->good >= 2; }

#endif /* JJY_H */
