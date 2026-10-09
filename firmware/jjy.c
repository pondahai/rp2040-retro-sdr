#include "jjy.h"

#include <math.h>
#include <string.h>

/* ---- 幀格式 --------------------------------------------------------------
 *
 * 每一秒放什麼。BCD 欄位用「權重」表示（0 = 不屬於那個欄位）。
 * 來源：NICT JJY 說明頁與 Wikipedia 的 JJY 條目（兩者一致）。 */

#define MARK  (-1)
#define ZERO  (-2)
#define OTHER (-3)    /* PA1、PA2、LS1、LS2：另外處理 */

/* 第幾秒是哪個欄位 */
enum { F_NONE, F_MIN, F_HOUR, F_YDAY, F_YEAR, F_WDAY };

static const struct { int8_t kind; uint8_t field; uint8_t weight; } SLOT[60] = {
    /*  0 M   */ { MARK, 0, 0 },
    /*  1     */ { 0, F_MIN, 40 }, { 0, F_MIN, 20 }, { 0, F_MIN, 10 },
    /*  4     */ { ZERO, 0, 0 },
    /*  5     */ { 0, F_MIN, 8 }, { 0, F_MIN, 4 }, { 0, F_MIN, 2 }, { 0, F_MIN, 1 },
    /*  9 P1  */ { MARK, 0, 0 },
    /* 10, 11 */ { ZERO, 0, 0 }, { ZERO, 0, 0 },
    /* 12     */ { 0, F_HOUR, 20 }, { 0, F_HOUR, 10 },
    /* 14     */ { ZERO, 0, 0 },
    /* 15     */ { 0, F_HOUR, 8 }, { 0, F_HOUR, 4 }, { 0, F_HOUR, 2 }, { 0, F_HOUR, 1 },
    /* 19 P2  */ { MARK, 0, 0 },
    /* 20, 21 */ { ZERO, 0, 0 }, { ZERO, 0, 0 },
    /* 22     */ { 0, F_YDAY, 200 }, { 0, F_YDAY, 100 },
    /* 24     */ { ZERO, 0, 0 },
    /* 25     */ { 0, F_YDAY, 80 }, { 0, F_YDAY, 40 }, { 0, F_YDAY, 20 }, { 0, F_YDAY, 10 },
    /* 29 P3  */ { MARK, 0, 0 },
    /* 30     */ { 0, F_YDAY, 8 }, { 0, F_YDAY, 4 }, { 0, F_YDAY, 2 }, { 0, F_YDAY, 1 },
    /* 34, 35 */ { ZERO, 0, 0 }, { ZERO, 0, 0 },
    /* 36 PA1 */ { OTHER, 0, 0 },
    /* 37 PA2 */ { OTHER, 0, 0 },
    /* 38 SU1 */ { ZERO, 0, 0 },
    /* 39 P4  */ { MARK, 0, 0 },
    /* 40 SU2 */ { ZERO, 0, 0 },
    /* 41     */ { 0, F_YEAR, 80 }, { 0, F_YEAR, 40 }, { 0, F_YEAR, 20 }, { 0, F_YEAR, 10 },
    /* 45     */ { 0, F_YEAR, 8 }, { 0, F_YEAR, 4 }, { 0, F_YEAR, 2 }, { 0, F_YEAR, 1 },
    /* 49 P5  */ { MARK, 0, 0 },
    /* 50     */ { 0, F_WDAY, 4 }, { 0, F_WDAY, 2 }, { 0, F_WDAY, 1 },
    /* 53 LS1 */ { OTHER, 0, 0 },
    /* 54 LS2 */ { OTHER, 0, 0 },
    /* 55–58  */ { ZERO, 0, 0 }, { ZERO, 0, 0 }, { ZERO, 0, 0 }, { ZERO, 0, 0 },
    /* 59 P0  */ { MARK, 0, 0 },
};

/* 第 15、45 分鐘：40–48 秒送呼號，50–55 秒是停播預告。這幾秒不檢查。 */
static int callsign_slot(int sec)
{
    return (sec >= 40 && sec <= 48) || (sec >= 50 && sec <= 58);
}

int jjy_decode(const uint8_t sym[60], jjy_time *out)
{
    int v[6] = { 0 }, pa1 = 0, pa2 = 0, s;

    /* 先看分鐘：決定這一幀有沒有呼號 */
    for (s = 0; s < 60; s++)
        if (SLOT[s].kind == MARK && sym[s] != JJY_M)
            return JJY_E_MARKER;
    for (s = 1; s <= 8; s++)
        if (sym[s] == JJY_1 && SLOT[s].field == F_MIN)
            v[F_MIN] += SLOT[s].weight;
    int callsign = (v[F_MIN] == 15 || v[F_MIN] == 45);

    memset(v, 0, sizeof v);
    for (s = 0; s < 60; s++) {
        if (SLOT[s].kind == MARK)
            continue;
        if (callsign && callsign_slot(s))
            continue;
        if (sym[s] != JJY_0 && sym[s] != JJY_1)
            return JJY_E_SYMBOL;
        if (SLOT[s].kind == ZERO) {
            if (sym[s] != JJY_0)
                return JJY_E_ZERO;
            continue;
        }
        if (SLOT[s].field && sym[s] == JJY_1) {
            v[SLOT[s].field] += SLOT[s].weight;
            if (SLOT[s].field == F_HOUR) pa1 ^= 1;
            if (SLOT[s].field == F_MIN)  pa2 ^= 1;
        }
    }

    /* 偶同位：PA1 = 時的位元和 mod 2，PA2 = 分的位元和 mod 2 */
    if (sym[36] != pa1 || sym[37] != pa2)
        return JJY_E_PARITY;

    if (v[F_MIN] > 59 || v[F_HOUR] > 23 || v[F_YDAY] < 1 || v[F_YDAY] > 366)
        return JJY_E_RANGE;
    /* BCD 的每一位數不能超過 9：8+4+2+1 = 15 也加得出來，總和檢查抓不到 */
    {
        static const uint8_t NIBBLE[][4] = {
            { 5, 6, 7, 8 },       /* 分 個位 */
            { 15, 16, 17, 18 },   /* 時 個位 */
            { 25, 26, 27, 28 },   /* 日 十位 */
            { 30, 31, 32, 33 },   /* 日 個位 */
        };
        for (unsigned k = 0; k < sizeof NIBBLE / sizeof NIBBLE[0]; k++) {
            const uint8_t *b = NIBBLE[k];
            if (((sym[b[0]] << 3) | (sym[b[1]] << 2) | (sym[b[2]] << 1) | sym[b[3]]) > 9)
                return JJY_E_RANGE;
        }
    }
    if (!callsign && (v[F_YEAR] > 99 || v[F_WDAY] > 6))
        return JJY_E_RANGE;

    out->min = v[F_MIN];
    out->hour = v[F_HOUR];
    out->yday = v[F_YDAY];
    out->year = callsign ? -1 : v[F_YEAR];
    out->wday = callsign ? -1 : v[F_WDAY];
    return 0;
}

/* 權重 -> 在 BCD 數字裡是第幾個位元：1,2,4,8 -> 0–3；10..80 -> 4–7；100,200 -> 8–9 */
static int weight_bit(int w)
{
    int base = w >= 100 ? 8 : w >= 10 ? 4 : 0;
    int u = w >= 100 ? w / 100 : w >= 10 ? w / 10 : w;
    return base + (u == 1 ? 0 : u == 2 ? 1 : u == 4 ? 2 : 3);
}

void jjy_encode(const jjy_time *t, uint8_t sym[60])
{
    int val[6] = { 0 };
    val[F_MIN] = t->min;
    val[F_HOUR] = t->hour;
    val[F_YDAY] = t->yday;
    val[F_YEAR] = t->year < 0 ? 0 : t->year;
    val[F_WDAY] = t->wday < 0 ? 0 : t->wday;

    int pa1 = 0, pa2 = 0;
    for (int s = 0; s < 60; s++) {
        sym[s] = JJY_0;
        if (SLOT[s].kind == MARK) {
            sym[s] = JJY_M;
            continue;
        }
        if (!SLOT[s].field)
            continue;
        int x = val[SLOT[s].field];
        int bcd = (x / 100) << 8 | (x / 10 % 10) << 4 | (x % 10);
        sym[s] = (uint8_t)((bcd >> weight_bit(SLOT[s].weight)) & 1);
        if (sym[s] == JJY_1) {
            int f = SLOT[s].field;
            if (f == F_HOUR) pa1 ^= 1;
            if (f == F_MIN)  pa2 ^= 1;
        }
    }
    sym[36] = (uint8_t)pa1;
    sym[37] = (uint8_t)pa2;
}

/* ---- 波形 -> 符號 ---------------------------------------------------------- */

/* 一個狀態要維持這麼久才算數：濾掉雜訊造成的短暫翻轉。
 * 最短的合法脈衝是 0.2 s（標記），最短的低電位是 0.2 s（「0」的後段）。 */
#define JJY_CONFIRM_MS 80

static char sym_char(uint8_t s)
{
    return s == JJY_0 ? '0' : s == JJY_1 ? '1' : s == JJY_M ? 'M' : '?';
}

static void frame_push(jjy *j, uint8_t s, uint32_t t_rise);

static void emit(jjy *j, uint8_t s, uint32_t t_rise)
{
    memmove(j->hist, j->hist + 1, JJY_HIST - 1);
    j->hist[JJY_HIST - 1] = sym_char(s);
    j->symbols++;
    frame_push(j, s, t_rise);
}

static uint8_t classify(uint32_t width_ms)
{
    if (width_ms < 100)  return JJY_ERR;
    if (width_ms < 350)  return JJY_M;     /* 0.2 s */
    if (width_ms < 650)  return JJY_1;     /* 0.5 s */
    if (width_ms < 950)  return JJY_0;     /* 0.8 s */
    return JJY_ERR;
}

void jjy_init(jjy *j)
{
    memset(j, 0, sizeof(*j));
    j->pos = -1;
    j->last_sym = JJY_ERR;
    memset(j->hist, '.', JJY_HIST);
    j->hist[JJY_HIST] = 0;
}

void jjy_push(jjy *j, float power, uint32_t t)
{
    float x = 10.0f * log10f(power + 1e-12f);

    if (!j->primed) {
        j->hi = j->lo = j->smooth = x;
        j->primed = 1;
        return;
    }
    /* 輕微平滑（約 3 點），再追高低準位：快攻慢放 */
    j->smooth += (x - j->smooth) * 0.4f;
    x = j->smooth;
    if (x > j->hi) j->hi += (x - j->hi) * 0.3f;
    else           j->hi -= 0.01f;            /* 約 1.2 dB/s */
    if (x < j->lo) j->lo += (x - j->lo) * 0.3f;
    else           j->lo += 0.01f;

    float span = j->hi - j->lo;
    if (span < 6.0f)                          /* 高低差不到 6 dB = 沒訊號 */
        return;
    float th = (j->hi + j->lo) * 0.5f, h = span * 0.1f;

    if (j->high) {
        /* 等下降邊緣 = 脈衝結束，量寬度 */
        if (x < th - h) {
            if (!j->has_cand) { j->t_cand = t; j->has_cand = 1; }
            if (t - j->t_cand >= JJY_CONFIRM_MS) {
                j->high = 0;
                j->has_cand = 0;
                if (j->have_rise)
                    emit(j, classify(j->t_cand - j->t_rise), j->t_rise);
            }
        } else {
            j->has_cand = 0;
        }
    } else {
        /* 等上升邊緣 = 新的一秒開始 */
        if (x > th + h) {
            if (!j->has_cand) { j->t_cand = t; j->has_cand = 1; }
            if (t - j->t_cand >= JJY_CONFIRM_MS) {
                j->high = 1;
                j->has_cand = 0;
                if (j->have_rise) {
                    /* 中間漏掉的秒數補上「?」，幀的位置才不會錯位 */
                    uint32_t period = j->t_cand - j->t_rise;
                    int missed = (int)((period + 500) / 1000) - 1;
                    for (int k = 0; k < missed && k < 60; k++)
                        emit(j, JJY_ERR, j->t_rise + 1000u * (uint32_t)(k + 1));
                }
                j->t_rise = j->t_cand;
                j->have_rise = 1;
            }
        } else {
            j->has_cand = 0;
        }
    }
}

/* ---- 符號 -> 幀 ------------------------------------------------------------ */

static int32_t minute_of_year(const jjy_time *t)
{
    return ((int32_t)t->yday * 24 + t->hour) * 60 + t->min;
}

static void frame_push(jjy *j, uint8_t s, uint32_t t_rise)
{
    /* 兩個標記連在一起 = P0 + M：這個 M 是第 0 秒。對齊中途看到也重新對齊 */
    if (j->last_sym == JJY_M && s == JJY_M) {
        j->frame[0] = JJY_M;
        j->pos = 1;
        j->t_frame = t_rise;
    } else if (j->pos >= 0) {
        j->frame[j->pos++] = s;
    }
    j->last_sym = s;

    if (j->pos == 60) {
        jjy_time t;
        int err = jjy_decode(j->frame, &t);
        j->frames++;
        j->last_err = err;
        if (err == 0) {
            if (j->good > 0 && minute_of_year(&t) == minute_of_year(&j->t) + 1)
                j->good++;
            else
                j->good = 1;
            j->t = t;
            j->t_ms = j->t_frame;
        } else {
            j->good = 0;
        }
        /* 下一幀從下一個 M 開始；frame[59] 就是 P0，配上下一個 M 自然對齊 */
        j->pos = -1;
        j->last_sym = j->frame[59];
    }
}
