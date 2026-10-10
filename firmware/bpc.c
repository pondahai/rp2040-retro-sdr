#include "bpc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- 幀格式 ---------------------------------------------------------------- */

static int popcount2(int v) { return (v & 1) + ((v >> 1) & 1); }

int bpc_decode(const uint8_t sym[20], bpc_time *out)
{
    if (sym[0] != BPC_M)
        return BPC_E_MARKER;
    for (int s = 1; s < 20; s++) {
        if (sym[s] == BPC_M)
            return BPC_E_MARKER;
        if (sym[s] > 3)
            return BPC_E_SYMBOL;
    }

    /* 偶同位：P10 低位涵蓋 P1–P9，P19 低位涵蓋 P11–P18（同一秒的高位不算） */
    int p = 0;
    for (int s = 1; s <= 9; s++)
        p += popcount2(sym[s]);
    if ((p & 1) != (sym[10] & 1))
        return BPC_E_PARITY;
    p = 0;
    for (int s = 11; s <= 18; s++)
        p += popcount2(sym[s]);
    if ((p & 1) != (sym[19] & 1))
        return BPC_E_PARITY;

    int idx = sym[1];
    int hour = sym[3] << 2 | sym[4];
    int min = sym[5] << 4 | sym[6] << 2 | sym[7];
    int wday = sym[8] << 2 | sym[9];
    int pm = sym[10] >> 1;
    int day = sym[11] << 4 | sym[12] << 2 | sym[13];
    int month = sym[14] << 2 | sym[15];
    int year = (sym[16] << 4 | sym[17] << 2 | sym[18]) + (sym[19] >> 1) * 64;

    if (idx > 2 || hour > 11 || min > 59 || wday < 1 || wday > 7 ||
        day < 1 || day > 31 || month < 1 || month > 12)
        return BPC_E_RANGE;

    out->sec = idx * 20;
    out->hour = hour + (pm ? 12 : 0);
    out->min = min;
    out->wday = wday;
    out->day = day;
    out->month = month;
    out->year = year;
    return 0;
}

void bpc_encode(const bpc_time *t, uint8_t sym[20])
{
    int h12 = t->hour % 12, pm = t->hour >= 12, y = t->year % 64;
    memset(sym, 0, 20);
    sym[0] = BPC_M;
    sym[1] = (uint8_t)(t->sec / 20);
    sym[3] = (uint8_t)(h12 >> 2);         sym[4] = (uint8_t)(h12 & 3);
    sym[5] = (uint8_t)(t->min >> 4);      sym[6] = (uint8_t)(t->min >> 2 & 3);
    sym[7] = (uint8_t)(t->min & 3);
    sym[8] = (uint8_t)(t->wday >> 2);     sym[9] = (uint8_t)(t->wday & 3);
    sym[11] = (uint8_t)(t->day >> 4);     sym[12] = (uint8_t)(t->day >> 2 & 3);
    sym[13] = (uint8_t)(t->day & 3);
    sym[14] = (uint8_t)(t->month >> 2);   sym[15] = (uint8_t)(t->month & 3);
    sym[16] = (uint8_t)(y >> 4);          sym[17] = (uint8_t)(y >> 2 & 3);
    sym[18] = (uint8_t)(y & 3);
    int p = 0;
    for (int s = 1; s <= 9; s++)
        p += popcount2(sym[s]);
    sym[10] = (uint8_t)(pm << 1 | (p & 1));
    p = 0;
    for (int s = 11; s <= 18; s++)
        p += popcount2(sym[s]);
    sym[19] = (uint8_t)((t->year >= 64) << 1 | (p & 1));
}

/* ---- 波形 -> 符號 ----------------------------------------------------------- */

/* 最短的低電位是 0.1 s（符號 0），確認時間要比它短。包絡每 8.2 ms 一點，40 ms ≈ 5 點。 */
#define BPC_CONFIRM_MS 40

static char sym_char(uint8_t s)
{
    return s <= 3 ? (char)('0' + s) : s == BPC_M ? 'M' : '?';
}

static void frame_push(bpc *b, uint8_t s, uint32_t t_sec);

static void emit(bpc *b, uint8_t s, uint32_t t_sec)
{
    memmove(b->hist, b->hist + 1, BPC_HIST - 1);
    b->hist[BPC_HIST - 1] = sym_char(s);
    b->symbols++;
    frame_push(b, s, t_sec);
}

/* 低電位寬度 -> 符號。0.1 s 一階，邊界取中間 */
static uint8_t classify(uint32_t w)
{
    if (w < 50)  return BPC_ERR;
    if (w < 150) return 0;
    if (w < 250) return 1;
    if (w < 350) return 2;
    if (w < 480) return 3;
    return BPC_ERR;
}

void bpc_init(bpc *b)
{
    memset(b, 0, sizeof(*b));
    b->pos = -1;
    memset(b->hist, '.', BPC_HIST);
    b->hist[BPC_HIST] = 0;
}

void bpc_push(bpc *b, float power, uint32_t t)
{
    float x = 10.0f * log10f(power + 1e-12f);

    if (!b->primed) {
        b->hi = b->lo = b->smooth = x;
        b->primed = 1;
        return;
    }
    /* 同 jjy：輕微平滑，高低準位快攻慢放 */
    b->smooth += (x - b->smooth) * 0.4f;
    x = b->smooth;
    if (x > b->hi) b->hi += (x - b->hi) * 0.3f;
    else           b->hi -= 0.01f;
    if (x < b->lo) b->lo += (x - b->lo) * 0.3f;
    else           b->lo += 0.01f;

    float span = b->hi - b->lo;
    if (span < 6.0f)
        return;
    float th = (b->hi + b->lo) * 0.5f, h = span * 0.1f;

    if (!b->low) {
        /* 等下降邊緣 = 新的一秒開始 */
        if (x < th - h) {
            if (!b->has_cand) { b->t_cand = t; b->has_cand = 1; }
            if (t - b->t_cand >= BPC_CONFIRM_MS) {
                b->low = 1;
                b->has_cand = 0;
                if (b->have_fall) {
                    /* 隔了兩秒 = 中間那一秒沒降 = P0 標記；更多則補「?」 */
                    uint32_t period = b->t_cand - b->t_fall;
                    int n = (int)((period + 500) / 1000);
                    if (n == 2)
                        emit(b, BPC_M, b->t_fall + 1000u);
                    else
                        for (int k = 1; k < n && k < 20; k++)
                            emit(b, BPC_ERR, b->t_fall + 1000u * (uint32_t)k);
                }
                b->t_fall = b->t_cand;
                b->have_fall = 1;
            }
        } else {
            b->has_cand = 0;
        }
    } else {
        /* 等上升邊緣 = 低電位結束，量寬度 */
        if (x > th + h) {
            if (!b->has_cand) { b->t_cand = t; b->has_cand = 1; }
            if (t - b->t_cand >= BPC_CONFIRM_MS) {
                b->low = 0;
                b->has_cand = 0;
                emit(b, classify(b->t_cand - b->t_fall), b->t_fall);
            }
        } else {
            b->has_cand = 0;
        }
    }
}

/* ---- 符號 -> 幀 ------------------------------------------------------------- */

static int32_t sec_of_day(const bpc_time *t)
{
    return ((int32_t)t->hour * 60 + t->min) * 60 + t->sec;
}

static void frame_push(bpc *b, uint8_t s, uint32_t t_sec)
{
    if (s == BPC_M) {                     /* 標記 = 新的一幀；中途看到也重新對齊 */
        b->frame[0] = BPC_M;
        b->pos = 1;
        b->t_frame = t_sec;
        return;
    }
    if (b->pos < 0)
        return;
    b->frame[b->pos++] = s;
    if (b->pos < 20)
        return;

    bpc_time t;
    int err = bpc_decode(b->frame, &t);
    for (int i = 0; i < 20; i++)
        b->last_frame[i] = sym_char(b->frame[i]);
    b->last_frame[20] = 0;
    b->frames++;
    b->last_err = err;
    if (err == 0) {
        if (b->good > 0 && sec_of_day(&t) == (sec_of_day(&b->t) + 20) % 86400)
            b->good++;
        else
            b->good = 1;
        b->t = t;
        b->t_ms = b->t_frame;
    } else {
        b->good = 0;
    }
    b->pos = -1;                          /* 下一幀等下一個標記 */
}

/* ---- 記錄檔的一行（見 bpc.h） ------------------------------------------------ */

static const char *db10(char *s, int v)
{
    sprintf(s, "%s%d.%d", v < 0 ? "-" : "", abs(v) / 10, abs(v) % 10);
    return s;
}

static int log_head(char *buf, int size, char kind, uint32_t up_s, int32_t tune_hz,
                    int sig10, int nf10)
{
    char a[16];
    return snprintf(buf, (size_t)size, "%c up=%02lu:%02lu:%02lu tune=%ld sig=%s nf=%d", kind,
                    (unsigned long)(up_s / 3600), (unsigned long)(up_s / 60 % 60),
                    (unsigned long)(up_s % 60), (long)tune_hz, db10(a, sig10), nf10 / 10);
}

void bpc_log_status(const bpc *b, uint32_t up_s, int32_t tune_hz, int sig10, int nf10,
                    char *buf, int size)
{
    char a[16];
    int k = log_head(buf, size, 'S', up_s, tune_hz, sig10, nf10);
    if (k < 0 || k >= size)
        return;
    snprintf(buf + k, (size_t)(size - k), " sn=%s span=%d sym=%lu frames=%lu good=%d err=%d hist=%s",
             db10(a, sig10 - nf10), (int)(b->hi - b->lo), (unsigned long)b->symbols,
             (unsigned long)b->frames, b->good, b->last_err, b->hist);
}

void bpc_log_frame(const bpc *b, uint32_t up_s, int32_t tune_hz, int sig10, int nf10,
                   char *buf, int size)
{
    int k = log_head(buf, size, 'F', up_s, tune_hz, sig10, nf10);
    if (k < 0 || k >= size)
        return;
    k += snprintf(buf + k, (size_t)(size - k), " err=%d good=%d", b->last_err, b->good);
    if (k >= size)
        return;
    if (b->last_err == 0) {
        k += snprintf(buf + k, (size_t)(size - k), " cst=%02d:%02d:%02d date=%04d-%02d-%02d wday=%d",
                      b->t.hour, b->t.min, b->t.sec, 2000 + b->t.year, b->t.month, b->t.day,
                      b->t.wday);
        if (k >= size)
            return;
    }
    snprintf(buf + k, (size_t)(size - k), " sym=%s", b->last_frame);
}
