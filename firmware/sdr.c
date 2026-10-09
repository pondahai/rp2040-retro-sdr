#include "sdr.h"

#include <stdio.h>
#include <string.h>

#define RGB565(r, g, b) \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

#define C_WHITE  RGB565(240, 240, 240)
#define C_GRAY   RGB565(140, 140, 150)
#define C_DIM    RGB565(90, 90, 100)
#define C_CYAN   RGB565(80, 230, 255)
#define C_AMBER  RGB565(255, 190, 60)
#define C_RED    RGB565(255, 80, 60)

/* 每塊的樣本數扣掉 skip 之後，最多切得出 7 段 4096（見 RetroSDR.ino 的
 * SDR_BLOCK 與 SDR_SKIP）。K 越大越平滑，但處理時間也跟著線性增加 ——
 * 超過一塊的時間（65 ms）就會開始掉塊，畫面上的 DROP 會往上跳。 */
static const struct { uint8_t k, ema; const char *name; } AVG[SDR_AVG_LEVELS] = {
    { 1, 0, "1" },
    { 2, 0, "2" },
    { 4, 0, "4" },
    { 4, 2, "4E" },
    { 7, 3, "7E" },
};

void sdr_init(sdr *s)
{
    memset(s, 0, sizeof(*s));
    spectrum_init(&s->sp);
    wfall_init(&s->wf);
    s->cursor = 88;                   /* 約 68.75 kHz，BPC 附近 */
    s->ref_db = -40;                  /* 雜訊底線（約 -90）落在下方 2/3 處 */
    s->range_db = 80;
    s->avg = 2;
    s->peak_on = 1;
}

int sdr_k(const sdr *s)
{
    return AVG[s->avg].k;
}

void sdr_block(sdr *s, const uint16_t *x, int n, int skip)
{
    s->sp.ema_shift = AVG[s->avg].ema;
    if (spectrum_block(&s->sp, x, n, skip, AVG[s->avg].k) < 1)
        return;
    wfall_push(&s->wf, s->sp.raw, (s->ref_db - s->range_db) * 10,
               s->range_db * 10);
    s->blocks++;
}

int sdr_cursor_bin(const sdr *s)
{
    int b0, b1, best;
    spectrum_px_bins(s->cursor, &b0, &b1);
    best = b0;
    for (int b = b0 + 1; b < b1; b++)
        if (s->sp.bin_db[b] > s->sp.bin_db[best])
            best = b;
    return best;
}

/* ---- 按鍵 ---------------------------------------------------------------- */

static void cursor_move(sdr *s, int d)
{
    s->cursor += d;
    if (s->cursor < 0) s->cursor = 0;
    if (s->cursor > UI_W - 1) s->cursor = UI_W - 1;
}

/* "68.5" -> 68500 Hz。只接受數字與一個小數點，最多三位小數。 */
static int32_t parse_khz(const char *e)
{
    int32_t ip = 0, fp = 0, scale = 1000;
    int dot = 0;
    if (!*e)
        return -1;
    for (; *e; e++) {
        if (*e == '.') {
            if (dot) return -1;
            dot = 1;
        } else if (!dot) {
            if (ip > 1000) return -1;     /* 早就超過 250 kHz，免得溢位 */
            ip = ip * 10 + (*e - '0');
        } else {
            scale /= 10;
            if (scale == 0) return -1;    /* 小數超過三位 = 比 1 Hz 還細 */
            fp += (*e - '0') * scale;
        }
    }
    return ip * 1000 + fp;
}

static int entry_key(sdr *s, uint8_t c)
{
    if ((c >= '0' && c <= '9') || c == '.') {
        if (s->entry_len < (int)sizeof(s->entry) - 1) {
            s->entry[s->entry_len++] = (char)c;
            s->entry[s->entry_len] = 0;
        }
        s->entering = 1;
        return 1;
    }
    if (!s->entering)
        return 0;
    if (c == KEY_BS || c == KEY_DEL) {
        if (s->entry_len)
            s->entry[--s->entry_len] = 0;
        if (!s->entry_len)
            s->entering = 0;
        return 1;
    }
    if (c == KEY_ESC) {
        s->entering = 0;
        s->entry_len = 0;
        s->entry[0] = 0;
        return 1;
    }
    if (c == KEY_ENTER) {
        int32_t hz = parse_khz(s->entry);
        if (hz >= 0 && hz < SP_FS / 2) {
            /* 像素 p 涵蓋 [p, p+1) × 781.25 Hz */
            s->cursor = (int)((int64_t)hz * UI_W / (SP_FS / 2));
            cursor_move(s, 0);
        }
        s->entering = 0;
        s->entry_len = 0;
        s->entry[0] = 0;
        return 1;
    }
    return 1;                         /* 輸入中，其他鍵一律吃掉 */
}

int sdr_key(sdr *s, const key_event *ev)
{
    uint8_t c = ev->code;

    if (entry_key(s, c))
        return 1;

    /* 大寫 H / L 是大步（Shift），其餘指令不分大小寫（CapsLock 開著也能用） */
    if (c == 'H') { cursor_move(s, -10); return 1; }
    if (c == 'L') { cursor_move(s, 10);  return 1; }
    if (c >= 'A' && c <= 'Z')
        c = (uint8_t)(c - 'A' + 'a');

    switch (c) {
    case KEY_LEFT:  case 'h': cursor_move(s, -1); return 1;
    case KEY_RIGHT: case 'l': cursor_move(s, 1);  return 1;
    case KEY_UP:    case 'k':
        if (s->ref_db < 0) s->ref_db += 5;
        return 1;
    case KEY_DOWN:  case 'j':
        if (s->ref_db > -140) s->ref_db -= 5;
        return 1;
    case KEY_PGUP:  case ']':
        if (s->range_db < 120) s->range_db += 20;
        return 1;
    case KEY_PGDN:  case '[':
        if (s->range_db > 40) s->range_db -= 20;
        return 1;
    case 'a':
        s->avg = (s->avg + 1) % SDR_AVG_LEVELS;
        return 1;
    case 'p':
        s->peak_on = !s->peak_on;
        spectrum_peak_reset(&s->sp);
        return 1;
    case 'q':
        s->quiet = !s->quiet;
        return 1;
    }
    return 0;
}

/* ---- 畫面準備 ------------------------------------------------------------ */

/* dB×10 -> "-63.2"。不用 %f：M0+ 上的浮點 printf 又大又慢。 */
static void fmt_db10(char *b, int v)
{
    int neg = v < 0;
    if (neg) v = -v;
    sprintf(b, "%s%d.%d", neg ? "-" : "", v / 10, v % 10);
}

/* Hz -> "68.48"（kHz，兩位小數，四捨五入） */
static void fmt_khz(char *b, int32_t hz)
{
    int32_t c = (hz + 5) / 10;        /* 0.01 kHz 為單位 */
    sprintf(b, "%ld.%02ld", (long)(c / 100), (long)(c % 100));
}

static void text(sdr *s, int x, int y, uint16_t color, const char *str)
{
    if (s->ntext >= UI_MAX_TEXT)
        return;
    ui_text *t = &s->text[s->ntext++];
    t->x = (int16_t)x;
    t->y = (int16_t)y;
    t->color = color;
    /* 超出螢幕寬的部分截掉 —— 畫出去也看不到 */
    size_t n = strlen(str);
    if (n > UI_TEXT_COLS)
        n = UI_TEXT_COLS;
    memcpy(t->s, str, n);
    t->s[n] = 0;
}

static int floordiv(int a, int b)
{
    return a >= 0 ? a / b : -((-a + b - 1) / b);
}

static uint8_t db_row(const sdr *s, int db10)
{
    int r = (s->ref_db * 10 - db10) * UI_H_SPEC / (s->range_db * 10);
    if (r < 0) r = 0;
    if (r > UI_H_SPEC - 1) r = UI_H_SPEC - 1;
    return (uint8_t)r;
}

void sdr_prepare(sdr *s)
{
    char b[64], d1[16], d2[16];
    int x;

    for (x = 0; x < UI_W; x++) {
        s->trace_row[x] = db_row(s, s->sp.trace[x]);
        s->peak_row[x] = db_row(s, s->sp.peak[x]);
    }
    for (int r = 0; r < UI_H_SPEC; r++) {
        int top = s->ref_db * 10 - r * s->range_db * 10 / UI_H_SPEC;
        int bot = s->ref_db * 10 - (r + 1) * s->range_db * 10 / UI_H_SPEC;
        s->grid[r] = (uint8_t)(floordiv(top, 100) != floordiv(bot, 100));
    }

    s->ntext = 0;

    /* 狀態列 */
    text(s, 4, 4, C_WHITE, "RETRO-SDR");
    sprintf(b, "0-250kHz  REF %d  RNG %d  AVG %s",
            s->ref_db, s->range_db, AVG[s->avg].name);
    text(s, 70, 4, C_GRAY, b);
    if (s->quiet)
        text(s, UI_W - 6 * 5 - 4, 4, C_RED, "QUIET");
    else if (s->peak_on)
        text(s, UI_W - 6 * 2 - 4, 4, C_AMBER, "PK");

    /* 頻譜上的刻度 */
    sprintf(b, "%d", s->ref_db);
    text(s, 2, UI_Y_SPEC + 2, C_DIM, b);
    sprintf(b, "%d", s->ref_db - s->range_db);
    text(s, 2, UI_Y_SPEC + UI_H_SPEC - 19, C_DIM, b);
    text(s, 2, UI_Y_SPEC + UI_H_SPEC - 9, C_DIM, "0");
    for (int k = 1; k <= 4; k++) {
        sprintf(b, "%dk", k * 50);
        text(s, k * 64 - (int)strlen(b) * 3, UI_Y_SPEC + UI_H_SPEC - 9, C_DIM, b);
    }
    text(s, UI_W - 6 * 4 - 1, UI_Y_SPEC + UI_H_SPEC - 9, C_DIM, "250k");

    /* 資訊列 1：游標與雜訊底線 */
    {
        int cb = sdr_cursor_bin(s);
        fmt_khz(d1, spectrum_bin_hz(cb));
        fmt_db10(d2, s->sp.bin_db[cb]);
        sprintf(b, "CUR %s kHz  %s dBFS", d1, d2);
        text(s, 4, UI_Y_INFO + 4, C_CYAN, b);
        fmt_db10(d1, s->sp.nf);
        sprintf(b, "NF %s", d1);
        text(s, UI_W - (int)strlen(b) * 6 - 4, UI_Y_INFO + 4, C_AMBER, b);
    }

    /* 資訊列 2：輸入中就顯示輸入，否則是處理統計 */
    if (s->entering) {
        sprintf(b, "FREQ> %s_ kHz    ENTER=GO  ESC=CANCEL", s->entry);
        text(s, 4, UI_Y_INFO + 18, C_AMBER, b);
    } else {
        /* Hann 窗的等效雜訊頻寬是 1.5 個 bin：122 × 1.5 ≈ 183 Hz */
        sprintf(b, "RBW 183Hz %dx4096  PROC %lums  SCAN %luus  DROP %lu",
                s->sp.k_used, (unsigned long)s->proc_ms,
                (unsigned long)s->scan_us, (unsigned long)s->drops);
        text(s, 4, UI_Y_INFO + 18, C_GRAY, b);
    }

    /* 按鍵提示 */
    text(s, 4, UI_Y_HINT + 1, C_DIM,
         "PAD <>CURSOR ^vREF  A/B RANGE  SEL AVG  START PEAK");
    text(s, 4, UI_Y_HINT + 9, C_DIM,
         "KBD 0-9 . ENTER=FREQ  H/L J/K  [ ] A P  Q=QUIET");
}
