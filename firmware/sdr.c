#include "sdr.h"
#include "eq.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define RGB565(r, g, b) \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

#define C_WHITE  RGB565(240, 240, 240)
#define C_GRAY   RGB565(140, 140, 150)
#define C_DIM    RGB565(90, 90, 100)
#define C_CYAN   RGB565(80, 230, 255)
#define C_AMBER  RGB565(255, 190, 60)
#define C_GREEN  RGB565(110, 230, 110)
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
    s->step_hz = 100;
    s->mode = DDC_CW;
    s->bw_hz = ddc_default_bw(DDC_CW);
    s->vol = 5;
    s->eq = 2;                        /* SPK+：上機實測機上喇叭最清楚（eq.h） */
    sdr_tune(s, 68500);               /* BPC */
    s->ref_db = -40;                  /* 雜訊底線（約 -90）落在下方 2/3 處 */
    s->range_db = 80;
    s->avg = 2;
    s->peak_on = 1;
    preset_init(&s->presets);
    s->preset_idx = -1;
}

/* 訊息顯示幾次重畫：平常每塊一次（66.5 ms），45 次約 3 秒 */
#define SDR_MSG_TTL 45

void sdr_msg(sdr *s, const char *text)
{
    /* 超出一行的部分截掉（不用 snprintf：gcc 會對刻意的截斷發警告） */
    size_t n = strlen(text);
    if (n > sizeof s->msg - 1)
        n = sizeof s->msg - 1;
    memcpy(s->msg, text, n);
    s->msg[n] = 0;
    s->msg_ttl = SDR_MSG_TTL;
}

void sdr_preset_next(sdr *s)
{
    preset_list *l = &s->presets;
    if (l->n <= 0)
        return;
    s->preset_idx = (s->preset_idx + 1) % l->n;
    const preset *p = &l->p[s->preset_idx];
    s->mode = p->mode;
    s->bw_hz = p->bw_hz;
    sdr_tune(s, p->hz);               /* 也會標記 ddc_dirty */
    char lab[48], b[UI_TEXT_COLS + 1 + 48];  /* 過長的部分由 sdr_msg 截掉 */
    preset_label(p, lab, sizeof lab);
    snprintf(b, sizeof b, "PRESET %d/%d  %s", s->preset_idx + 1, l->n, lab);
    sdr_msg(s, b);
}

void sdr_current_preset(const sdr *s, preset *out)
{
    memset(out, 0, sizeof(*out));
    out->hz = s->tune_hz;
    out->mode = s->mode;
    out->bw_hz = s->bw_hz;
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
    sdr_band_meter(s);
    s->blocks++;
}

/* ---- S 表 ---------------------------------------------------------------- */

/* 10^(d/100)，d 是 dB×10。查表：小數部分（0.0–9.9 dB）一張、十進位一張，
 * 不呼叫 powf（M0+ 軟體浮點，通帶 66 個 bin 會花掉 Core 0 不少時間）。 */
static float lin10(int d)
{
    static float frac[100];
    static int ready;
    if (!ready) {
        for (int i = 0; i < 100; i++)
            frac[i] = powf(10.0f, i / 100.0f);
        ready = 1;
    }
    int n = d >= 0 ? d / 100 : -((-d + 99) / 100);
    float v = frac[d - n * 100];
    for (; n > 0; n--) v *= 10.0f;
    for (; n < 0; n++) v *= 0.1f;
    return v;
}

static int hz_bin(int32_t hz)
{
    int b = (int)(((int64_t)hz * SP_N + SP_FS / 2) / SP_FS);
    return b < 0 ? 0 : b > SP_BINS - 1 ? SP_BINS - 1 : b;
}

void sdr_band_meter(sdr *s)
{
    /* 解調通帶（跟 ddc_set 一致）：AM／CW 以調諧點為中心，USB／LSB 從 ±150 Hz 起 */
    int32_t lo, hi;
    switch (s->mode) {
    case DDC_USB: lo = s->tune_hz + 150; hi = lo + s->bw_hz; break;
    case DDC_LSB: hi = s->tune_hz - 150; lo = hi - s->bw_hz; break;
    default:      lo = s->tune_hz - s->bw_hz / 2; hi = s->tune_hz + s->bw_hz / 2; break;
    }
    int p0 = hz_bin(lo), p1 = hz_bin(hi);
    if (p0 < SP_NF_LO) p0 = SP_NF_LO;
    if (p1 < p0) p1 = p0;
    int np = p1 - p0 + 1;

    /* 兩側各隔一格，各取 max(通帶寬, 16 格 ≈ 2 kHz)；中位數不怕旁邊有一兩個台 */
    int w = np > 16 ? np : 16;
    int16_t adj[256];                     /* 兩側各最多 128 格（通帶 16 kHz）；放不下的就不取 */
    int na = 0;
    for (int b = p0 - 2 - w + 1; b <= p0 - 2; b++)
        if (b >= SP_NF_LO && na < (int)(sizeof adj / sizeof adj[0])) adj[na++] = s->sp.bin_db[b];
    for (int b = p1 + 2; b < p1 + 2 + w; b++)
        if (b < SP_BINS && na < (int)(sizeof adj / sizeof adj[0])) adj[na++] = s->sp.bin_db[b];
    if (na < 4) {
        s->band_sn = 0;
        return;
    }
    for (int i = 1; i < na; i++) {        /* 插入排序，最多幾百個 */
        int16_t v = adj[i];
        int j = i - 1;
        while (j >= 0 && adj[j] > v) { adj[j + 1] = adj[j]; j--; }
        adj[j + 1] = v;
    }
    int med = adj[na / 2];

    /* 通帶內功率相對中位數加總（線性），再除以 np = 相對同頻寬的雜訊 */
    float sum = 0;
    for (int b = p0; b <= p1; b++)
        sum += lin10(s->sp.bin_db[b] - med);
    float r = sum / (float)np;
    /* 單塊的起伏：CW 500（約 5 格）標準差 1.7 dB、AM 8k 0.5 dB（PC 量的，平均無偏）。
     * 平均 1/4 讓長條不跳，換了通帶就從這一塊重來 */
    if (p0 != s->band_p0 || p1 != s->band_p1 || s->band_avg <= 0) {
        s->band_p0 = p0;
        s->band_p1 = p1;
        s->band_avg = r;
    } else {
        s->band_avg += (r - s->band_avg) * 0.25f;
    }
    r = s->band_avg;
    int sn = (int)lroundf(100.0f * log10f(r > 1e-6f ? r : 1e-6f));
    s->band_sn = (int16_t)sn;
    s->band_db = (int16_t)(med + (int)lroundf(100.0f * log10f(sum > 1e-6f ? sum : 1e-6f)));
}

int sdr_cursor_bin(const sdr *s)
{
    int b = (int)(((int64_t)s->tune_hz * SP_N + SP_FS / 2) / SP_FS);
    return b > SP_BINS - 1 ? SP_BINS - 1 : b;
}

void sdr_tune(sdr *s, int32_t hz)
{
    if (hz < 0) hz = 0;
    if (hz > SP_FS / 2 - 1) hz = SP_FS / 2 - 1;
    s->tune_hz = hz;
    s->cursor = (int)((int64_t)hz * UI_W / (SP_FS / 2));
    s->ddc_dirty = 1;
}

/* ---- 按鍵 ---------------------------------------------------------------- */

static const int STEPS[] = { 10, 100, 1000, 10000 };

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
        if (hz >= 0 && hz < SP_FS / 2)
            sdr_tune(s, hz);
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
    if (c == 'H') { sdr_tune(s, s->tune_hz - 10 * s->step_hz); return 1; }
    if (c == 'L') { sdr_tune(s, s->tune_hz + 10 * s->step_hz); return 1; }
    if (c == 'F') { s->preset_save_req = 1; return 1; }
    if (c >= 'A' && c <= 'Z')
        c = (uint8_t)(c - 'A' + 'a');

    switch (c) {
    case KEY_LEFT:  case 'h': sdr_tune(s, s->tune_hz - s->step_hz); return 1;
    case KEY_RIGHT: case 'l': sdr_tune(s, s->tune_hz + s->step_hz); return 1;
    case 's': {
        int i = 0;
        while (STEPS[i] != s->step_hz && i < 3) i++;
        s->step_hz = STEPS[(i + 1) % 4];
        /* 換步進時把調諧點對齊到新步進，之後的數字才整齊 */
        if (s->step_hz > 10)
            sdr_tune(s, (s->tune_hz + s->step_hz / 2) / s->step_hz * s->step_hz);
        return 1;
    }
    case 'm':
        s->mode = (s->mode + 1) % DDC_NMODES;
        s->bw_hz = ddc_default_bw(s->mode);
        s->ddc_dirty = 1;
        return 1;
    case 'b':
        s->bw_hz = ddc_next_bw(s->mode, s->bw_hz);
        s->ddc_dirty = 1;
        return 1;
    case KEY_PGUP: case '=':
        if (s->vol < SDR_VOL_MAX) s->vol++;
        return 1;
    case KEY_PGDN: case '-':
        if (s->vol > 0) s->vol--;
        return 1;
    case KEY_UP:    case 'k':
        if (s->ref_db < 0) s->ref_db += 5;
        return 1;
    case KEY_DOWN:  case 'j':
        if (s->ref_db > -140) s->ref_db -= 5;
        return 1;
    case ']':
        if (s->range_db < 120) s->range_db += 20;
        return 1;
    case '[':
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
    case 't':
        s->tx_on = !s->tx_on;
        return 1;
    case 'i':
        s->show_stats = !s->show_stats;
        return 1;
    case 'r':
        s->rec_on = !s->rec_on;
        return 1;
    case 'f':
        s->preset_req = 1;
        return 1;
    case 'e': {
        static const char *const DESC[EQ_NPRESETS] = {
            "EQ FLAT  (no filter)",
            "EQ SPK   HP 300 Hz + 2 kHz +6 dB",
            "EQ SPK+  HP 400 Hz + 2.5 kHz +9 dB",
            "EQ VOICE HP 300 + 2 kHz +6 dB + LP 3.2 kHz",
        };
        s->eq = (s->eq + 1) % EQ_NPRESETS;
        sdr_msg(s, DESC[s->eq]);
        return 1;
    }
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

    /* 狀態列：收什麼、怎麼收 */
    {
        int32_t hz = s->tune_hz;
        sprintf(b, "%3s %3ld.%03ld kHz", ddc_mode_name(s->mode),
                (long)(hz / 1000), (long)(hz % 1000));
        text(s, 4, 4, C_CYAN, b);
        sprintf(b, "BW %d  STEP %d  VOL %d", s->bw_hz, s->step_hz, s->vol);
        text(s, 112, 4, C_GRAY, b);
    }
    if (s->tx_on)
        text(s, UI_W - 6 * 8 - 4, 4, C_RED, "TX");
    /* 右上角只有一格：錄音 > QUIET > PK。左邊的 BW/STEP/VOL 最長到 x = 268 */
    if (s->rec_on)
        text(s, UI_W - 6 * 5 - 4, 4, C_RED, "  REC");
    else if (s->quiet)
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
        /* 調諧點 ±1 bin 的最大值：載波不一定剛好落在 bin 中央 */
        int cb = sdr_cursor_bin(s), lv = s->sp.bin_db[cb];
        if (cb > 0 && s->sp.bin_db[cb - 1] > lv) lv = s->sp.bin_db[cb - 1];
        if (cb < SP_BINS - 1 && s->sp.bin_db[cb + 1] > lv) lv = s->sp.bin_db[cb + 1];
        fmt_db10(d1, lv);
        fmt_db10(d2, lv - s->sp.nf);
        sprintf(b, "SIG %s  S/N %s", d1, d2);
        text(s, 4, UI_Y_INFO + 4, C_CYAN, b);
        /* S 表：通帶內 S/N（數字＋下面的長條） */
        fmt_db10(d1, s->band_sn);
        sprintf(b, "BAND %s dB", d1);
        text(s, 4 + 23 * 6, UI_Y_INFO + 4, C_GREEN, b);
        int px = s->band_sn * (UI_W - 8) / (SDR_METER_DB * 10);
        s->meter_px = (int16_t)(px < 0 ? 0 : px > UI_W - 8 ? UI_W - 8 : px);
        fmt_db10(d1, s->sp.nf);
        sprintf(b, "NF %s", d1);
        text(s, UI_W - (int)strlen(b) * 6 - 4, UI_Y_INFO + 4, C_AMBER, b);
    }

    /* 資訊列 2：輸入中就顯示輸入，否則是處理統計 */
    if (s->entering) {
        sprintf(b, "FREQ> %s_ kHz    ENTER=GO  ESC=CANCEL", s->entry);
        text(s, 4, UI_Y_INFO + 18, C_AMBER, b);
    } else if (s->msg_ttl > 0) {
        /* 選台、存台的訊息：幾秒就消失，所以比錄音狀態還優先 */
        text(s, 4, UI_Y_INFO + 18, C_AMBER, s->msg);
        s->msg_ttl--;
    } else if (s->rec[0]) {
        /* 錄音中（或剛停）：檔名、秒數、掉塊 —— 比授時碼優先 */
        text(s, 4, UI_Y_INFO + 18, s->rec_on ? C_RED : C_AMBER, s->rec);
    } else if (s->tc[0]) {
        /* 調在授時台上：顯示解碼。鎖定前是琥珀色，鎖定後是青色 */
        text(s, 4, UI_Y_INFO + 18, s->tc_locked ? C_CYAN : C_AMBER, s->tc);
    } else {
        sprintf(b, "REF %d RNG %d AVG %s  PROC %lums DROP %lu",
                s->ref_db, s->range_db, AVG[s->avg].name,
                (unsigned long)s->proc_ms, (unsigned long)s->drops);
        text(s, 4, UI_Y_INFO + 18, C_GRAY, b);
    }

    /* 統計頁：蓋在瀑布圖的位置。不插 USB（用電池跑、收訊最乾淨）時看數字用 */
    if (s->show_stats)
        for (int i = 0; i < UI_STAT_LINES; i++)
            if (s->stats[i][0])
                text(s, 4, UI_Y_WF + 4 + i * 8, i == 0 ? C_WHITE : C_GRAY, s->stats[i]);

    /* 按鍵提示 */
    text(s, 4, UI_Y_HINT + 1, C_DIM,
         "PAD <>TUNE ^vREF  A/B VOL  SEL MODE  START STEP");
    text(s, 4, UI_Y_HINT + 9, C_DIM,
         "KBD 0-9.ENTER FREQ M B S -/= VOL [ ] A P Q T I R F");
}
