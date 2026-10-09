/* 逐列算出畫面。
 *
 * 不留整張 framebuffer：320×240 的 RGB565 要 150 KB，ADC 的雙緩衝已經用掉
 * 128 KB。這裡一次只產生一列（640 bytes），sketch 拿兩條輪流送 DMA ——
 * 送這一列的同時算下一列。
 *
 * 所有會用到除法的東西都在 sdr_prepare() 算好了，這裡只剩查表與比較。
 */
#include "font5x7.h"
#include "sdr.h"

#define RGB565(r, g, b) \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

#define C_BAR    RGB565(16, 24, 48)
#define C_INFO   RGB565(8, 8, 16)
#define C_GRID   RGB565(44, 44, 52)
#define C_FILL   RGB565(0, 56, 84)
#define C_TRACE  RGB565(80, 230, 255)
#define C_PEAK   RGB565(200, 180, 60)
#define C_CURSOR RGB565(255, 80, 40)

static void fill(uint16_t *out, uint16_t c)
{
    for (int x = 0; x < UI_W; x++)
        out[x] = c;
}

static void spec_line(const sdr *s, int r, uint16_t *out)
{
    uint16_t base = s->grid[r] ? C_GRID : 0;

    for (int x = 0; x < UI_W; x++) {
        uint16_t c = base;
        int t = s->trace_row[x];
        int p = x ? s->trace_row[x - 1] : t;
        int lo = t < p ? t : p, hi = t < p ? p : t;

        if ((x & 63) == 0 && x)
            c = C_GRID;                     /* 每 50 kHz 一條直格線 */
        if (r > hi)
            c = C_FILL;                     /* 曲線以下（列號越大 dB 越低） */
        else if (r >= lo)
            c = C_TRACE;                    /* 跟左鄰連成實線，不會斷成點 */
        else if (s->peak_on && r == s->peak_row[x])
            c = C_PEAK;
        if (x == s->cursor)
            c = C_CURSOR;
        out[x] = c;
    }
}

static void wf_line(const sdr *s, int r, uint16_t *out)
{
    const uint8_t *row = wfall_row(&s->wf, r);
    for (int x = 0; x < UI_W; x++)
        out[x] = s->wf.pal[row[x]];
    if (r & 2)                              /* 游標在瀑布上畫虛線，不遮住訊號 */
        out[s->cursor] = C_CURSOR;
}

static void text_line(const sdr *s, int y, uint16_t *out)
{
    for (int i = 0; i < s->ntext; i++) {
        const ui_text *t = &s->text[i];
        int gy = y - t->y;
        if (gy < 0 || gy >= 7)
            continue;
        int x = t->x;
        for (const char *c = t->s; *c; c++, x += FONT_CW) {
            uint8_t bits = font5x7_row(*c, gy);
            for (int b = 0; b < 5; b++) {
                int px = x + b;
                if ((bits & (0x10 >> b)) && px >= 0 && px < UI_W)
                    out[px] = t->color;
            }
        }
    }
}

void ui_line(const sdr *s, int y, uint16_t *out)
{
    if (y < UI_Y_SPEC)
        fill(out, C_BAR);
    else if (y < UI_Y_SPEC + UI_H_SPEC)
        spec_line(s, y - UI_Y_SPEC, out);
    else if (y < UI_Y_WF + WF_H)
        wf_line(s, y - UI_Y_WF, out);
    else if (y < UI_Y_HINT)
        fill(out, C_INFO);
    else
        fill(out, C_BAR);

    text_line(s, y, out);

    /* ILI9341 吃 big-endian；DMA 是逐 byte 送，所以在這裡先翻好。 */
    for (int x = 0; x < UI_W; x++)
        out[x] = (uint16_t)((out[x] >> 8) | (out[x] << 8));
}
