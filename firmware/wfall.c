#include "wfall.h"

#include <string.h>

#define RGB565(r, g, b) \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

/* 黑 -> 藍 -> 青 -> 黃 -> 紅 -> 白。雜訊底線落在藍，訊號一路往暖色走。
 * RGB565 只有 5/6/5 位元，漸層本來就會有色階，用分段線性就夠。 */
static const uint8_t STOPS[][3] = {
    {   0,   0,   0 },
    {   0,   0, 160 },
    {   0, 200, 220 },
    { 240, 230,   0 },
    { 240,  40,   0 },
    { 255, 255, 255 },
};
#define NSTOPS ((int)(sizeof STOPS / sizeof STOPS[0]))

void wfall_init(wfall *wf)
{
    memset(wf->px, 0, sizeof(wf->px));
    wf->head = 0;
    for (int i = 0; i < 256; i++) {
        int seg = i * (NSTOPS - 1) / 256;
        int t = i * (NSTOPS - 1) - seg * 256;     /* 0..255，段內位置 */
        const uint8_t *a = STOPS[seg], *b = STOPS[seg + 1];
        int r = a[0] + (b[0] - a[0]) * t / 256;
        int g = a[1] + (b[1] - a[1]) * t / 256;
        int bl = a[2] + (b[2] - a[2]) * t / 256;
        wf->pal[i] = RGB565(r, g, bl);
    }
}

void wfall_push(wfall *wf, const int16_t *db10, int lo_db10, int range_db10)
{
    if (range_db10 < 1)
        range_db10 = 1;
    wf->head = (wf->head + 1) % WF_H;
    uint8_t *row = wf->px[wf->head];
    for (int x = 0; x < WF_W; x++) {
        int v = (db10[x] - lo_db10) * 255 / range_db10;
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        row[x] = (uint8_t)v;
    }
}
