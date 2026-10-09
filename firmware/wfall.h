/* 瀑布圖：320×96 的 8 位元調色盤索引環形緩衝（30 KB）。
 *
 * 不用 ILI9341 的硬體垂直捲動：那只能沿面板的長軸捲，橫向擺放時長軸是
 * 水平方向，瀑布會變成左右流動（DESIGN.md §7）。所以存在 RAM 裡，每次
 * 重畫時從最新的一列往下送。
 */
#ifndef WFALL_H
#define WFALL_H

#include <stdint.h>

#define WF_W 320
#define WF_H 96

typedef struct {
    uint8_t  px[WF_H][WF_W];
    int      head;                      /* 最新那一列在 px 裡的索引 */
    uint16_t pal[256];                  /* 索引 -> RGB565（原生位元組順序） */
} wfall;

void wfall_init(wfall *wf);

/* 推一列進去。db10 是 WF_W 個 dBFS×10；lo..lo+range 對應到調色盤 0..255。 */
void wfall_push(wfall *wf, const int16_t *db10, int lo_db10, int range_db10);

/* 第 r 列（0 = 最新，在最上面）。 */
static inline const uint8_t *wfall_row(const wfall *wf, int r)
{
    int i = wf->head - r;
    if (i < 0)
        i += WF_H;
    return wf->px[i];
}

#endif /* WFALL_H */
