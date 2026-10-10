"""產生載入器選單的封面：96×96，RGB565 big-endian（18432 bytes）。

風格依 rp2040-retro-handheld/docs/ICON-STYLE.md：具體物件（1980 年代米色短波收音機）、
淺色底、主體填滿、硬邊 pixel art 但有亮／暗面、受光在左上、深褐描邊、一個暖色重點（紅指針）。

    python tools/make_cover.py            -> assets/RetroSDR.ino.RAW 與 assets/cover.png（放大 4 倍預覽）
"""
import os
import random
from PIL import Image, ImageDraw

W = H = 96
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "assets")

BG = (255, 255, 255)                # 跟其他封面一樣純白
OUTL = (58, 48, 44)
BODY = (196, 186, 166)
BODY_L = (220, 211, 192)
BODY_D = (160, 149, 127)
BODY_DD = (133, 122, 102)
GRILLE = (92, 80, 70)
RED = (208, 64, 42)
RED_L = (240, 110, 80)
METAL = (92, 86, 80)
METAL_L = (150, 143, 135)
METAL_D = (60, 55, 51)
CHROME = (222, 222, 226)
CHROME_D = (150, 150, 158)
METER = (244, 238, 220)
SP_BG = (10, 14, 22)                 # 螢幕顏色取自機上畫面
SP_FILL = (16, 70, 92)
SP_TRACE = (90, 220, 250)
WF_SPECK = [(30, 110, 170), (40, 150, 190), (60, 180, 200), (30, 90, 160), (50, 130, 180)]
WF_YEL = (250, 230, 110)
WF_RED = (240, 120, 70)
WF_CURSOR = (255, 80, 40)
WF_HI = (60, 70, 90)


def build():
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)

    # 天線：右後方斜上，三節由粗到細，頂端一顆球
    segs = [((84, 27), (88, 15), 3), ((88, 15), (91, 6), 2), ((91, 6), (93, 1), 1)]
    for (a, b, w) in segs:
        d.line([a, b], fill=OUTL, width=w + 2)
    for (a, b, w) in segs:
        d.line([a, b], fill=CHROME_D, width=w)
        if w > 1:
            d.line([(a[0] - 1, a[1]), (b[0] - 1, b[1])], fill=CHROME, width=1)
    d.ellipse([90, 0, 95, 4], fill=CHROME, outline=OUTL)
    d.ellipse([81, 25, 87, 30], fill=METAL, outline=OUTL)   # 天線座

    # 機身：正面、頂面、右側面（等角感）
    front = [(3, 34), (82, 34), (82, 92), (3, 92)]
    top = [(3, 34), (13, 26), (93, 26), (82, 34)]
    side = [(82, 34), (93, 26), (93, 84), (82, 92)]
    d.polygon(top, fill=BODY_L)
    d.polygon(side, fill=BODY_D)
    d.polygon(front, fill=BODY)
    # 提把（頂面上的一條凹槽）
    d.polygon([(22, 31), (26, 28), (70, 28), (66, 31)], fill=BODY_D)
    d.line([(22, 31), (66, 31)], fill=BODY_DD)
    # 正面倒角：上、左亮，下、右暗
    d.line([(4, 35), (81, 35)], fill=BODY_L)
    d.line([(4, 35), (4, 91)], fill=BODY_L)
    d.line([(4, 91), (81, 91)], fill=BODY_D)
    d.line([(81, 36), (81, 91)], fill=BODY_D)
    # 側面下緣陰影
    d.line([(83, 91), (92, 84)], fill=BODY_DD)
    for poly in (top, side, front):
        d.polygon(poly, outline=OUTL)

    # 喇叭網：左半，橫條
    gx0, gy0, gx1, gy1 = 8, 41, 34, 86
    d.rectangle([gx0, gy0, gx1, gy1], fill=BODY_D, outline=OUTL)
    for y in range(gy0 + 3, gy1 - 1, 3):
        d.line([(gx0 + 2, y), (gx1 - 2, y)], fill=GRILLE)
        d.line([(gx0 + 2, y + 1), (gx1 - 2, y + 1)], fill=BODY_L)
    d.line([(gx0 + 1, gy0 + 1), (gx1 - 1, gy0 + 1)], fill=BODY_DD)   # 內緣陰影（受光在左上）

    # 右半整塊是螢幕，畫得跟機上的畫面一樣（firmware/screen.ppm）：
    # 上半是頻譜（青色軌跡沿著雜訊底線，幾根尖峰，下面填深青），
    # 下半是瀑布圖（藍青色雜訊顆粒，強訊號是往下流的黃／紅細直線，跟上面的尖峰對齊），
    # 紅色調諧游標貫穿兩者：頻譜上實線、瀑布上虛線。
    x0, y0, x1, y1 = 38, 40, 79, 87
    d.rectangle([x0 - 1, y0 - 1, x1 + 1, y1 + 1], fill=BODY_D, outline=OUTL)   # 螢幕框
    d.rectangle([x0, y0, x1, y1], fill=SP_BG, outline=OUTL)
    sx0, sx1 = x0 + 1, x1 - 1
    sy0, sy1 = y0 + 1, y0 + 21                # 頻譜區
    wy0, wy1 = sy1 + 2, y1 - 1                # 瀑布區
    floor = sy1 - 4                           # 雜訊底線
    peaks = {sx0 + 6: (sy0 + 2, WF_YEL), sx0 + 15: (sy0 + 9, WF_YEL),
             sx0 + 30: (sy0 + 5, WF_RED), sx0 + 34: (sy0 + 12, WF_YEL)}
    cursor = sx0 + 15
    wr = random.Random(68500)
    # 頻譜：每欄一個高度，雜訊底線上下抖，尖峰處拉高；軌跡以下填深青
    prev = floor
    for x in range(sx0, sx1 + 1):
        h = peaks[x][0] if x in peaks else floor + wr.choice((-1, 0, 0, 1))
        for y in range(h + 1, sy1 + 1):
            im.putpixel((x, y), SP_FILL)
        lo, hi = min(h, prev), max(h, prev)
        for y in range(lo, hi + 1):           # 跟左鄰連成實線
            im.putpixel((x, y), SP_TRACE)
        prev = h if x not in peaks else floor
    d.line([(sx0, sy1 + 1), (sx1, sy1 + 1)], fill=OUTL)         # 頻譜與瀑布的分隔
    # 瀑布：每個像素是雜訊顆粒，訊號欄是往下流的細直線（偶爾衰落一格）
    for y in range(wy0, wy1 + 1):
        for x in range(sx0, sx1 + 1):
            c = wr.choice(WF_SPECK)
            if x in peaks and wr.random() > 0.08:
                c = peaks[x][1]
            im.putpixel((x, y), c)
    # 游標
    for y in range(sy0, sy1 + 1):
        im.putpixel((cursor, y), WF_CURSOR)
    for y in range(wy0, wy1 + 1):
        if (y - wy0) & 2:
            im.putpixel((cursor, y), WF_CURSOR)
    d.line([(x0 + 1, y0 + 1), (x1 - 1, y0 + 1)], fill=WF_HI)     # 玻璃上緣的反光

    # 機腳
    for fx in (9, 72):
        d.rectangle([fx, 92, fx + 6, 94], fill=BODY_DD, outline=OUTL)

    # 明暗層次：正面由上往下、頂面由左往右、側面由上往下各走一段漸層，再加塑膠顆粒。
    # RGB565 一階是 8／4／8，所以顆粒要 ±8 才留得下來（ICON-STYLE §2.4：色數要到三位數）
    rnd = random.Random(1987)
    px = im.load()
    body_cols = {BODY, BODY_L, BODY_D, BODY_DD}
    soft = {METER, GRILLE, METAL, METAL_L}
    for y in range(H):
        for x in range(W):
            c = px[x, y]
            if c in body_cols:
                if x <= 82 and y >= 34:            # 正面：上亮下暗
                    shade = 10 - (y - 34) * 20 // 58
                elif y < 34:                       # 頂面：左亮右暗
                    shade = 12 - x * 12 // 96
                else:                              # 側面：上亮下暗
                    shade = -2 - (y - 26) * 10 // 66
                n = shade + rnd.randint(-8, 8)
                px[x, y] = tuple(max(0, min(255, v + n)) for v in c)
            elif c in soft:
                n = rnd.randint(-5, 5)
                px[x, y] = tuple(max(0, min(255, v + n)) for v in c)
    return im


def to_rgb565_be(im):
    out = bytearray()
    for (r, g, b) in im.get_flattened_data():
        v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        out += bytes(((v >> 8) & 0xFF, v & 0xFF))
    return bytes(out)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    im = build()
    raw = to_rgb565_be(im)
    assert len(raw) == W * H * 2
    with open(os.path.join(OUT, "RetroSDR.ino.RAW"), "wb") as f:
        f.write(raw)
    im.save(os.path.join(OUT, "cover_source.png"))
    im.resize((W * 4, H * 4), Image.NEAREST).save(os.path.join(OUT, "cover.png"))
    print("wrote assets/RetroSDR.ino.RAW (%d bytes), %d colors" % (len(raw), len(set(im.get_flattened_data()))))
