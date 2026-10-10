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
DIAL = (239, 224, 168)
DIAL_L = (248, 238, 196)
DIAL_D = (214, 196, 132)
RED = (208, 64, 42)
RED_L = (240, 110, 80)
METAL = (92, 86, 80)
METAL_L = (150, 143, 135)
METAL_D = (60, 55, 51)
CHROME = (222, 222, 226)
CHROME_D = (150, 150, 158)
METER = (244, 238, 220)


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

    # 刻度窗：亮的米黃背光，刻度＋紅指針
    x0, y0, x1, y1 = 38, 40, 78, 58
    d.rectangle([x0, y0, x1, y1], fill=DIAL, outline=OUTL)
    d.rectangle([x0 + 1, y0 + 1, x1 - 1, y0 + 4], fill=DIAL_L)
    d.rectangle([x0 + 1, y1 - 3, x1 - 1, y1 - 1], fill=DIAL_D)
    for i, x in enumerate(range(x0 + 3, x1 - 1, 3)):
        h = 4 if i % 3 == 0 else 2
        d.line([(x, y0 + 7), (x, y0 + 7 + h)], fill=OUTL)
    for x in range(x0 + 4, x1 - 2, 6):            # 第二排刻度（另一個波段）
        d.line([(x, y0 + 13), (x + 2, y0 + 13)], fill=BODY_DD)
    d.line([(61, y0 + 2), (61, y1 - 2)], fill=RED, width=1)
    d.line([(62, y0 + 2), (62, y1 - 2)], fill=RED_L, width=1)

    # S 表：小方窗、弧形刻度、指針
    mx0, my0, mx1, my1 = 38, 62, 55, 77
    d.rectangle([mx0, my0, mx1, my1], fill=METER, outline=OUTL)
    d.arc([mx0 + 2, my0 + 3, mx1 - 2, my1 + 9], 200, 340, fill=OUTL)
    d.arc([mx0 + 2, my0 + 3, mx1 - 2, my1 + 9], 300, 340, fill=RED)
    d.line([(46, 75), (51, 66)], fill=OUTL)
    d.line([(mx0 + 1, my1 - 1), (mx1 - 1, my1 - 1)], fill=BODY_L)

    # 調諧大旋鈕：深色金屬，左上高光
    cx, cy, r = 68, 71, 9
    d.ellipse([cx - r, cy - r + 1, cx + r, cy + r + 1], fill=METAL_D)   # 底下的影子
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=METAL, outline=OUTL)
    d.pieslice([cx - r + 1, cy - r + 1, cx + r - 1, cy + r - 1], 180, 270, fill=METAL_L)
    d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=METAL, outline=METAL_D)
    for a in range(0, 360, 30):                      # 滾花
        import math
        px = cx + round((r - 1) * math.cos(math.radians(a)))
        py = cy + round((r - 1) * math.sin(math.radians(a)))
        im.putpixel((px, py), METAL_D)
    d.line([(cx, cy - 3), (cx, cy - 1)], fill=RED_L)    # 刻線

    # 波段按鍵：一排四顆
    for i in range(4):
        bx = 39 + i * 10
        d.rectangle([bx, 82, bx + 7, 87], fill=BODY_L, outline=OUTL)
        d.line([(bx + 1, 86), (bx + 6, 86)], fill=BODY_D)
    d.rectangle([39 + 1, 83, 39 + 6, 85], fill=RED)   # 按下的那顆

    # 機腳
    for fx in (9, 72):
        d.rectangle([fx, 92, fx + 6, 94], fill=BODY_DD, outline=OUTL)

    # 明暗層次：正面由上往下、頂面由左往右、側面由上往下各走一段漸層，再加塑膠顆粒。
    # RGB565 一階是 8／4／8，所以顆粒要 ±8 才留得下來（ICON-STYLE §2.4：色數要到三位數）
    rnd = random.Random(1987)
    px = im.load()
    body_cols = {BODY, BODY_L, BODY_D, BODY_DD}
    soft = {DIAL, DIAL_L, DIAL_D, METER, GRILLE, METAL, METAL_L}
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
