#!/usr/bin/env python3
"""從 arduino-pico 的 memmap_default.ld 生成偏移編譯用的 linker script。

用法:
    python loader_offset/gen_app_ld.py [arduino-pico 平台路徑]

不給路徑時自動找 %LOCALAPPDATA%\\Arduino15\\packages\\rp2040\\hardware\\rp2040\\<版本>。

產出 loader_offset/memmap_app_arduino.ld —— **生成的檔案,不要手改**。
改 arduino-pico 版本就重跑這支腳本。

--------------------------------------------------------------------------
為什麼要用生成的,不手寫
--------------------------------------------------------------------------
rp2040-retro-loader 的 README §3.1 記著一次教訓:憑印象手寫 memmap 的結果是
「編得過但開機掛掉」,而且很難查。arduino-pico 的這份檔案在版本之間一樣會動
(OTA 區、partition table、picolibc/llvm-libc 的 PROVIDE 都是後來才加的)。
所以這裡只做四處定點手術,找不到預期的樣板就直接報錯,不會安靜地生出壞檔案。

--------------------------------------------------------------------------
四處改動
--------------------------------------------------------------------------
① FLASH ORIGIN 0x10000000 -> 0x10004000,LENGTH 扣掉 16KB
   前 16KB 讓給載入器(從 SD 載入時)或跳板(USB 直接燒錄時)。
   LENGTH 扣掉同樣的量,所以 flash 尾端邊界不變 —— EEPROM / FS 區(絕對位址,
   由 __EEPROM_START__ / __FS_START__ 代入)照舊,不會被推出去。

② 丟掉 .boot2
   專題自己那份 boot2 永遠不會被執行:ROM 只認 flash 最前面 256 bytes,而那是
   載入器/跳板的地盤。留著還會佔掉向量表該在的位置。
   SDK 的 crt0.S 有引用 __boot2_entry_point,但在 RP2040 上被
   `#if !PICO_RP2040 && ...` 排除掉(已對 5.6.1 附的 pico-sdk 確認),所以
   discard 是安全的。換 arduino-pico 版本時重新確認:
       grep -n -B4 __boot2_entry_point <平台>/pico-sdk/src/rp2_common/pico_crt0/crt0.S

③ 丟掉 .OTA 與 .partition  ← 這一項是 arduino-pico 特有的,infones 沒遇過
   arduino-pico 的 image 前面不是向量表。預設編譯的實測佈局是:
       0x10000000  .boot2      256 bytes
       0x10000100  .ota        0x27f4  <- ROM/boot2 其實是跳到這裡
       0x100028f4  .partition  0x70c
       0x10003000  .text       <- 向量表在這裡才開始
   也就是說 arduino-pico 一律經過一段 OTA 前導程式才進本體。
   載入器與跳板是直接讀 APP_BASE 的向量表(SP + Reset)然後跳,不認這段前導。
   若只改 ORIGIN 而留著這兩段,向量表會落在 0x10007000,載入器跳到
   0x10004000 只會拿到 OTA blob 的頭幾個 byte 當 SP —— 開機直接死,而且
   症狀跟「沒燒進去」一模一樣。
   本專案不用 OTA、不用 LittleFS(磁碟映像走 SD 卡),所以整段丟掉。
   丟掉之後 0x10004000 第一個 byte 就是向量表,跟 infones 的偏移版同形。

④ 補 /DISCARD/
   ota.o 是被 link 指令寫死拉進來的(platform.txt 的 recipe.c.combine.pattern),
   .boot2 也一樣。輸出段拿掉之後這些輸入段會變成 orphan,ld 會自己找地方塞
   —— 很可能就塞在 .text 前面,等於白改。必須明確 discard。
"""

import os
import re
import sys
from pathlib import Path

BOOT_REGION_SIZE = 0x4000          # 跟 rp2040-retro-loader/common/boot_map.h 一致
APP_BASE = 0x10000000 + BOOT_REGION_SIZE

HERE = Path(__file__).resolve().parent
OUT = HERE / "memmap_app_arduino.ld"


def die(msg):
    print("gen_app_ld: " + msg, file=sys.stderr)
    sys.exit(1)


def find_platform():
    root = Path(os.environ.get("LOCALAPPDATA", "")) / "Arduino15" / "packages" / "rp2040" / "hardware" / "rp2040"
    if not root.is_dir():
        die("找不到 arduino-pico 平台目錄,請把路徑當參數傳進來:\n"
            "    python loader_offset/gen_app_ld.py <平台路徑>")
    versions = sorted(p for p in root.iterdir() if p.is_dir())
    if not versions:
        die(f"{root} 底下沒有任何版本")
    return versions[-1]


def cut_block(text, header, what):
    """砍掉一個 `<header> : { ... } > FLASH` 區塊(含大括號配對)。"""
    m = re.search(r"^[ \t]*" + re.escape(header) + r"[ \t]*:[ \t]*\{", text, re.M)
    if not m:
        die(f"在 memmap_default.ld 裡找不到 {what} 區塊 —— arduino-pico 的版面變了,"
            f"請人工看過再更新這支腳本")
    i = text.index("{", m.start())
    depth = 0
    for j in range(i, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                break
    else:
        die(f"{what} 區塊的大括號沒有配對")
    # 吃掉結尾的 `> FLASH` 與換行
    tail = re.compile(r"[ \t]*(>[ \t]*FLASH)?[ \t]*\r?\n").match(text, j + 1)
    end = tail.end() if tail else j + 1
    return text[:m.start()] + text[end:]


def main():
    platform = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else find_platform()
    src = platform / "lib" / "rp2040" / "memmap_default.ld"
    if not src.is_file():
        die(f"找不到 {src}")

    text = src.read_text(encoding="utf-8")

    # --- ① FLASH ORIGIN / LENGTH -------------------------------------------
    pat = re.compile(r"FLASH\(rx\)[ \t]*:[ \t]*ORIGIN[ \t]*=[ \t]*0x10000000[ \t]*,"
                     r"[ \t]*LENGTH[ \t]*=[ \t]*__FLASH_LENGTH__")
    text, n = pat.subn(
        f"FLASH(rx) : ORIGIN = {APP_BASE:#010x}, "
        f"LENGTH = __FLASH_LENGTH__ - {BOOT_REGION_SIZE // 1024}k",
        text)
    if n != 1:
        die("找不到預期的 FLASH MEMORY 那一行(或找到不只一個)")

    # --- ② 丟掉 .boot2 與它的 ASSERT ---------------------------------------
    text = cut_block(text, ".boot2", ".boot2")
    text, n = re.subn(
        r"[ \t]*ASSERT\(__boot2_end__ - __boot2_start__ == 256,\s*"
        r'"[^"]*"\)\r?\n', "", text)
    if n != 1:
        die("找不到 boot2 的 ASSERT(或找到不只一個)")

    # --- ③ 丟掉 .ota 與 .partition ------------------------------------------
    text = cut_block(text, ".ota", ".ota")
    text = cut_block(text, ".partition", ".partition")

    # --- ④ 補 /DISCARD/ ------------------------------------------------------
    anchor = re.search(r"^[ \t]*\.flash_begin[ \t]*:[ \t]*\{", text, re.M)
    if not anchor:
        die("找不到 .flash_begin,無法決定 /DISCARD/ 的插入點")
    # 注意: 這個檔案會被 arduino-pico 的 tools/simplesub.py 用系統 codepage
    # (在中文 Windows 上是 cp950) 讀進去,所以生成內容必須是純 ASCII。
    # 中文說明留在這支腳本與 README,不要寫進 .ld。
    discard = (
        "    /* Output sections for .boot2 / .OTA were removed (see gen_app_ld.py).\n"
        "       The input sections are still pulled in by the link recipe; without an\n"
        "       explicit discard ld would place them as orphans, most likely right in\n"
        "       front of the vector table. */\n"
        "    /DISCARD/ : {\n"
        "        *(.boot2)\n"
        "        *(.OTA)\n"
        "    }\n\n"
    )
    text = text[:anchor.start()] + discard + text[anchor.start():]

    banner = (
        "/* GENERATED - DO NOT EDIT BY HAND\n"
        " * source:    %s\n"
        " * generator: python loader_offset/gen_app_ld.py\n"
        " * purpose:   rp2040-retro-loader offset build (link at %#010x)\n"
        " */\n" % (src, APP_BASE)
    )
    out_text = banner + text
    if not out_text.isascii():
        die("生成內容含非 ASCII 字元,simplesub.py 會讀不進去")
    OUT.write_text(out_text, encoding="ascii")

    print(f"gen_app_ld: 來源 {src}")
    print(f"gen_app_ld: 產出 {OUT}")
    print(f"gen_app_ld: FLASH ORIGIN = {APP_BASE:#010x}, "
          f"已丟掉 .boot2 / .ota / .partition")


if __name__ == "__main__":
    main()
