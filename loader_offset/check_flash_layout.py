#!/usr/bin/env python3
"""偏移編譯的 build 期檢查 —— 擋下「編得過但開機是黑畫面」那一類問題。

用法:
    python loader_offset/check_flash_layout.py <偏移版 .uf2> [--flash-length N]

--flash-length 是板子給的可用 flash 長度(rpipico 2MB 無 FS 是 2093056),
也就是 image 尾端不能越過的線;越過去就撞上 EEPROM / LittleFS 區。

--------------------------------------------------------------------------
為什麼要有這支
--------------------------------------------------------------------------
rp2040-retro-loader README §3.4 第 ④ 項:偏移之後的佈局錯誤不會在編譯時報錯,
只會在實機上變成黑畫面。infones 就是這樣踩掉一輪(它的 ROM 區沒跟著位移)。

PicoApple2 沒有寫死的 flash 位址(磁碟映像走 SD 卡,不佔 flash),所以這裡真正
要守的不是 infones 那種「資料區重疊」,而是 arduino-pico 特有的三件事:

  ① 向量表必須正好在 APP_BASE。arduino-pico 預設的 image 前面壓著 .ota 與
     .partition,向量表其實在 +0x3000。少丟一段,載入器跳過去拿到的就是
     OTA blob 的頭幾個 byte 當 SP —— 症狀跟「根本沒燒進去」一模一樣。
  ② 向量表的 SP / Reset 要像真的。載入器的 app_present() 會驗這兩個值,
     驗不過它會拒絕交棒(跳板則會退回 BOOTSEL)。這裡先用同一組條件驗一次。
  ③ image 尾端不能越過 flash 可用長度。

順便驗 UF2 是連續的 —— README §3.5 坑 3:位址不連續的 UF2 拖曳燒錄會安靜截斷。
"""

import struct
import sys
from pathlib import Path

BOOT_REGION_SIZE = 0x4000
APP_BASE = 0x10000000 + BOOT_REGION_SIZE
FLASH_BASE = 0x10000000
SRAM_BASE = 0x20000000
SRAM_END = 0x20042000          # RP2040: 264KB(含 SCRATCH_X/Y)

UF2_MAGIC0 = 0x0A324655
UF2_MAGIC1 = 0x9E5D5157


def die(msg):
    print("\n  flash 佈局檢查失敗:\n\n" + msg + "\n", file=sys.stderr)
    sys.exit(1)


def read_uf2(path):
    data = Path(path).read_bytes()
    if len(data) % 512:
        die(f"{path} 不是合法的 UF2(長度不是 512 的倍數)")
    blocks = []
    for off in range(0, len(data), 512):
        m0, m1, _flags, addr, size = struct.unpack_from("<IIIII", data, off)
        if m0 != UF2_MAGIC0 or m1 != UF2_MAGIC1:
            die(f"{path} 第 {off // 512} 塊的 UF2 magic 不對")
        blocks.append((addr, size, data[off + 32:off + 32 + size]))
    if not blocks:
        die(f"{path} 是空的")
    return blocks


def main():
    args = sys.argv[1:]
    flash_length = 2093056
    if "--flash-length" in args:
        i = args.index("--flash-length")
        flash_length = int(args[i + 1], 0)
        del args[i:i + 2]
    if len(args) != 1:
        print(__doc__)
        sys.exit(2)

    blocks = read_uf2(args[0])
    blocks.sort(key=lambda b: b[0])
    start = blocks[0][0]
    end = blocks[-1][0] + blocks[-1][1]
    limit = FLASH_BASE + flash_length

    # --- ① 向量表在 APP_BASE ------------------------------------------------
    if start != APP_BASE:
        die(f"  image 起點是 {start:#010x},應該是 {APP_BASE:#010x}。\n\n"
            f"  最可能的原因:linker script 沒換成偏移版,或是 .ota / .partition\n"
            f"  沒有被丟掉(arduino-pico 預設會在向量表前面塞 0x3000 bytes)。\n"
            f"  重跑 python loader_offset/gen_app_ld.py,並確認 build 用的是它。")

    # --- 連續性 --------------------------------------------------------------
    cursor = start
    for addr, size, _ in blocks:
        if addr != cursor:
            die(f"  UF2 位址不連續:{cursor:#010x} 之後跳到 {addr:#010x}。\n\n"
                f"  不連續的 UF2 拖曳燒錄會安靜截斷(loader README 3.5 坑 3)。")
        cursor += size

    # --- ② 向量表內容像不像真的(跟 app_present() 同一組條件) ----------------
    head = blocks[0][2]
    if len(head) < 8:
        die("  第一塊 UF2 不到 8 bytes,讀不到向量表")
    sp, entry = struct.unpack_from("<II", head, 0)
    if not (SRAM_BASE < sp <= SRAM_END):
        die(f"  向量表的初始 SP = {sp:#010x},不在 SRAM 範圍內。\n"
            f"  載入器的 app_present() 會拒絕交棒。")
    if not (APP_BASE <= (entry & ~1) < limit) or not (entry & 1):
        die(f"  向量表的 Reset 向量 = {entry:#010x},不在 APP 區或沒有 Thumb bit。\n"
            f"  載入器的 app_present() 會拒絕交棒。")

    # --- ③ 尾端不越線 --------------------------------------------------------
    headroom = limit - end
    if headroom < 0:
        die(f"  image 尾端 {end:#010x} 越過 flash 可用上限 {limit:#010x},"
            f"超出 {-headroom} bytes。\n\n"
            f"  再往後就是 EEPROM / LittleFS 區。請縮小 image,或在 arduino-cli\n"
            f"  的 FQBN 上改用不同的 flash 切分。")

    print(f"flash 佈局 OK: image {start:#010x}..{end:#010x} "
          f"({end - start} bytes), 上限 {limit:#010x}, 餘裕 {headroom} bytes")
    print(f"              向量表 SP={sp:#010x} Reset={entry:#010x} "
          f"(app_present() 的條件都通過)")


if __name__ == "__main__":
    main()
