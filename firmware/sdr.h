/* RetroSDR 的狀態機：樣本塊與按鍵進來，畫面出去。
 *
 * 跟 retro-dict 的 app.c 同一個分工 —— **不知道板子存在**。sketch 負責
 * ADC、鍵盤掃描、SPI；這裡只吃「一塊樣本」與「按鍵事件」，吐出「第 y 列
 * 長什麼樣」。所以整條路徑都能在 PC 上用合成訊號跑過（test_pc.c）。
 */
#ifndef SDR_H
#define SDR_H

#include <stdint.h>

#include "ddc.h"
#include "keys.h"
#include "preset.h"
#include "spectrum.h"
#include "wfall.h"

/* 一塊 ADC 樣本：32768 點 = 65.5 ms，塊與塊之間掃一次鍵盤（DESIGN.md §2.3）。
 * 開頭 SDR_SKIP 點丟掉：GPIO 26 剛從推 CLOCK 切回 ADC，偏壓網路
 * （約 41 kΩ × 1 nF ≈ 41 µs）還在回穩，512 點 ≈ 1 ms 是 25 倍時間常數。 */
#define SDR_BLOCK 32768
#define SDR_SKIP  512

#define UI_W 320
#define UI_H 240

/* 畫面分區（DESIGN.md §7） */
#define UI_Y_STATUS 0
#define UI_Y_SPEC   16
#define UI_H_SPEC   80
#define UI_Y_WF     96               /* 高 WF_H = 96 */
#define UI_Y_INFO   192
#define UI_Y_HINT   224

#define UI_TEXT_COLS (UI_W / 6)
#define UI_MAX_TEXT  32
#define UI_STAT_LINES 11             /* 統計頁的行數：瀑布圖區 96 px / 8 px，留上下邊 */

typedef struct {
    int16_t  x, y;
    uint16_t color;
    char     s[UI_TEXT_COLS + 1];
} ui_text;

#define SDR_VOL_MAX 10

/* 平均的檔位：每塊做幾段 FFT、指數平均的強度 */
#define SDR_AVG_LEVELS 5

typedef struct {
    spectrum sp;
    wfall    wf;

    int32_t tune_hz;                  /* 調諧點（解調與游標都看它） */
    int cursor;                       /* 游標像素 0..319，由 tune_hz 算出 */
    int step_hz;                      /* ←→ 一次調多少 */
    int mode;                         /* DDC_AM / CW / USB / LSB */
    int bw_hz;
    int vol;                          /* 0..SDR_VOL_MAX，0 = 靜音 */
    int eq;                           /* 喇叭音色預設（eq.h），e 輪流切 */
    int ddc_dirty;                    /* 調諧、模式、頻寬改了：平台要轉給 Core 1 */
    int ref_db;                       /* 頻譜頂端，dBFS（整數 dB） */
    int range_db;                     /* 頻譜高度代表幾 dB */
    int avg;                          /* 0..SDR_AVG_LEVELS-1 */
    int peak_on;
    int quiet;                        /* 1 = 暫停 LCD 更新（雜訊診斷用） */
    int tx_on;                        /* 1 = GPIO 0 輸出測試訊號（平台負責實作） */
    char tc[UI_TEXT_COLS + 1];        /* 授時碼狀態列，平台填；空字串 = 不顯示 */
    int  show_stats;                  /* 1 = 瀑布圖的位置改顯示統計（鍵盤 I） */
    char stats[UI_STAT_LINES][UI_TEXT_COLS + 1];   /* 平台填 */
    int  tc_locked;
    int  rec_on;                      /* 1 = 錄音中（鍵盤 R 切換；平台開檔失敗會清回 0） */
    char rec[UI_TEXT_COLS + 1];       /* 錄音狀態列，平台填；空字串 = 不顯示 */

    /* 預設清單（鍵盤 f 選台、F 存台）。按鍵只送出請求：平台第一次要先從
     * SD 卡讀 PRESETS.TXT，存台要寫檔，這些都不在這裡做。 */
    preset_list presets;
    int  preset_idx;                  /* 目前在第幾個，-1 = 還沒選過 */
    int  preset_req;                  /* 1 = 按了 f，平台讀好清單後呼叫 sdr_preset_next() */
    int  preset_save_req;             /* 1 = 按了 F，平台寫檔後呼叫 preset_add() 與 sdr_msg() */
    char msg[UI_TEXT_COLS + 1];       /* 短暫的訊息（選台、存台），幾秒後消失 */
    int  msg_ttl;                     /* 還要顯示幾次重畫 */

    char entry[12];                   /* 頻率輸入中（kHz） */
    int  entry_len;
    int  entering;

    /* 由平台填的統計 */
    uint32_t proc_ms;                 /* 處理一塊花多久 */
    uint32_t scan_us;                 /* 鍵盤掃描的空隙 */
    uint32_t drops;                   /* 處理不及、整塊沒接上的次數 */
    uint32_t blocks;

    /* sdr_prepare() 算好、ui_line() 直接查的東西 */
    uint8_t  trace_row[UI_W];         /* 0..UI_H_SPEC-1，頻譜曲線落在第幾列 */
    uint8_t  peak_row[UI_W];
    uint8_t  grid[UI_H_SPEC];         /* 該列是不是 10 dB 格線（1/0） */
    ui_text  text[UI_MAX_TEXT];
    int      ntext;
} sdr;

void sdr_init(sdr *s);

/* 這一塊要做幾段 FFT（依平均檔位）。 */
int sdr_k(const sdr *s);

/* 一塊樣本進來：頻譜、雜訊底線、瀑布圖往下推一列。 */
void sdr_block(sdr *s, const uint16_t *x, int n, int skip);

/* 處理一個按鍵。回傳 1 = 畫面要重畫。 */
int sdr_key(sdr *s, const key_event *ev);

/* 重畫前呼叫一次：算好每列要用的東西與所有文字。 */
void sdr_prepare(sdr *s);

/* 第 y 列，320 個 RGB565，**big-endian**（ILI9341 的線序，DMA 直接送）。 */
void ui_line(const sdr *s, int y, uint16_t *out);

/* 調諧點所在的 bin。 */
int sdr_cursor_bin(const sdr *s);

/* 切到清單的下一個台（調諧點、模式、頻寬），並顯示訊息。 */
void sdr_preset_next(sdr *s);

/* 在資訊列顯示一段訊息約 3 秒。 */
void sdr_msg(sdr *s, const char *text);

/* 目前的調諧點、模式、頻寬，當成一個預設（存台用；名稱留空）。 */
void sdr_current_preset(const sdr *s, preset *out);

/* 設調諧點（會夾在 0..fs/2 內，並更新游標像素、標記 ddc_dirty）。 */
void sdr_tune(sdr *s, int32_t hz);

#endif /* SDR_H */
