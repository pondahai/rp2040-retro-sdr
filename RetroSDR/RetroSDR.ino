// ============================================================================
// RetroSDR —— RP2040 掌機上的 LF 直接取樣 SDR（M1：寬頻頻譜＋瀑布圖）
//
// 這支 sketch 只做三件事：
//
//   1. GPIO 26 分時：ADC 抓一整塊 -> 切回 GPIO 掃鍵盤 -> 再切回 ADC
//   2. 抓好的那一塊交給 sdr.c（FFT、雜訊底線、瀑布圖）
//   3. 逐列向 ui.c 要畫面，DMA 送上 ILI9341
//
// **DSP 與畫面邏輯一行都不在這裡。** fft.c、spectrum.c、wfall.c、sdr.c、
// ui.c 都是純 C，在 PC 上用合成訊號跑過（firmware/test_pc.c）。
//
// 硬體前提（DESIGN.md §1.1）：GPIO 26 上要有偏壓網路
//
//     3V3 ──[330k]──┬──[47k]── GND        偏壓 ≈ 0.41 V
//                   │
//     天線 ──||──────┴── GPIO 26（ADC0 ／ 74HC CLOCK）
//           1nF
//
// 沒接也能跑，只是看到的是浮接的雜訊。
//
// 接腳沿用生態系共用骨架（rp2040-retro-handheld/docs/HARDWARE.md）。
// 授權 GPL-3.0。TFT_DMA.cpp/.h 與面板初始化序列取自 rp2040-retro-dict
// （源自 PicoApple2，同為 GPL-3.0）。
// ============================================================================

#include <Arduino.h>

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "TFT_DMA.h"

extern "C" {
#include "src/rs_sdr.h"
}

// ---- 顯示器（spi0）----
#define PIN_DISPLAY_SCK  18
#define PIN_DISPLAY_MOSI 19
#define PIN_DISPLAY_CS   17
#define PIN_DISPLAY_DC   20
#define PIN_DISPLAY_RST  21
#define PIN_DISPLAY_BL   22

// ---- 鍵盤矩陣 ----
// CLOCK 同時是天線輸入（ADC0）。LATCH 是 595 的 RCLK 與 165 的 SH/LD 共用
// —— 從 PicoApple2 的 scan_matrix() 時序看得出來：拉高鎖存 595，拉低 1 µs
// 讓 165 載入，再拉高。閒置時停在高，取樣期間不動它，鍵盤 LED 就不會閃。
#define DATA_OUT_PIN     15
#define LATCH_PIN        14
#define CLOCK_PIN        26
#define DATA_IN_PIN      27

// ---- 遊戲按鍵（active-low、內部上拉）----
#define PIN_BTN_UP       9
#define PIN_BTN_DOWN     5
#define PIN_BTN_LEFT     8
#define PIN_BTN_RIGHT    6
#define PIN_BTN_A        2
#define PIN_BTN_B        3
#define PIN_BTN_SELECT   28
#define PIN_BTN_START    4

static TFT_DMA tft(PIN_DISPLAY_CS, PIN_DISPLAY_DC, PIN_DISPLAY_RST,
                   PIN_DISPLAY_MOSI, PIN_DISPLAY_SCK);

static sdr      g_sdr;
static keys     g_keys;
static uint16_t g_buf[2][SDR_BLOCK];       // 2 × 64 KB，ADC 雙緩衝
static uint16_t g_line[2][UI_W];           // 一列算、一列送
static int      g_cur;                      // DMA 正在寫哪一塊

// ============================================================================
// ADC：500 ksps，DMA 一次抓 SDR_BLOCK 點
//
// clk_adc 48 MHz、clkdiv 0 = 每 96 個時脈轉換一次 = 500 ksps，這是 RP2040
// ADC 的上限。FIFO 門檻 1，每個樣本都觸發一次 DREQ。
// ============================================================================

static int g_adc_dma;
static dma_channel_config g_adc_cfg;

static void adcSetup()
{
    adc_init();
    adc_set_clkdiv(0);
    adc_fifo_setup(true,    // 寫進 FIFO
                   true,    // 產生 DREQ
                   1,       // 一個樣本就觸發
                   false,   // 不要把錯誤旗標混進樣本
                   false);  // 保留 12 位元，不縮成 8 位元
    g_adc_dma = dma_claim_unused_channel(true);
    g_adc_cfg = dma_channel_get_default_config(g_adc_dma);
    channel_config_set_transfer_data_size(&g_adc_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&g_adc_cfg, false);
    channel_config_set_write_increment(&g_adc_cfg, true);
    channel_config_set_dreq(&g_adc_cfg, DREQ_ADC);
}

// GPIO 26 從「推 CLOCK」切成 ADC 輸入，開始抓一塊。
//
// 切換前 CLOCK 一定是低（掃描最後一步把它放低）。切過去之後腳位變成高阻抗，
// 偏壓網路把它慢慢拉到 0.41 V —— 仍在 74HC 的 LOW 門檻之下，595/165 看到的
// 一直是 0，不會多吃到時脈。回穩的那一段（約 41 µs 時間常數）由 SDR_SKIP
// 丟掉。
static void captureStart(uint16_t *dst)
{
    gpio_put(CLOCK_PIN, 0);
    adc_gpio_init(CLOCK_PIN);              // 功能 NULL、關數位輸入、關上下拉
    adc_select_input(0);                   // GPIO 26 = ADC0
    adc_fifo_drain();
    dma_channel_configure(g_adc_dma, &g_adc_cfg, dst, &adc_hw->fifo,
                          SDR_BLOCK, true);
    adc_run(true);
}

// 停 ADC，把 GPIO 26 交還給鍵盤。
static void captureStop()
{
    adc_run(false);
    if (dma_channel_is_busy(g_adc_dma))    // 正常不會發生：呼叫前已等 DMA 做完
        dma_channel_abort(g_adc_dma);
    while (!(adc_hw->cs & ADC_CS_READY_BITS))
        tight_loop_contents();             // 等最後一次轉換做完再清 FIFO
    adc_fifo_drain();

    gpio_init(CLOCK_PIN);                  // 先以輸入、輸出值 0 接手 ...
    gpio_put(CLOCK_PIN, 0);
    gpio_set_dir(CLOCK_PIN, GPIO_OUT);     // ... 再轉成輸出：不會冒出高脈衝
}

// ============================================================================
// 鍵盤矩陣
//
// 時序自 PicoApple2.ino scan_matrix()（經 rp2040-retro-dict）搬移，只有一處
// 不同：**移位時脈放慢了**。CLOCK 線上現在掛著 1 nF 耦合電容與天線，原本
// shiftOut() 的速度下邊緣來不及爬完，74HC595 可能少吃或多吃一個時脈。這裡
// 每個半週期等 2 µs（約 125 kHz），並把驅動能力開到 12 mA（DESIGN.md §1.2）。
//
// 輸出格式是 keys.c 要的「每列一個 byte、bit c = [row][col]」；74HC165 的
// 接線順序是反的，所以讀回來要 7-col。
// ============================================================================

static inline void clockPulse()
{
    gpio_put(CLOCK_PIN, 1);
    busy_wait_us_32(2);
    gpio_put(CLOCK_PIN, 0);
    busy_wait_us_32(2);
}

static void shiftOutSlow(uint8_t v)          // MSB first
{
    for (int i = 7; i >= 0; i--) {
        gpio_put(DATA_OUT_PIN, (v >> i) & 1);
        busy_wait_us_32(1);
        clockPulse();
    }
}

static uint8_t shiftInSlow()                 // 先讀再打時脈，與原版相同
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        if (gpio_get(DATA_IN_PIN))
            data |= (uint8_t)(1 << i);
        clockPulse();
    }
    return data;
}

static void scanMatrix(uint8_t rows[8])
{
    for (int row = 0; row < 8; row++) {
        gpio_put(LATCH_PIN, 0);
        shiftOutSlow(0);
        shiftOutSlow((uint8_t)(1 << row));
        gpio_put(LATCH_PIN, 1); busy_wait_us_32(5);
        gpio_put(LATCH_PIN, 0); busy_wait_us_32(1);
        gpio_put(LATCH_PIN, 1);

        uint8_t colData = shiftInSlow();
        uint8_t bits = 0;
        for (int col = 0; col < 8; col++)
            if (colData & (1 << (7 - col)))
                bits |= (uint8_t)(1 << col);
        rows[row] = bits;
    }
    clockPulse();
    gpio_put(CLOCK_PIN, 0);                  // 交給 ADC 前一定是低
}

// ============================================================================
// 遊戲按鍵 —— 跟鍵盤矩陣同一種 key_event
//
// 做法照搬 rp2040-retro-dict 的 dpadPoll()。鍵盤硬體已經沒有方向鍵了
// （那幾格在矩陣上空著），所以游標與參考位準主要靠這 8 顆。
//
// 這裡每 65 ms 才輪詢一次（跟著 ADC 的塊走），去彈跳 30 ms 的意思就變成
// 「連續兩次讀到一樣才算」—— 跟 keys.c 那邊是同一個效果，不必改參數。
// ============================================================================

static const struct { uint8_t gpio; uint8_t code; uint8_t repeat; } BTN[] = {
    { PIN_BTN_UP,     KEY_UP    , 1 },     // 參考位準 +5 dB
    { PIN_BTN_DOWN,   KEY_DOWN  , 1 },     // 參考位準 -5 dB
    { PIN_BTN_LEFT,   KEY_LEFT  , 1 },     // 游標
    { PIN_BTN_RIGHT,  KEY_RIGHT , 1 },
    { PIN_BTN_A,      KEY_PGUP  , 0 },     // 範圍 +20 dB
    { PIN_BTN_B,      KEY_PGDN  , 0 },     // 範圍 -20 dB
    { PIN_BTN_SELECT, 'a'       , 0 },     // 平均檔位
    { PIN_BTN_START,  'p'       , 0 },     // 峰值保持
};
#define BTN_N ((int)(sizeof BTN / sizeof BTN[0]))

static uint8_t  g_btn_stable;
static uint8_t  g_btn_pending;
static uint32_t g_btn_changed_ms[BTN_N];
static int      g_btn_repeat_i = -1;
static uint32_t g_btn_repeat_at;

static void dpadInit()
{
    for (int i = 0; i < BTN_N; i++) {
        gpio_init(BTN[i].gpio);
        gpio_set_dir(BTN[i].gpio, GPIO_IN);
        gpio_pull_up(BTN[i].gpio);
    }
}

static int dpadEmit(int n, key_event *out, int max, uint8_t code, int repeat)
{
    if (n >= max)
        return n;
    out[n].code = code;
    out[n].mods = 0;
    out[n].repeat = (uint8_t)repeat;
    return n + 1;
}

static int dpadPoll(int n, key_event *out, int max)
{
    uint32_t now = millis();
    uint8_t raw = 0;

    for (int i = 0; i < BTN_N; i++)
        if (!gpio_get(BTN[i].gpio))
            raw |= (uint8_t)(1u << i);

    for (int i = 0; i < BTN_N; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        int now_down  = (raw & bit) != 0;
        int was_down  = (g_btn_pending & bit) != 0;
        int is_stable = (g_btn_stable & bit) != 0;

        if (now_down != was_down) {
            g_btn_changed_ms[i] = now;
        } else if (now_down != is_stable &&
                   (int32_t)(now - g_btn_changed_ms[i]) >= KEYS_DEBOUNCE_MS) {
            g_btn_stable = (uint8_t)(now_down ? (g_btn_stable | bit)
                                              : (g_btn_stable & ~bit));
            if (now_down) {
                n = dpadEmit(n, out, max, BTN[i].code, 0);
                g_btn_repeat_i = i;
                g_btn_repeat_at = now + KEYS_REPEAT_MS;
            } else if (g_btn_repeat_i == i) {
                g_btn_repeat_i = -1;
            }
        } else if (now_down && is_stable && g_btn_repeat_i == i &&
                   BTN[i].repeat &&
                   (int32_t)(now - g_btn_repeat_at) >= 0) {
            n = dpadEmit(n, out, max, BTN[i].code, 1);
            g_btn_repeat_at = now + KEYS_RATE_MS;
        }
    }

    g_btn_pending = raw;
    return n;
}

// ============================================================================
// 顯示器
// ============================================================================

static void displayBegin()
{
    // 整段（含背光腳與硬體 reset 脈衝）自 rp2040-retro-dict 的 display_begin()
    // 原封搬移，源頭是 PicoApple2.ino setup1()。
    pinMode(PIN_DISPLAY_BL, OUTPUT); digitalWrite(PIN_DISPLAY_BL, LOW);

    tft.begin();
    gpio_put(PIN_DISPLAY_RST, 0); delay(100);
    gpio_put(PIN_DISPLAY_RST, 1); delay(100);
    tft.writeCommand(0x01); delay(150);
    tft.writeCommand(0xCB); tft.writeData(0x39); tft.writeData(0x2C);
                            tft.writeData(0x00); tft.writeData(0x34);
                            tft.writeData(0x02);
    tft.writeCommand(0xCF); tft.writeData(0x00); tft.writeData(0xC1);
                            tft.writeData(0x30);
    tft.writeCommand(0xE8); tft.writeData(0x85); tft.writeData(0x00);
                            tft.writeData(0x78);
    tft.writeCommand(0xEA); tft.writeData(0x00); tft.writeData(0x00);
    tft.writeCommand(0xED); tft.writeData(0x64); tft.writeData(0x03);
                            tft.writeData(0x12); tft.writeData(0x81);
    tft.writeCommand(0xF7); tft.writeData(0x20);
    tft.writeCommand(0xC0); tft.writeData(0x23);
    tft.writeCommand(0xC1); tft.writeData(0x10);
    tft.writeCommand(0xC5); tft.writeData(0x3E); tft.writeData(0x28);
    tft.writeCommand(0xC7); tft.writeData(0x86);
    tft.setRotation(1);                     // 橫的，320x240
    tft.writeCommand(0x3A); tft.writeData(0x55);
    tft.writeCommand(0xB1); tft.writeData(0x00); tft.writeData(0x18);
    tft.writeCommand(0xB6); tft.writeData(0x08); tft.writeData(0x82);
                            tft.writeData(0x27);
    tft.writeCommand(0x11); delay(150);      // sleep out
    tft.writeCommand(0x29); delay(150);      // display on

    spi_set_baudrate(spi0, 62500000);
    tft.fillScreen(0x0000);
    digitalWrite(PIN_DISPLAY_BL, HIGH);
}

// 整張畫面約 20 ms（153 KB @ 62.5 MHz）。算下一列與 DMA 送這一列同時進行。
static void render()
{
    sdr_prepare(&g_sdr);
    tft.startFrame(0, 0, UI_W - 1, UI_H - 1);
    for (int y = 0; y < UI_H; y++) {
        uint16_t *buf = g_line[y & 1];
        ui_line(&g_sdr, y, buf);
        tft.waitTransferDone();
        tft.sendScanlineAsync(buf, UI_W);
    }
    tft.waitTransferDone();
    digitalWrite(PIN_DISPLAY_CS, HIGH);
}

// ============================================================================

void setup()
{
    Serial.begin(115200);

    gpio_init(DATA_OUT_PIN); gpio_set_dir(DATA_OUT_PIN, GPIO_OUT);
    gpio_init(LATCH_PIN);    gpio_set_dir(LATCH_PIN, GPIO_OUT);
    gpio_put(LATCH_PIN, 1);                       // 閒置停在高
    gpio_init(CLOCK_PIN);    gpio_set_dir(CLOCK_PIN, GPIO_OUT);
    gpio_put(CLOCK_PIN, 0);
    gpio_set_drive_strength(CLOCK_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(CLOCK_PIN, GPIO_SLEW_RATE_FAST);
    gpio_init(DATA_IN_PIN);  gpio_set_dir(DATA_IN_PIN, GPIO_IN);
    dpadInit();

    displayBegin();
    sdr_init(&g_sdr);
    keys_init(&g_keys);
    adcSetup();

    render();                                     // 開機先有畫面，不等第一塊
    g_cur = 0;
    captureStart(g_buf[g_cur]);
}

void loop()
{
    static uint32_t last_log;
    static bool painted_quiet;

    // 進來時 DMA 已經做完 = 上一圈（處理＋畫面）比一塊的 65 ms 還久，這一塊
    // 之後到現在的樣本沒接上。寬頻頻譜不在乎連續性，但記下來，畫面上看得到。
    if (!dma_channel_is_busy(g_adc_dma))
        g_sdr.drops++;
    dma_channel_wait_for_finish_blocking(g_adc_dma);

    uint32_t t0 = time_us_32();
    captureStop();
    uint8_t rows[8];
    scanMatrix(rows);
    int done = g_cur;
    g_cur ^= 1;
    captureStart(g_buf[g_cur]);
    g_sdr.scan_us = time_us_32() - t0;

    // 按鍵
    key_event ev[KEYS_MAX_EVENTS];
    uint32_t now = millis();
    int n = keys_update(&g_keys, rows, now, ev, KEYS_MAX_EVENTS);
    n = dpadPoll(n, ev, KEYS_MAX_EVENTS);
    for (int i = 0; i < n; i++)
        sdr_key(&g_sdr, &ev[i]);

    // DSP
    sdr_block(&g_sdr, g_buf[done], SDR_BLOCK, SDR_SKIP);

    // 畫面。QUIET 模式只畫一次（讓使用者看到 QUIET 字樣），之後完全不碰
    // SPI —— 用來比較「LCD 在送」與「LCD 安靜」時的雜訊底線差多少。
    // 瀑布圖照樣在 RAM 裡累積，解除後那一段會以不同的底色出現。
    if (!g_sdr.quiet) {
        render();
        painted_quiet = false;
    } else if (!painted_quiet) {
        render();
        painted_quiet = true;
    }

    g_sdr.proc_ms = (time_us_32() - t0) / 1000;   // 掃描＋DSP＋畫面，下一張才顯示

    if (now - last_log >= 1000) {
        last_log = now;
        Serial.printf("blk %lu  proc %lu ms  scan %lu us  drops %lu  NF %d.%d dBFS\n",
                      (unsigned long)g_sdr.blocks, (unsigned long)g_sdr.proc_ms,
                      (unsigned long)g_sdr.scan_us, (unsigned long)g_sdr.drops,
                      g_sdr.sp.nf / 10, abs(g_sdr.sp.nf % 10));
    }
}
