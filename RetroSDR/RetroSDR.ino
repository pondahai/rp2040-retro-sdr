// ============================================================================
// RetroSDR —— RP2040 掌機上的 LF 直接取樣 SDR
//
// Core 0：
//   1. GPIO 26 分時：ADC 抓一整塊 -> 切回 GPIO 掃鍵盤 -> 再切回 ADC
//   2. 抓好的那一塊交給 sdr.c（FFT、雜訊底線、瀑布圖）
//   3. 逐列向 ui.c 要畫面，DMA 送上 ILI9341
//
// Core 1（M2）：
//   4. 同一塊樣本交給 ddc.c（混頻、CIC、FIR、解調、AGC）-> 音訊環形緩衝
//   5. 計時中斷每 128 µs 取一個樣本寫進 GPIO 7 的 PWM -> PAM8403 -> 喇叭
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
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

#include "TFT_DMA.h"

extern "C" {
#include "src/rs_sdr.h"
#include "src/rs_jjy.h"
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

// ---- 喇叭 ----
// 與 retro-dict / InfoNES 同一支腳。PWM 載波 500 kHz，跟 ADC 取樣率相同，
// 經 500 ksps 取樣後會摺疊到 DC，被 FFT 去 DC 與 DDC 濾掉（DESIGN.md §4.3）。
// 前提是 clk_sys 為 250 MHz：250 MHz / 500 = 500 kHz。改時脈要一起改 wrap。
#define PIN_SPEAKER      7
#define PWM_WRAP         499
#define PWM_MID          250

// ---- 測試訊號（M2 驗收用）----
// GPIO 0 目前閒置（HARDWARE.md）。用 PWM 打一個 68.49 kHz 的方波（BPC 附近），
// 靠板上走線的串音或旁邊繞一圈線耦合進 GPIO 26，不用外部儀器就能驗證
// 天線 -> ADC -> 頻譜 -> DDC -> 喇叭 的整條路徑。方波的 3 次諧波在 205.5 kHz，
// 頻譜上也該看得到。鍵盤 T 開關。
//
// 250 MHz / 3650 = 68,493 Hz。是方波，不是正弦，而且 GPIO 推的是 3.3 V ——
// 當然不能接天線發射出去，只是給自己聽。
#define PIN_TESTTONE     0
#define TESTTONE_WRAP    3649
#define TESTTONE_FREQ    (SYS_CLOCK_KHZ * 1000 / (TESTTONE_WRAP + 1))

static void testToneSet(bool on)
{
    static bool inited, state;
    if (!inited) {
        uint slice = pwm_gpio_to_slice_num(PIN_TESTTONE);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_config_set_wrap(&cfg, TESTTONE_WRAP);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(PIN_TESTTONE, (TESTTONE_WRAP + 1) / 2);
        inited = true;
    }
    if (on == state)
        return;
    state = on;
    if (on) {
        gpio_set_function(PIN_TESTTONE, GPIO_FUNC_PWM);
    } else {
        gpio_init(PIN_TESTTONE);           // 關掉 = 回到 SIO 輸入，高阻抗
    }
}

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

// 系統時脈。arduino-pico 預設 133 MHz，M1 第一次上機量到一塊要 139 ms
// （一塊只有 65 ms），所以比照 PicoApple2 超頻到 250 MHz。
//
// 改這個數字不影響取樣率：clk_adc 來自 PLL_USB 的 48 MHz，跟 clk_sys 無關。
// SPI 的 62.5 MHz 在 spi_set_baudrate() 時依新的 clk_peri 重算，250 MHz 下
// 剛好整除。M2 的音訊 PWM 才會跟它綁在一起（DESIGN.md §2.2）。
#define SYS_CLOCK_KHZ 250000

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
    { PIN_BTN_LEFT,   KEY_LEFT  , 1 },     // 調諧
    { PIN_BTN_RIGHT,  KEY_RIGHT , 1 },
    { PIN_BTN_A,      KEY_PGUP  , 1 },     // 音量 +
    { PIN_BTN_B,      KEY_PGDN  , 1 },     // 音量 -
    { PIN_BTN_SELECT, 'm'       , 0 },     // 解調模式
    { PIN_BTN_START,  's'       , 0 },     // 調諧步進
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
// 畫面時間拆成「算像素」與「等 SPI」，診斷用（每秒印一次）。
static uint32_t g_t_prep, g_t_line, g_t_wait;

static void render()
{
    uint32_t t0 = time_us_32();
    sdr_prepare(&g_sdr);
    uint32_t t1 = time_us_32(), tl = 0, tw = 0;
    tft.startFrame(0, 0, UI_W - 1, UI_H - 1);
    for (int y = 0; y < UI_H; y++) {
        uint16_t *buf = g_line[y & 1];
        uint32_t a = time_us_32();
        ui_line(&g_sdr, y, buf);
        uint32_t b = time_us_32();
        tft.waitTransferDone();
        tw += time_us_32() - b;
        tl += b - a;
        tft.sendScanlineAsync(buf, UI_W);
    }
    uint32_t b = time_us_32();
    tft.waitTransferDone();
    tw += time_us_32() - b;
    digitalWrite(PIN_DISPLAY_CS, HIGH);
    g_t_prep = t1 - t0;
    g_t_line = tl;
    g_t_wait = tw;
}

// ============================================================================
// Core 1：DDC 與音訊
//
// Core 0 每抓完一塊就把緩衝區編號丟進跨核 FIFO。那一塊要等下一塊抓完（65 ms
// 之後）才會被 DMA 覆寫，Core 1 只要在那之前做完就好 —— 實測見序列埠的 ddc。
//
// 音訊的產出是一陣一陣的（每 66.5 ms 一次 512 個），播放是等速的。塊與塊
// 之間有鍵盤掃描的空隙，產出率比 7812.5 Hz 略低約 1.6%，所以播放間隔會依
// 環形緩衝的水位在 127/128/130 µs 之間微調，不讓它見底也不讓它溢出。
// ============================================================================

static ddc g_ddc;                          // 只有 Core 1 碰
// 授時碼解碼器：Core 1 寫，Core 0 只讀（畫面用，讀到一半被改也只是某一格
// 符號晚一圈才更新，不影響解碼本身）。
static jjy g_jjy;
static volatile uint32_t g_env_now;        // 最新一個包絡點的時間戳（ms）

// Core 0 寫、Core 1 讀。改了就把 seq 加一，Core 1 在下一塊開頭套用。
static volatile int32_t  g_p_tune;
static volatile int      g_p_mode, g_p_bw;
static volatile uint32_t g_p_seq;
static volatile int      g_vol;            // 0..SDR_VOL_MAX
static volatile bool     g_core0_ready;

#define AUD_N 2048                         // 2 的次方
static int16_t           g_aud[AUD_N];
static volatile uint32_t g_aud_w, g_aud_r; // 只增不減，相減就是水位
static volatile uint32_t g_aud_under;      // 播放時沒東西可播的次數
static volatile uint32_t g_ddc_us;         // Core 1 處理一塊花多久

static uint s_pwm_slice, s_pwm_chan;

static int64_t audioTick(alarm_id_t, void *)
{
    uint32_t fill = g_aud_w - g_aud_r;
    int level = PWM_MID;
    if (fill) {
        int32_t v = g_aud[g_aud_r & (AUD_N - 1)];
        g_aud_r = g_aud_r + 1;
        // ±32767 × 音量 -> ±(PWM_MID-1)
        level += (int)(v * g_vol * (PWM_MID - 1) / (32767 * SDR_VOL_MAX));
    } else {
        g_aud_under = g_aud_under + 1;
    }
    pwm_set_chan_level(s_pwm_slice, s_pwm_chan, (uint16_t)level);

    // 負值 = 以上一次的預定時間為準再過這麼久（不累積誤差）
    //
    // 水位要停在一塊（512）以上：產出是每 66.5 ms 一次倒進 512 個，兩次之間
    // 要撐得住。第一版門檻設在 256，上機實測每秒見底約 56 次（喀喀聲）。
    // 停在 768 附近 = 約 0.1 s 的延遲，換來不斷音。
    if (fill < 768)  return -135;          // 7407 Hz，明顯比產出（約 7580）慢 -> 回升
    if (fill > 1280) return -127;          // 7874 Hz，比產出快一點 -> 水位下降
    return -128;
}

static void audioBegin()
{
    gpio_set_function(PIN_SPEAKER, GPIO_FUNC_PWM);
    s_pwm_slice = pwm_gpio_to_slice_num(PIN_SPEAKER);
    s_pwm_chan = pwm_gpio_to_channel(PIN_SPEAKER);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(s_pwm_slice, &cfg, true);
    pwm_set_chan_level(s_pwm_slice, s_pwm_chan, PWM_MID);
}

void setup1()
{
    while (!g_core0_ready)
        tight_loop_contents();
    ddc_init(&g_ddc);
    jjy_init(&g_jjy);
    // 鬧鐘池建在 Core 1：中斷就跑在 Core 1，不會被 Core 0 的 FFT 或 SPI 拖慢
    // （PicoApple2 的音訊重放也是這樣做）
    alarm_pool_t *pool = alarm_pool_create_with_unused_hardware_alarm(4);
    alarm_pool_add_alarm_in_us(pool, 128, audioTick, NULL, true);
}

void loop1()
{
    static uint32_t seq_applied;
    static int16_t out[SDR_BLOCK / DDC_DECIM];
    uint32_t idx;

    if (!rp2040.fifo.pop_nb(&idx))
        return;

    uint32_t seq = g_p_seq;
    if (seq != seq_applied) {
        seq_applied = seq;
        int32_t old = g_ddc.tune_hz;
        ddc_set(&g_ddc, g_p_tune, g_p_mode, g_p_bw);
        if (g_p_tune != old)
            jjy_init(&g_jjy);               // 換台了，前面的符號不算數
    }

    uint32_t t0 = time_us_32();
    // FIFO 的一個 word：bit 0 = 緩衝區編號，其餘 = 這一塊之前的空檔（取樣點數）
    int gap = (int)(idx >> 1);
    int n = ddc_block(&g_ddc, g_buf[idx & 1], SDR_BLOCK, SDR_SKIP, gap, out,
                      (int)(sizeof out / sizeof out[0]));
    g_ddc_us = time_us_32() - t0;

    for (int k = 0; k < g_ddc.env_n; k++)
        jjy_push(&g_jjy, g_ddc.env[k], g_ddc.env_ms[k]);
    if (g_ddc.env_n)
        g_env_now = g_ddc.env_ms[g_ddc.env_n - 1];

    for (int i = 0; i < n; i++) {
        if (g_aud_w - g_aud_r >= AUD_N)
            break;                          // 滿了就丟（正常不會發生）
        g_aud[g_aud_w & (AUD_N - 1)] = out[i];
        g_aud_w = g_aud_w + 1;
    }
}

// ============================================================================
// 授時碼狀態列：調在 JJY（40 / 60 kHz ±500 Hz）上才顯示
//
//   JJY 22:49:37 OK  ...M0010M01001  （鎖定：時間每秒往前走）
//   JJY --:--:--     ...M00?10       （還沒解出來）
//
// 時間 = 最近解出的那一幀（代表它的 M 那一刻）＋ 從那一刻到現在的時間。
// 兩個時間戳都來自 DDC 的取樣計數，所以跟 millis() 無關，也把鍵盤掃描的
// 空檔算進去了。
// ============================================================================

static void updateTimecodeLine()
{
    int32_t f = g_sdr.tune_hz;
    if (!((f > 39500 && f < 40500) || (f > 59500 && f < 60500))) {
        g_sdr.tc[0] = 0;
        g_sdr.tc_locked = 0;
        return;
    }
    char tbuf[16];
    const char *flag = "  ";
    if (g_jjy.good >= 1) {
        uint32_t el = (g_env_now - g_jjy.t_ms) / 1000;
        uint32_t secs = (uint32_t)(g_jjy.t.hour * 3600 + g_jjy.t.min * 60) + el;
        secs %= 86400;
        snprintf(tbuf, sizeof tbuf, "%02lu:%02lu:%02lu", (unsigned long)(secs / 3600),
                 (unsigned long)(secs / 60 % 60), (unsigned long)(secs % 60));
        flag = jjy_locked(&g_jjy) ? "OK" : "? ";
    } else {
        strcpy(tbuf, "--:--:--");
    }
    // 符號放最右邊，最新的在最後面
    int room = UI_TEXT_COLS - 17;
    snprintf(g_sdr.tc, sizeof g_sdr.tc, "JJY %s %s %s", tbuf, flag,
             g_jjy.hist + JJY_HIST - room);
    g_sdr.tc_locked = jjy_locked(&g_jjy);
}

// 調諧點 ±1 bin 的最大值（dBFS×10）。序列埠與統計頁共用。
static int tuneLevel()
{
    const spectrum *sp = &g_sdr.sp;
    int cb = sdr_cursor_bin(&g_sdr), lv = sp->bin_db[cb];
    if (cb > 0 && sp->bin_db[cb - 1] > lv) lv = sp->bin_db[cb - 1];
    if (cb < SP_BINS - 1 && sp->bin_db[cb + 1] > lv) lv = sp->bin_db[cb + 1];
    return lv;
}

// dB×10 -> "-97.0"
static const char *db10(char *b, int v)
{
    sprintf(b, "%s%d.%d", v < 0 ? "-" : "", abs(v) / 10, abs(v) % 10);
    return b;
}

// ============================================================================
// 統計頁（鍵盤 I）：序列埠上那些數字，搬到螢幕上
//
// 收訊最乾淨的時候是拔掉 USB、用電池跑（筆電充電器的雜訊會從 USB 地線灌進來），
// 那時候讀不到序列埠。這頁只在打開時才組字串，不打開不花時間。
// ============================================================================

static void updateStats(uint32_t scan_us, uint32_t dsp_us, uint32_t draw_us)
{
    if (!g_sdr.show_stats)
        return;
    const spectrum *sp = &g_sdr.sp;
    char (*L)[UI_TEXT_COLS + 1] = g_sdr.stats;
    const int W = UI_TEXT_COLS + 1;
    char a[12], b[12];
    uint32_t up = millis() / 1000;
    int lv = tuneLevel();

    snprintf(L[0], W, "STATS                          uptime %02lu:%02lu:%02lu",
             (unsigned long)(up / 3600), (unsigned long)(up / 60 % 60), (unsigned long)(up % 60));
    snprintf(L[1], W, "CORE0  proc %lu ms = scan %lu.%lu + dsp %lu + draw %lu",
             (unsigned long)g_sdr.proc_ms, (unsigned long)(scan_us / 1000),
             (unsigned long)(scan_us / 100 % 10), (unsigned long)(dsp_us / 1000),
             (unsigned long)(draw_us / 1000));
    snprintf(L[2], W, "       fft %lu ms  db %lu ms   blocks %lu  drop %lu",
             (unsigned long)(sp->t_fft / 1000), (unsigned long)(sp->t_db / 1000),
             (unsigned long)g_sdr.blocks, (unsigned long)g_sdr.drops);
    snprintf(L[3], W, "CORE1  ddc %lu ms of 65  (fir %lu  cic %lu)",
             (unsigned long)(g_ddc_us / 1000), (unsigned long)(g_ddc.t_post / 1000),
             (unsigned long)((g_ddc.t_total - g_ddc.t_post) / 1000));
    snprintf(L[4], W, "AUDIO  fill %lu  underruns %lu  vol %d",
             (unsigned long)(g_aud_w - g_aud_r), (unsigned long)g_aud_under, g_sdr.vol);
    snprintf(L[5], W, "CLOCK  sys %lu MHz  peri %lu MHz  spi %lu.%lu MHz",
             (unsigned long)(clock_get_hz(clk_sys) / 1000000),
             (unsigned long)(clock_get_hz(clk_peri) / 1000000),
             (unsigned long)(spi_get_baudrate(spi0) / 1000000),
             (unsigned long)(spi_get_baudrate(spi0) / 100000 % 10));
    {
        char c[12];
        snprintf(L[6], W, "SIGNAL NF %s dBFS  tune %s dBFS  S/N %s",
                 db10(a, sp->nf), db10(b, lv), db10(c, lv - sp->nf));
    }
    snprintf(L[7], W, "JJY    span %d dB  sym %lu  frames %lu  err %d",
             (int)(g_jjy.hi - g_jjy.lo), (unsigned long)g_jjy.symbols,
             (unsigned long)g_jjy.frames, g_jjy.last_err);
    snprintf(L[8], W, "TX     %s", g_sdr.tx_on ? "on, GPIO 0, 68493 Hz" : "off");
    L[9][0] = 0;
    snprintf(L[10], W, "I = back to waterfall");
}

static void pushDdcParams()
{
    g_p_tune = g_sdr.tune_hz;
    g_p_mode = g_sdr.mode;
    g_p_bw = g_sdr.bw_hz;
    g_p_seq = g_p_seq + 1;
    g_sdr.ddc_dirty = 0;
}

void setup()
{
    // 要在任何周邊設定之前：SPI 的除頻是依當下的 clk_peri 算的
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
    // arduino-pico 把 clk_peri 接在 48 MHz 的 USB PLL 上，SPI 最快只有
    // clk_peri/2 = 24 MHz —— 上機實測 spi0 = 24 MHz，一張畫面光等 SPI 就
    // 48 ms。改接 clk_sys 之後 62.5 MHz 才真的拿得到（250/4）。
    // 這支韌體沒有用 UART，換 clk_peri 不影響別的東西；USB 用的是 clk_usb。
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    SYS_CLOCK_KHZ * 1000, SYS_CLOCK_KHZ * 1000);
    Serial.begin(115200);
    sp_clock_us = time_us_32;
    ddc_clock_us = time_us_32;

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
    audioBegin();
    g_vol = g_sdr.vol;
    pushDdcParams();
    g_core0_ready = true;

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
    // 新的這一塊（g_cur）之前空了 scan_us 這麼久，換算成 500 ksps 的點數，
    // 跟著「剛抓好的那一塊」一起交給 Core 1 —— Core 1 處理下一塊時會用到。
    // 剛抓好的那一塊在下一塊抓完之前都不會被覆寫。
    {
        static uint32_t gap_before_done;
        rp2040.fifo.push_nb((uint32_t)done | (gap_before_done << 1));
        gap_before_done = g_sdr.scan_us / 2;
    }

    // 按鍵
    key_event ev[KEYS_MAX_EVENTS];
    uint32_t now = millis();
    int n = keys_update(&g_keys, rows, now, ev, KEYS_MAX_EVENTS);
    n = dpadPoll(n, ev, KEYS_MAX_EVENTS);
    for (int i = 0; i < n; i++)
        sdr_key(&g_sdr, &ev[i]);
    if (g_sdr.ddc_dirty)
        pushDdcParams();
    g_vol = g_sdr.vol;
    testToneSet(g_sdr.tx_on);
    updateTimecodeLine();

    // DSP
    uint32_t t1 = time_us_32();
    sdr_block(&g_sdr, g_buf[done], SDR_BLOCK, SDR_SKIP);
    uint32_t t2 = time_us_32();

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

    uint32_t t3 = time_us_32();
    g_sdr.proc_ms = (t3 - t0) / 1000;             // 掃描＋DSP＋畫面，下一張才顯示
    updateStats(g_sdr.scan_us, t2 - t1, t3 - t2);

    // 每秒一行，拆開各段時間：要優化哪一邊，看這裡
    if (now - last_log >= 1000) {
        const spectrum *sp = &g_sdr.sp;
        last_log = now;
        Serial.printf("blk %lu drops %lu | proc %lu ms = scan %lu us + dsp %lu ms "
                      "(K=%d: win %lu fft %lu db %lu post %lu us) + draw %lu ms "
                      "| %lu MHz | NF %d.%d dBFS\n",
                      (unsigned long)g_sdr.blocks, (unsigned long)g_sdr.drops,
                      (unsigned long)g_sdr.proc_ms, (unsigned long)g_sdr.scan_us,
                      (unsigned long)((t2 - t1) / 1000), sp->k_used,
                      (unsigned long)sp->t_win, (unsigned long)sp->t_fft,
                      (unsigned long)sp->t_db, (unsigned long)sp->t_post,
                      (unsigned long)((t3 - t2) / 1000),
                      (unsigned long)(clock_get_hz(clk_sys) / 1000000),
                      sp->nf / 10, abs(sp->nf % 10));
        {
            int cb = sdr_cursor_bin(&g_sdr), lv = sp->bin_db[cb];
            if (cb > 0 && sp->bin_db[cb - 1] > lv) lv = sp->bin_db[cb - 1];
            if (cb < SP_BINS - 1 && sp->bin_db[cb + 1] > lv) lv = sp->bin_db[cb + 1];
            int b3 = (int)((3LL * TESTTONE_FREQ * SP_N + SP_FS / 2) / SP_FS);
            Serial.printf("    tune %ld Hz %s: sig %d.%d dBFS (S/N %d.%d) | 3rd harmonic bin %d.%d | tx %d\n",
                          (long)g_sdr.tune_hz, ddc_mode_name(g_sdr.mode),
                          lv / 10, abs(lv % 10), (lv - sp->nf) / 10, abs((lv - sp->nf) % 10),
                          sp->bin_db[b3] / 10, abs(sp->bin_db[b3] % 10), g_sdr.tx_on);
        }
        if (g_sdr.tc[0])
            Serial.printf("    %s | span %d dB, sym %lu, frames %lu, err %d\n", g_sdr.tc,
                          (int)(g_jjy.hi - g_jjy.lo), (unsigned long)g_jjy.symbols,
                          (unsigned long)g_jjy.frames, g_jjy.last_err);
        Serial.printf("    core1: ddc %lu us/block (fir+demod %lu us, rest %lu us), "
                      "audio fill %lu, underruns %lu\n",
                      (unsigned long)g_ddc_us, (unsigned long)g_ddc.t_post,
                      (unsigned long)(g_ddc.t_total - g_ddc.t_post),
                      (unsigned long)(g_aud_w - g_aud_r), (unsigned long)g_aud_under);
        Serial.printf("    draw: prep %lu us, ui_line %lu us, spi wait %lu us | "
                      "clk_peri %lu MHz, spi0 %lu Hz\n",
                      (unsigned long)g_t_prep, (unsigned long)g_t_line,
                      (unsigned long)g_t_wait,
                      (unsigned long)(clock_get_hz(clk_peri) / 1000000),
                      (unsigned long)spi_get_baudrate(spi0));
    }
}
