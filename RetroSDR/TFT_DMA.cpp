#include "TFT_DMA.h"
#include "hardware/gpio.h"

TFT_DMA::TFT_DMA(uint cs, uint dc, uint rst, uint mosi, uint sck) 
    : _cs(cs), _dc(dc), _rst(rst), _mosi(mosi), _sck(sck), _dma_chan(-1) {}

void TFT_DMA::begin() {
    gpio_init(_cs); gpio_set_dir(_cs, GPIO_OUT); gpio_put(_cs, 1);
    gpio_init(_dc); gpio_set_dir(_dc, GPIO_OUT); gpio_put(_dc, 0);
    gpio_init(_rst); gpio_set_dir(_rst, GPIO_OUT); gpio_put(_rst, 1);

    spi_init(_spi, 20000000);
    gpio_set_function(_mosi, GPIO_FUNC_SPI);
    gpio_set_function(_sck, GPIO_FUNC_SPI);
    
    _dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, spi_get_index(_spi) == 0 ? DREQ_SPI0_TX : DREQ_SPI1_TX);
    dma_channel_configure(_dma_chan, &c, &spi_get_hw(_spi)->dr, NULL, 0, false);
}

void TFT_DMA::writeCommand(uint8_t cmd) {
    waitTransferDone();
    gpio_put(_dc, 0); gpio_put(_cs, 0);
    spi_write_blocking(_spi, &cmd, 1);
    gpio_put(_cs, 1);
}

void TFT_DMA::writeData(uint8_t data) {
    waitTransferDone();
    gpio_put(_dc, 1); gpio_put(_cs, 0);
    spi_write_blocking(_spi, &data, 1);
    gpio_put(_cs, 1);
}

void TFT_DMA::startFrame(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    writeCommand(0x2A); 
    uint8_t xdata[] = {(uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF), (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)};
    gpio_put(_dc, 1); gpio_put(_cs, 0); spi_write_blocking(_spi, xdata, 4); gpio_put(_cs, 1);

    writeCommand(0x2B);
    uint8_t ydata[] = {(uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF), (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)};
    gpio_put(_dc, 1); gpio_put(_cs, 0); spi_write_blocking(_spi, ydata, 4); gpio_put(_cs, 1);

    waitTransferDone();
    gpio_put(_dc, 0); gpio_put(_cs, 0);
    uint8_t cmd = 0x2C;
    spi_write_blocking(_spi, &cmd, 1);
    gpio_put(_dc, 1); // 切換為 Data 模式準備 DMA
}

void TFT_DMA::sendScanlineAsync(const uint16_t* buffer, uint16_t length) {
    dma_channel_set_read_addr(_dma_chan, buffer, false);
    dma_channel_set_trans_count(_dma_chan, length * 2, true);
}

void TFT_DMA::waitTransferDone() {
    if (_dma_chan >= 0 && dma_channel_is_busy(_dma_chan)) {
        dma_channel_wait_for_finish_blocking(_dma_chan);
    }
    // 關鍵：DMA 完成只代表資料已填入 SPI TX FIFO，移位暫存器可能還在打最後
    // 幾個 byte。必須等 PL022 的 BSY 清零（移位完成且 FIFO 排空）才能安全送下
    // 一個命令／拉 CS，否則 command/data 會在 SPI 線上對撞，累積性失步致畫面全白。
    while (spi_is_busy(_spi)) tight_loop_contents();
}

void TFT_DMA::setRotation(uint8_t r) {
    writeCommand(0x36);
    switch (r % 4) {
        case 0: writeData(0x48); break;
        case 1: writeData(0x28); break;
        case 2: writeData(0x88); break;
        case 3: writeData(0xE8); break;
    }
}

void TFT_DMA::fillScreen(uint16_t color) {
    drawRect(0, 0, 320, 240, color);
}

// 整列緩衝：一次填滿一整行像素再送出，取代每像素一次 spi_write_blocking。
// 320 px * 2 byte = 640 byte，放 .bss 不吃堆疊。
static uint8_t s_line_buf[320 * 2];

void TFT_DMA::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (w == 0 || h == 0) return;
    uint16_t line_w = w > 320 ? 320 : w;
    for (uint16_t i = 0; i < line_w; i++) {
        s_line_buf[i * 2]     = (uint8_t)(color >> 8);
        s_line_buf[i * 2 + 1] = (uint8_t)(color & 0xFF);
    }

    startFrame(x, y, x + w - 1, y + h - 1);
    for (uint32_t row = 0; row < h; row++) {
        uint16_t remain = w;
        while (remain) {
            uint16_t n = remain > line_w ? line_w : remain;
            spi_write_blocking(_spi, s_line_buf, n * 2);
            remain -= n;
        }
    }
    gpio_put(_cs, 1);
}

void TFT_DMA::drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, const uint8_t* font_rom) {
    if (!font_rom) return;
    uint8_t char_idx = (uint8_t)c;
    if (char_idx >= 32 && char_idx <= 63) char_idx += 64;
    else if (char_idx >= 96) char_idx -= 32;

    // 整個字元 7x8 = 56 px = 112 byte，組完一次送出（原本是 56 次 2-byte 呼叫）
    uint8_t glyph[7 * 8 * 2];
    uint8_t hi_c = (uint8_t)(color >> 8), lo_c = (uint8_t)(color & 0xFF);
    uint8_t hi_b = (uint8_t)(bg >> 8),    lo_b = (uint8_t)(bg & 0xFF);

    uint8_t* p = glyph;
    for (int row = 0; row < 8; row++) {
        uint8_t bits = font_rom[char_idx * 8 + row];
        for (int col = 0; col < 7; col++) {
            bool pixel = (bits & (1 << (6 - col))) != 0;
            *p++ = pixel ? hi_c : hi_b;
            *p++ = pixel ? lo_c : lo_b;
        }
    }

    startFrame(x, y, x + 6, y + 7);
    spi_write_blocking(_spi, glyph, sizeof(glyph));
    gpio_put(_cs, 1);
}
