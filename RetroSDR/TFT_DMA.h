#ifndef TFT_DMA_H
#define TFT_DMA_H

#include <Arduino.h>
#include "hardware/spi.h"
#include "hardware/dma.h"

class TFT_DMA {
public:
    TFT_DMA(uint cs, uint dc, uint rst, uint mosi, uint sck);
    void begin();
    
    // 進入傳輸模式
    void startFrame(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    // 非阻塞 DMA 傳送
    void sendScanlineAsync(const uint16_t* buffer, uint16_t length);
    // 等待 DMA 完成
    void waitTransferDone();

    // 基礎 UI 繪製 (阻塞式，確保穩定)
    void fillScreen(uint16_t color);
    void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void setRotation(uint8_t r);
    void drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, const uint8_t* font_rom);

    // 指令發送
    void writeCommand(uint8_t cmd);
    void writeData(uint8_t data);

private:
    uint _cs, _dc, _rst, _mosi, _sck;
    spi_inst_t* _spi = spi0;
    int _dma_chan;
};

#endif
