/* .wav 檔頭，見 wav.h。 */
#include "wav.h"

#include <string.h>

static void le16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void le32(uint8_t *p, uint32_t v) { le16(p, v); le16(p + 2, v >> 16); }

void wav_header(uint8_t h[WAV_HDR], uint32_t rate, uint32_t n)
{
    uint32_t data = n * 2;
    memcpy(h, "RIFF", 4);
    le32(h + 4, 36 + data);
    memcpy(h + 8, "WAVEfmt ", 8);
    le32(h + 16, 16);              /* fmt 區塊長度 */
    le16(h + 20, 1);               /* PCM */
    le16(h + 22, 1);               /* 單聲道 */
    le32(h + 24, rate);
    le32(h + 28, rate * 2);        /* 每秒位元組 */
    le16(h + 32, 2);               /* 每個樣本幾位元組 */
    le16(h + 34, 16);
    memcpy(h + 36, "data", 4);
    le32(h + 40, data);
}
