#include "spectrum.h"

#include <math.h>
#include <string.h>

/* 工作區放在模組裡，不放進 spectrum 結構：它們只在 spectrum_block()
 * 執行期間有意義，而結構會被整個嵌進 sdr。 */
static int16_t s_re[FFT_M], s_im[FFT_M];   /* 偶數點 / 奇數點 */
static int16_t s_win[SP_N];
static int32_t s_acc[SP_BINS];

uint32_t (*sp_clock_us)(void);

static uint32_t now_us(void)
{
    return sp_clock_us ? sp_clock_us() : 0;
}

/* 峰值保持每塊衰減多少（dB×10）。一塊 65 ms，1 -> 約 1.5 dB/s。 */
#define SP_PEAK_DECAY 1

void spectrum_init(spectrum *sp)
{
    const double PI = 3.14159265358979323846;
    fft_init();
    for (int i = 0; i < SP_N; i++)
        s_win[i] = (int16_t)lround((0.5 - 0.5 * cos(2.0 * PI * i / SP_N)) * 32767.0);
    memset(sp, 0, sizeof(*sp));
    sp->nf = FFT_DB10_MIN;
}

void spectrum_px_bins(int px, int *b0, int *b1)
{
    *b0 = px * SP_BINS / SP_W;
    *b1 = (px + 1) * SP_BINS / SP_W;
}

void spectrum_peak_reset(spectrum *sp)
{
    memcpy(sp->peak, sp->raw, sizeof(sp->peak));
}

/* 中位數用直方圖求：1 dB 一格，-160..0 dBFS。排序 2048 個值在 M0+ 上太貴。 */
static int16_t median_db10(const int16_t *v, int n)
{
    static uint16_t hist[161];
    int i, half = n / 2, sum = 0;

    memset(hist, 0, sizeof(hist));
    for (i = 0; i < n; i++) {
        int b = -v[i] / 10;              /* 0 .. 160，0 = 0 dBFS */
        if (b < 0) b = 0;
        if (b > 160) b = 160;
        hist[b]++;
    }
    /* 從最低（b 最大）往上數到一半 */
    for (i = 160; i >= 0; i--) {
        sum += hist[i];
        if (sum > half)
            return (int16_t)(-i * 10);
    }
    return 0;
}

int spectrum_block(spectrum *sp, const uint16_t *x, int n, int skip, int k)
{
    int avail = (n - skip) / SP_N;
    if (k > avail)
        k = avail;
    if (k < 1)
        return 0;

    memset(s_acc, 0, sizeof(s_acc));
    sp->t_win = sp->t_fft = sp->t_db = 0;

    for (int seg = 0; seg < k; seg++) {
        uint32_t t0 = now_us();
        const uint16_t *p = x + skip + seg * SP_N;
        uint32_t sum = 0;
        int i;

        for (i = 0; i < SP_N; i++)
            sum += p[i];
        int32_t mean = (int32_t)((sum + SP_N / 2) / SP_N);

        /* 去 DC、<<3 放大到 q15 的上半段、乘窗。(4095·8)·32767 仍在 int32 內。 */
        /* 實數 FFT 的打包：偶數點進實部、奇數點進虛部（見 fft.h） */
        for (i = 0; i < SP_N; i++) {
            int32_t v = (((int32_t)p[i] - mean) * 8 * s_win[i]) >> 15;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            if (i & 1)
                s_im[i >> 1] = (int16_t)v;
            else
                s_re[i >> 1] = (int16_t)v;
        }

        uint32_t t1 = now_us();
        int sh = fft_run(s_re, s_im);
        uint32_t t2 = now_us();

        fft_real_db_acc(s_re, s_im, sh, s_acc);
        uint32_t t3 = now_us();
        sp->t_win += t1 - t0;
        sp->t_fft += t2 - t1;
        sp->t_db += t3 - t2;
    }

    uint32_t t4 = now_us();

    for (int i = 0; i < SP_BINS; i++)
        sp->bin_db[i] = (int16_t)(s_acc[i] / k);

    for (int px = 0; px < SP_W; px++) {
        int b0, b1;
        spectrum_px_bins(px, &b0, &b1);
        int16_t mx = FFT_DB10_MIN;
        for (int b = b0; b < b1; b++)
            if (sp->bin_db[b] > mx)
                mx = sp->bin_db[b];
        sp->raw[px] = mx;

        if (!sp->primed) {
            sp->trace[px] = mx;
            sp->peak[px] = mx;
        } else {
            if (sp->ema_shift)
                sp->trace[px] = (int16_t)(sp->trace[px] +
                                          ((mx - sp->trace[px]) >> sp->ema_shift));
            else
                sp->trace[px] = mx;
            if (mx > sp->peak[px])
                sp->peak[px] = mx;
            else
                sp->peak[px] = (int16_t)(sp->peak[px] - SP_PEAK_DECAY);
        }
    }
    sp->primed = 1;

    /* bin 0..3 是 DC 與窗函數的殘渣，不算進雜訊底線 */
    sp->nf = median_db10(sp->bin_db + 4, SP_BINS - 4);
    sp->k_used = k;
    sp->t_post = now_us() - t4;
    return k;
}
