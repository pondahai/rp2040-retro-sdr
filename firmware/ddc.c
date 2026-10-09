/* 板子上 -O3 並把熱迴圈放進 RAM，理由同 fft.c。 */
#if defined(ARDUINO_ARCH_RP2040)
#pragma GCC optimize("O3")
#define DDC_RAM_FUNC __attribute__((noinline, section(".time_critical.ddc_block")))
#else
#define DDC_RAM_FUNC
#endif

#include "ddc.h"

#include <math.h>
#include <string.h>

/* 本振的 sin 表：1024 點，q14（振幅 16384）。q14 而不是 q15，是為了讓
 * 12 位元輸入 × 本振 = 26 位元，CIC 長完 18 位元正好 44，int64 綽綽有餘。
 * 表的量化造成的雜散約 -60 dBc 以下，對 12 位元 ADC 夠用。 */
#define NCO_BITS 10
#define NCO_N    (1 << NCO_BITS)
static int16_t s_sin[NCO_N];
static int s_ready;

/* CIC 輸出 44 位元 -> FIR 輸入。>>16 之後滿刻度約 2^27，FIR 累加用 int64。 */
#define CIC_SHIFT 16

/* AGC：包絡放開的速度（每個音訊樣本乘一次，約 0.5 s 降 1/e），以及增益上限
 * 對應的最小包絡。沒有訊號時雜訊會被放大到 32767·0.5·(雜訊/AGC_FLOOR)，
 * 這個值要上機聽了再調。 */
#define AGC_DECAY 0.99975f
#define AGC_FLOOR 4000.0f
#define AGC_TARGET 16000.0f

static const char *const MODE_NAME[DDC_NMODES] = { "AM", "CW", "USB", "LSB" };

static const int BW_AM[]  = { 4000, 6000, 0 };
static const int BW_CW[]  = { 250, 500, 1000, 0 };
static const int BW_SSB[] = { 1800, 2400, 2700, 0 };

static const int *bw_table(int mode)
{
    switch (mode) {
    case DDC_AM: return BW_AM;
    case DDC_CW: return BW_CW;
    default:     return BW_SSB;
    }
}

const char *ddc_mode_name(int mode)
{
    return (mode >= 0 && mode < DDC_NMODES) ? MODE_NAME[mode] : "?";
}

int ddc_default_bw(int mode)
{
    return mode == DDC_AM ? 6000 : mode == DDC_CW ? 500 : 2400;
}

int ddc_next_bw(int mode, int bw_hz)
{
    const int *t = bw_table(mode);
    for (int i = 0; t[i]; i++)
        if (t[i] == bw_hz)
            return t[i + 1] ? t[i + 1] : t[0];
    return ddc_default_bw(mode);
}

void ddc_init(ddc *d)
{
    if (!s_ready) {
        for (int i = 0; i < NCO_N; i++)
            s_sin[i] = (int16_t)lround(sin(2.0 * 3.14159265358979323846 * i / NCO_N) * 16384.0);
        s_ready = 1;
    }
    memset(d, 0, sizeof(*d));
    ddc_set(d, 68500, DDC_CW, ddc_default_bw(DDC_CW));
}

/* f Hz 在取樣率 fs 下的相位步進（2^32 = 一圈）。f 可以是負的。 */
static uint32_t step_for(double f, double fs)
{
    double s = f / fs * 4294967296.0;
    return (uint32_t)(int64_t)llround(s);
}

void ddc_set(ddc *d, int32_t tune_hz, int mode, int bw_hz)
{
    const double afs = DDC_AFS_X2 / 2.0;
    int center = 0;

    d->tune_hz = tune_hz;
    d->mode = mode;
    d->bw_hz = bw_hz;
    d->conj = 0;

    /* 本振與 BFO 的位置（見 ddc.h 的表） */
    switch (mode) {
    case DDC_CW:
        d->lo_step = step_for(tune_hz, DDC_FS);
        d->bfo_step = step_for(DDC_CW_PITCH, afs);
        break;
    case DDC_USB:
        center = 150 + bw_hz / 2;
        d->lo_step = step_for(tune_hz + center, DDC_FS);
        d->bfo_step = step_for(center, afs);
        break;
    case DDC_LSB:
        center = 150 + bw_hz / 2;
        d->lo_step = step_for(tune_hz - center, DDC_FS);
        d->bfo_step = step_for(center, afs);
        d->conj = 1;
        break;
    default:                                  /* AM */
        d->lo_step = step_for(tune_hz, DDC_FS);
        d->bfo_step = 0;
        break;
    }

    /* 通道濾波器：Hamming 窗 sinc 低通，截止 bw/2，係數和 = 32768（q15 增益 1）。
     * 127 階在 7812.5 Hz 下過渡帶約 200 Hz，所以 CW 最窄給到 250 Hz。 */
    {
        double fc = bw_hz / 2.0 / afs;        /* 正規化截止頻率（相對取樣率） */
        double h[DDC_TAPS], sum = 0;
        int m = (DDC_TAPS - 1) / 2;
        for (int n = 0; n < DDC_TAPS; n++) {
            int k = n - m;
            double sinc = k ? sin(2.0 * 3.14159265358979323846 * fc * k) /
                              (3.14159265358979323846 * k)
                            : 2.0 * fc;
            double w = 0.54 - 0.46 * cos(2.0 * 3.14159265358979323846 * n / (DDC_TAPS - 1));
            h[n] = sinc * w;
            sum += h[n];
        }
        for (int n = 0; n < DDC_TAPS; n++)
            d->taps[n] = (int16_t)lround(h[n] / sum * 32768.0);
    }
}

DDC_RAM_FUNC int ddc_block(ddc *d, const uint16_t *x, int n, int skip, int gap,
                           int16_t *out, int max)
{
    int produced = 0;
    uint32_t st = d->lo_step;
    /* 沒看到的那段時間，本振照樣要轉過去 */
    uint32_t ph = d->lo_phase + (uint32_t)(gap + skip) * st;
    int64_t i0 = d->integ[0][0], i1 = d->integ[0][1], i2 = d->integ[0][2];
    int64_t q0 = d->integ[1][0], q1 = d->integ[1][1], q2 = d->integ[1][2];
    int dec = d->dec;

    if (skip > n)
        skip = n;
    x += skip;
    n -= skip;

    /* DC 用這一塊的平均扣：偏壓在 0.41 V（約 508），不是 ADC 中點 2048。
     * 扣錯不會進到通帶（DC 混頻後在 −f_LO），但會吃掉 CIC 的動態範圍。 */
    uint32_t sum = 0;
    for (int k = 0; k < n; k++)
        sum += x[k];
    int32_t dc = n ? (int32_t)((sum + n / 2) / n) : 2048;

    for (int k = 0; k < n; k++) {
        int32_t v = (int32_t)x[k] - dc;
        uint32_t idx = ph >> (32 - NCO_BITS);
        int32_t s = s_sin[idx];
        int32_t c = s_sin[(idx + NCO_N / 4) & (NCO_N - 1)];
        ph += st;

        /* x · e^{-jθ} = x·cos − j·x·sin */
        i0 += v * c;  i1 += i0;  i2 += i1;
        q0 -= v * s;  q1 += q0;  q2 += q1;

        if (++dec < DDC_DECIM)
            continue;
        dec = 0;

        /* ---- 以下每 64 個輸入跑一次：CIC 的梳狀段、FIR、解調 ---- */
        int64_t ci, cq, t;
        t = i2 - d->comb[0][0]; d->comb[0][0] = i2; ci = t;
        t = ci - d->comb[0][1]; d->comb[0][1] = ci; ci = t;
        t = ci - d->comb[0][2]; d->comb[0][2] = ci; ci = t;
        t = q2 - d->comb[1][0]; d->comb[1][0] = q2; cq = t;
        t = cq - d->comb[1][1]; d->comb[1][1] = cq; cq = t;
        t = cq - d->comb[1][2]; d->comb[1][2] = cq; cq = t;

        int hp = d->hpos;
        d->hist[0][hp] = d->hist[0][hp + DDC_TAPS] = (int32_t)(ci >> CIC_SHIFT);
        d->hist[1][hp] = d->hist[1][hp + DDC_TAPS] = (int32_t)(cq >> CIC_SHIFT);
        d->hpos = (hp + 1 == DDC_TAPS) ? 0 : hp + 1;

        /* hist[hp+1 .. hp+TAPS] 是最舊到最新；FIR 對稱，方向無所謂 */
        const int32_t *hi = &d->hist[0][hp + 1], *hq = &d->hist[1][hp + 1];
        int64_t ai = 0, aq = 0;
        for (int j = 0; j < DDC_TAPS; j++) {
            ai += (int64_t)hi[j] * d->taps[j];
            aq += (int64_t)hq[j] * d->taps[j];
        }
        float zi = (float)(ai >> 15), zq = (float)(aq >> 15);
        if (d->conj)
            zq = -zq;

        float pw = zi * zi + zq * zq;
        d->level += (pw - d->level) * 0.01f;

        float y;
        if (d->mode == DDC_AM) {
            float mag = sqrtf(pw);
            d->am_dc += (mag - d->am_dc) * 0.002f;
            y = mag - d->am_dc;
        } else {
            /* Re{(zi + j·zq)·e^{+jφ}} = zi·cosφ − zq·sinφ */
            uint32_t bi = d->bfo_phase >> (32 - NCO_BITS);
            float bs = s_sin[bi] * (1.0f / 16384.0f);
            float bc = s_sin[(bi + NCO_N / 4) & (NCO_N - 1)] * (1.0f / 16384.0f);
            d->bfo_phase += d->bfo_step;
            y = zi * bc - zq * bs;
        }

        /* AGC：快攻慢放。包絡有下限，沒訊號時不會把雜訊放到滿。 */
        float a = y < 0 ? -y : y;
        d->agc_env = a > d->agc_env ? a : d->agc_env * AGC_DECAY;
        float env = d->agc_env > AGC_FLOOR ? d->agc_env : AGC_FLOOR;
        float o = y * (AGC_TARGET / env);
        if (o > 32767.0f) o = 32767.0f;
        if (o < -32767.0f) o = -32767.0f;

        if (produced < max)
            out[produced++] = (int16_t)o;
    }

    d->lo_phase = ph;
    d->integ[0][0] = i0; d->integ[0][1] = i1; d->integ[0][2] = i2;
    d->integ[1][0] = q0; d->integ[1][1] = q1; d->integ[1][2] = q2;
    d->dec = dec;
    return produced;
}
