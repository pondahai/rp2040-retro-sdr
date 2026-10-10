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

/* 本振的 sin 表：1024 點，q10（振幅 1024）。
 *
 * 振幅只給 q10 是為了讓 CIC 第一級能用 int32（見 ddc.h）。雜散的上限
 * 本來就由相位表大小決定（1024 點、10 位元相位 ≈ -60 dBc），q10 的振幅
 * 量化（約 -66 dBc）不會讓它變差。 */
#define NCO_BITS 10
#define NCO_N    (1 << NCO_BITS)
#define NCO_AMP  1024
static int16_t s_sin[NCO_N];
static int s_ready;

/* CIC 輸出 -> FIR 輸入。滿刻度 2^11 × 2^10（本振）× 2^15（兩級 CIC：÷8 長 9 位元、
 * ÷4 長 6 位元）= 2^36，>>9 之後約 2^27，與 fir_sym() 的位元預算一致。 */
#define CIC_SHIFT 9

/* AGC：包絡放開的速度（每個音訊樣本乘一次，約 0.5 s 降 1/e），以及增益上限
 * 對應的最小包絡。沒有訊號時雜訊會被放大到 32767·0.5·(雜訊/AGC_FLOOR)，
 * 這個值要上機聽了再調。 */
#define AGC_DECAY 0.999875f
#define AGC_FLOOR 4000.0f
#define AGC_TARGET 16000.0f
/* AM 依載波定增益（見 ddc_block 的解調段）。AM_FLOOR 是載波的下限：比它弱
 * 的電台增益不再加大 —— REC001 上沒電台的地方 am_dc 約 15000、1026 kHz 混疊
 * 約 106000；下限設 60000，沒電台時才不會把雜訊放得比電台還大。 */
#define AM_TARGET  24000.0f
#define AM_FLOOR   60000.0f
#define AM_LIMIT   30000.0f

uint32_t (*ddc_clock_us)(void);

static uint32_t now_us(void)
{
    return ddc_clock_us ? ddc_clock_us() : 0;
}

/* 對稱 FIR，不用 64 位元乘法。
 *
 * 上機實測原本的寫法（每個係數一次 int64 乘法，M0+ 上是 __aeabi_lmul 函式
 * 呼叫）每個音訊樣本要約 18000 週期，佔 Core 1 七成。兩個改動：
 *
 * 1. 對稱摺疊：h[j]·(x[j] + x[N-1-j])，乘法減半。
 * 2. 樣本（最大約 2^28，摺疊後）拆成高 16 位元與低 16 位元各乘一次：
 *        s·t = (s_hi·t)·2^16 + s_lo·t
 *    s_hi ≤ 2^12，乘上 |t| ≤ 32767 再加總，int32 裝得下；
 *    s_lo 是 0..65535，乘 |t| ≤ 32767 最大 2,147,385,345 —— **剛好**在 int32 內，
 *    加總用 int64（只是加法，adds/adcs 兩條指令）。
 * 結果與 int64 直接乘完全相同，不是近似。 */
static inline int64_t fir_sym(const int32_t *x, const int16_t *taps)
{
    int32_t acc_hi = 0;
    int64_t acc_lo = 0;
    for (int j = 0; j < DDC_TAPS / 2; j++) {
        int32_t sx = x[j] + x[DDC_TAPS - 1 - j];
        int32_t t = taps[j];
        acc_hi += (sx >> 16) * t;
        acc_lo += (int32_t)(sx & 0xFFFF) * t;
    }
    {
        int32_t sx = x[DDC_TAPS / 2];
        int32_t t = taps[DDC_TAPS / 2];
        acc_hi += (sx >> 16) * t;
        acc_lo += (int32_t)(sx & 0xFFFF) * t;
    }
    return ((int64_t)acc_hi << 16) + acc_lo;
}

static const char *const MODE_NAME[DDC_NMODES] = { "AM", "CW", "USB", "LSB" };

static const int BW_AM[]  = { 8000, 6000, 4000, 0 };
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
    return mode == DDC_AM ? 8000 : mode == DDC_CW ? 500 : 2400;
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
            s_sin[i] = (int16_t)lround(sin(2.0 * 3.14159265358979323846 * i / NCO_N) * NCO_AMP);
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
     * 127 階在 15625 Hz 下過渡帶約 400 Hz（7812.5 Hz 時是 200 Hz）。 */
    {
        double fc = bw_hz / 2.0 / afs;        /* 正規化截止頻率（相對取樣率） */
        double h[DDC_TAPS], sum = 0;
        int m = (DDC_TAPS - 1) / 2;
        for (int n = 0; n <= m; n++) {
            int k = n - m;
            double sinc = k ? sin(2.0 * 3.14159265358979323846 * fc * k) /
                              (3.14159265358979323846 * k)
                            : 2.0 * fc;
            double w = 0.54 - 0.46 * cos(2.0 * 3.14159265358979323846 * n / (DDC_TAPS - 1));
            h[n] = h[DDC_TAPS - 1 - n] = sinc * w;
        }
        /* 只算一半再鏡射：ddc_block 的 FIR 靠「完全對稱」把乘法減半，
         * 兩邊各自 lround 可能差 1 LSB，那就不對稱了。 */
        for (int n = 0; n < DDC_TAPS; n++)
            sum += h[n];
        for (int n = 0; n <= m; n++)
            d->taps[n] = d->taps[DDC_TAPS - 1 - n] = (int16_t)lround(h[n] / sum * 32768.0);
    }
}

DDC_RAM_FUNC int ddc_block(ddc *d, const uint16_t *x, int n, int skip, int gap,
                           int16_t *out, int max)
{
    int produced = 0;
    uint32_t t_start = now_us(), t_post = 0;
    uint32_t st = d->lo_step;
    /* 沒看到的那段時間，本振照樣要轉過去 */
    uint32_t ph = d->lo_phase + (uint32_t)(gap + skip) * st;
    d->t_samples += (uint64_t)(gap + skip);
    d->env_n = 0;

    /* 空檔換算成音訊樣本數，先在 out 開頭佔位，整塊做完再內插填上 */
    d->gap_acc += (uint32_t)(gap + skip);
    int fill = (int)(d->gap_acc / DDC_DECIM);
    d->gap_acc %= DDC_DECIM;
    if (fill > DDC_FILL_MAX) fill = DDC_FILL_MAX;
    if (fill > max) fill = max;
    d->bfo_phase += (uint32_t)fill * d->bfo_step;
    produced = fill;
    uint32_t i0 = d->integ_a[0][0], i1 = d->integ_a[0][1], i2 = d->integ_a[0][2];
    uint32_t q0 = d->integ_a[1][0], q1 = d->integ_a[1][1], q2 = d->integ_a[1][2];
    int dec_a = d->dec_a;

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

    uint64_t t_base = d->t_samples;          /* x[0] 的真實取樣時間 */
    for (int k = 0; k < n; k++) {
        int32_t v = (int32_t)x[k] - dc;
        uint32_t idx = ph >> (32 - NCO_BITS);
        int32_t s = s_sin[idx];
        int32_t c = s_sin[(idx + NCO_N / 4) & (NCO_N - 1)];
        ph += st;

        /* x · e^{-jθ} = x·cos − j·x·sin。第一級積分器，int32 模數運算 */
        i0 += (uint32_t)(v * c);  i1 += i0;  i2 += i1;
        q0 -= (uint32_t)(v * s);  q1 += q0;  q2 += q1;

        if (++dec_a < 8)
            continue;
        dec_a = 0;

        /* ---- 每 8 個輸入：第一級梳狀段 -> 第二級積分器（62.5 kHz） ---- */
        uint32_t ua, ub;
        ua = i2 - d->comb_a[0][0]; d->comb_a[0][0] = i2;
        ub = ua - d->comb_a[0][1]; d->comb_a[0][1] = ua;
        ua = ub - d->comb_a[0][2]; d->comb_a[0][2] = ub;
        int32_t ya = (int32_t)ua;
        ua = q2 - d->comb_a[1][0]; d->comb_a[1][0] = q2;
        ub = ua - d->comb_a[1][1]; d->comb_a[1][1] = ua;
        ua = ub - d->comb_a[1][2]; d->comb_a[1][2] = ub;
        int32_t yb = (int32_t)ua;

        d->integ[0][0] += ya; d->integ[0][1] += d->integ[0][0]; d->integ[0][2] += d->integ[0][1];
        d->integ[1][0] += yb; d->integ[1][1] += d->integ[1][0]; d->integ[1][2] += d->integ[1][1];

        if (++d->dec < DDC_DECIM / 8)
            continue;
        d->dec = 0;

        /* ---- 以下每 DDC_DECIM 個輸入跑一次：第二級梳狀段、FIR、解調 ---- */
        uint32_t tp = now_us();
        int64_t ci, cq, t;
        int64_t i2b = d->integ[0][2], q2b = d->integ[1][2];
        t = i2b - d->comb[0][0]; d->comb[0][0] = i2b; ci = t;
        t = ci - d->comb[0][1]; d->comb[0][1] = ci; ci = t;
        t = ci - d->comb[0][2]; d->comb[0][2] = ci; ci = t;
        t = q2b - d->comb[1][0]; d->comb[1][0] = q2b; cq = t;
        t = cq - d->comb[1][1]; d->comb[1][1] = cq; cq = t;
        t = cq - d->comb[1][2]; d->comb[1][2] = cq; cq = t;

        int hp = d->hpos;
        d->hist[0][hp] = d->hist[0][hp + DDC_TAPS] = (int32_t)(ci >> CIC_SHIFT);
        d->hist[1][hp] = d->hist[1][hp + DDC_TAPS] = (int32_t)(cq >> CIC_SHIFT);
        d->hpos = (hp + 1 == DDC_TAPS) ? 0 : hp + 1;

        /* hist[hp+1 .. hp+TAPS] 是最舊到最新；FIR 對稱，方向無所謂 */
        int64_t ai = fir_sym(&d->hist[0][hp + 1], d->taps);
        int64_t aq = fir_sym(&d->hist[1][hp + 1], d->taps);
        float zi = (float)(ai >> 15), zq = (float)(aq >> 15);
        if (d->conj)
            zq = -zq;

        float pw = zi * zi + zq * zq;
        d->level += (pw - d->level) * 0.01f;

        d->env_acc += pw;
        if (++d->env_cnt == DDC_ENV_DECIM) {
            if (d->env_n < DDC_ENV_MAX) {
                d->env[d->env_n] = d->env_acc / DDC_ENV_DECIM;
                /* 這一點代表過去 64 個音訊樣本，時間取區間中點 */
                d->env_ms[d->env_n] = (uint32_t)((t_base + k + 1 - DDC_DECIM * DDC_ENV_DECIM / 2) / 500);
                d->env_n++;
            }
            d->env_acc = 0;
            d->env_cnt = 0;
        }

        float y;
        if (d->mode == DDC_AM) {
            float mag = sqrtf(pw);
            d->am_dc += (mag - d->am_dc) * 0.001f;     /* τ ≈ 64 ms */
            y = mag - d->am_dc;
        } else {
            /* Re{(zi + j·zq)·e^{+jφ}} = zi·cosφ − zq·sinφ */
            uint32_t bi = d->bfo_phase >> (32 - NCO_BITS);
            float bs = s_sin[bi] * (1.0f / NCO_AMP);
            float bc = s_sin[(bi + NCO_N / 4) & (NCO_N - 1)] * (1.0f / NCO_AMP);
            d->bfo_phase += d->bfo_step;
            y = zi * bc - zq * bs;
        }

        float o;
        if (d->mode == DDC_AM) {
            /* AM：增益跟著載波（am_dc），不跟著音訊。載波不隨節目內容變，
             * 所以字與字之間的停頓不會被拉高、調變的大小聲原樣保留，
             * 只有衰落才會改變增益。100% 調變時 |y| ≈ 載波 -> AM_TARGET。 */
            float c = d->am_dc > AM_FLOOR ? d->am_dc : AM_FLOOR;
            o = y * (AM_TARGET / c);
            /* 限幅：只有快削頂時才壓。電台的峰值碰不到（REC001 的 1026 kHz
             * 峰值約 28500），沒有載波時的雜訊（等於 100% 調變）會被它擋住。 */
            float a = o < 0 ? -o : o;
            d->agc_env = a > d->agc_env ? a : d->agc_env * AGC_DECAY;
            if (d->agc_env > AM_LIMIT)
                o *= AM_LIMIT / d->agc_env;
        } else {
            /* CW／SSB 沒有載波：快攻慢放。包絡有下限，沒訊號時不會把雜訊放到滿。 */
            float a = y < 0 ? -y : y;
            d->agc_env = a > d->agc_env ? a : d->agc_env * AGC_DECAY;
            float env = d->agc_env > AGC_FLOOR ? d->agc_env : AGC_FLOOR;
            o = y * (AGC_TARGET / env);
        }
        if (o > 32767.0f) o = 32767.0f;
        if (o < -32767.0f) o = -32767.0f;

        if (produced < max)
            out[produced++] = (int16_t)o;
        t_post += now_us() - tp;
    }

    if (fill) {
        int32_t a = d->last_out, b = produced > fill ? out[fill] : a;
        for (int k = 0; k < fill; k++)
            out[k] = (int16_t)(a + (b - a) * (k + 1) / (fill + 1));
    }
    if (produced)
        d->last_out = out[produced - 1];

    d->lo_phase = ph;
    d->t_samples = t_base + (uint64_t)n;
    d->integ_a[0][0] = i0; d->integ_a[0][1] = i1; d->integ_a[0][2] = i2;
    d->integ_a[1][0] = q0; d->integ_a[1][1] = q1; d->integ_a[1][2] = q2;
    d->dec_a = dec_a;
    d->t_post = t_post;
    d->t_total = now_us() - t_start;
    return produced;
}
