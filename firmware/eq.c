/* eq.c — 見 eq.h。濾波器公式：RBJ Audio EQ Cookbook。 */
#include "eq.h"
#include <math.h>
#include <string.h>

#define Q 13

static const char *const NAMES[EQ_NPRESETS] = { "FLAT", "SPK", "SPK+", "VOICE" };

const char *eq_name(int preset)
{
    return preset >= 0 && preset < EQ_NPRESETS ? NAMES[preset] : "?";
}

static void store(eq_bq *q, double b0, double b1, double b2,
                  double a0, double a1, double a2)
{
    memset(q, 0, sizeof *q);
    q->b0 = (int32_t)lround(b0 / a0 * (1 << Q));
    q->b1 = (int32_t)lround(b1 / a0 * (1 << Q));
    q->b2 = (int32_t)lround(b2 / a0 * (1 << Q));
    q->a1 = (int32_t)lround(a1 / a0 * (1 << Q));
    q->a2 = (int32_t)lround(a2 / a0 * (1 << Q));
}

static void highpass(eq_bq *q, double f, double fs)
{
    double w = 2 * 3.14159265358979323846 * f / fs, c = cos(w);
    double al = sin(w) / (2 * 0.7071);
    store(q, (1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
}

static void lowpass(eq_bq *q, double f, double fs)
{
    double w = 2 * 3.14159265358979323846 * f / fs, c = cos(w);
    double al = sin(w) / (2 * 0.7071);
    store(q, (1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
}

static void peak(eq_bq *q, double f, double db, double qf, double fs)
{
    double w = 2 * 3.14159265358979323846 * f / fs, c = cos(w);
    double A = pow(10.0, db / 40.0), al = sin(w) / (2 * qf);
    store(q, 1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A);
}

void eq_set(eq *e, int preset, double fs)
{
    if (preset < 0 || preset >= EQ_NPRESETS)
        preset = 0;
    e->preset = preset;
    e->nbq = 0;
    switch (preset) {
    case 1:
        highpass(&e->bq[e->nbq++], 300, fs);
        peak(&e->bq[e->nbq++], 2000, 6, 1.0, fs);
        break;
    case 2:
        highpass(&e->bq[e->nbq++], 400, fs);
        peak(&e->bq[e->nbq++], 2500, 9, 0.9, fs);
        break;
    case 3:
        highpass(&e->bq[e->nbq++], 300, fs);
        peak(&e->bq[e->nbq++], 2000, 6, 1.0, fs);
        lowpass(&e->bq[e->nbq++], 3200, fs);
        break;
    default:
        break;
    }
}

void eq_run(eq *e, int16_t *x, int n)
{
    for (int k = 0; k < e->nbq; k++) {
        eq_bq *q = &e->bq[k];
        int32_t x1 = q->x1, x2 = q->x2, y1 = q->y1, y2 = q->y2;
        for (int i = 0; i < n; i++) {
            int32_t v = x[i];
            int32_t acc = q->b0 * v + q->b1 * x1 + q->b2 * x2
                        - q->a1 * y1 - q->a2 * y2;
            int32_t y = (acc + (1 << (Q - 1))) >> Q;
            if (y > 32767) y = 32767;       /* 提高的那段可能削頂：音量轉小一點 */
            if (y < -32767) y = -32767;
            x2 = x1; x1 = v;
            y2 = y1; y1 = y;
            x[i] = (int16_t)y;
        }
        q->x1 = x1; q->x2 = x2; q->y1 = y1; q->y2 = y2;
    }
}
