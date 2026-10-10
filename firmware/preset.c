/* 預設清單，見 preset.h。 */
#include "preset.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "ddc.h"

static const preset BUILTIN[] = {
    { 40000, DDC_CW, 500, "JJY 40 (Fukushima)" },
    { 60000, DDC_CW, 500, "JJY 60 (Kyushu)" },
    { 68500, DDC_CW, 500, "BPC (Shangqiu)" },
    { 68493, DDC_CW, 500, "Test tone (T key)" },
};
#define N_BUILTIN ((int)(sizeof BUILTIN / sizeof BUILTIN[0]))

void preset_init(preset_list *l)
{
    memset(l, 0, sizeof(*l));
    memcpy(l->p, BUILTIN, sizeof BUILTIN);
    l->n = l->n_builtin = N_BUILTIN;
}

/* bw 在不在這個模式的檔位表裡：從預設開始一路 next，繞回來就是沒有 */
static int bw_valid(int mode, int bw)
{
    int first = ddc_default_bw(mode), b = first;
    do {
        if (b == bw)
            return 1;
        b = ddc_next_bw(mode, b);
    } while (b != first);
    return 0;
}

static const char *skip_ws(const char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    return s;
}

/* kHz 字串 -> Hz，最多三位小數，不用浮點。成功回讀到的字元數，失敗回 0 */
static int parse_khz(const char *s, int32_t *hz)
{
    const char *p = s;
    int32_t ip = 0, fp = 0;
    int nd = 0, nf = 0;
    while (isdigit((unsigned char)*p) && nd < 4)
        ip = ip * 10 + (*p++ - '0'), nd++;
    if (!nd)
        return 0;
    if (*p == '.') {
        p++;
        while (isdigit((unsigned char)*p)) {
            if (++nf > 3)
                return 0;
            fp = fp * 10 + (*p++ - '0');
        }
    }
    while (nf < 3)
        fp *= 10, nf++;
    if (*p && *p != ' ' && *p != '\t')
        return 0;
    *hz = ip * 1000 + fp;
    return (int)(p - s);
}

int preset_parse(const char *line, preset *out)
{
    preset p;
    memset(&p, 0, sizeof p);
    const char *s = skip_ws(line);
    if (!*s || *s == '#' || *s == '\r' || *s == '\n')
        return 0;

    int k = parse_khz(s, &p.hz);
    if (!k || p.hz <= 0 || p.hz > 250000)
        return 0;
    s = skip_ws(s + k);

    p.mode = -1;
    for (int m = 0; m < DDC_NMODES; m++) {
        const char *nm = ddc_mode_name(m);
        size_t n = strlen(nm), i;
        for (i = 0; i < n && toupper((unsigned char)s[i]) == nm[i]; i++)
            ;
        if (i == n && (s[n] == 0 || s[n] == ' ' || s[n] == '\t' || s[n] == '\r' || s[n] == '\n')) {
            p.mode = m;
            s = skip_ws(s + n);
            break;
        }
    }
    if (p.mode < 0)
        return 0;

    /* 頻寬可省略：數字後面接空白或行尾才算 */
    p.bw_hz = 0;
    if (isdigit((unsigned char)*s)) {
        const char *q = s;
        int bw = 0;
        while (isdigit((unsigned char)*q) && bw < 100000)
            bw = bw * 10 + (*q++ - '0');
        if (*q == 0 || *q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') {
            p.bw_hz = bw;
            s = skip_ws(q);
        }
    }
    if (!p.bw_hz || !bw_valid(p.mode, p.bw_hz))
        p.bw_hz = ddc_default_bw(p.mode);

    /* 名稱：其餘全部，去掉行尾的換行與空白 */
    size_t n = strlen(s);
    while (n && (s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == ' ' || s[n - 1] == '\t'))
        n--;
    if (n > PRESET_NAME)
        n = PRESET_NAME;
    memcpy(p.name, s, n);
    p.name[n] = 0;

    *out = p;
    return 1;
}

void preset_format(const preset *p, char *buf, int size)
{
    snprintf(buf, (size_t)size, "%ld.%03ld %s %d%s%s", (long)(p->hz / 1000), (long)(p->hz % 1000),
             ddc_mode_name(p->mode), p->bw_hz, p->name[0] ? " " : "", p->name);
}

int preset_add(preset_list *l, const preset *p)
{
    for (int i = 0; i < l->n; i++)
        if (l->p[i].hz == p->hz && l->p[i].mode == p->mode)
            return 1;
    if (l->n >= PRESET_MAX)
        return 0;
    l->p[l->n++] = *p;
    return 1;
}

void preset_label(const preset *p, char *buf, int size)
{
    snprintf(buf, (size_t)size, "%ld.%03ld %s %s", (long)(p->hz / 1000), (long)(p->hz % 1000),
             ddc_mode_name(p->mode), p->name);
}
