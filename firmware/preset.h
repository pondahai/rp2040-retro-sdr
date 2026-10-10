/* M4 預設清單：內建幾個台，加上 SD 卡上的 PRESETS.TXT。
 *
 * PRESETS.TXT 一行一個台，可以在電腦上編輯：
 *
 *     # 註解
 *     40.000 CW 500 JJY Fukushima
 *     26.000 AM 8000 1026 kHz alias
 *     68.5 CW BPC
 *
 *   頻率（kHz，最多三位小數）、模式（AM／CW／USB／LSB）、頻寬（Hz，可省略
 *   或給 0 = 該模式的預設；不在該模式的檔位表裡也改用預設）、名稱（其餘全部，
 *   可省略）。看不懂的行跳過，不讓一行打錯就整個檔讀不到。
 *
 * 純 C，不知道板子存在（test_pc.c 驗證）。讀寫檔案是 sketch 的事，這裡只
 * 處理一行字串。
 */
#ifndef PRESET_H
#define PRESET_H

#include <stdint.h>

#define PRESET_MAX  32
#define PRESET_NAME 20

typedef struct {
    int32_t hz;
    int     mode;                    /* DDC_AM … */
    int     bw_hz;
    char    name[PRESET_NAME + 1];
} preset;

typedef struct {
    preset p[PRESET_MAX];
    int    n;
    int    n_builtin;                /* 前 n_builtin 個是內建的 */
} preset_list;

/* 只放內建的台。 */
void preset_init(preset_list *l);

/* 解析一行。成功回 1；空行、註解、格式錯誤回 0。 */
int preset_parse(const char *line, preset *out);

/* 一行文字（不含換行），buf 至少 48 bytes。 */
void preset_format(const preset *p, char *buf, int size);

/* 加到清單尾端。滿了回 0。和現有的同頻同模式就不重複加，回 1。 */
int preset_add(preset_list *l, const preset *p);

/* 畫面用：「40.000 CW JJY Fukushima」 */
void preset_label(const preset *p, char *buf, int size);

#endif /* PRESET_H */
