/* 8x8 鍵盤矩陣 -> 按鍵事件。
 *
 * 純 C，不碰 GPIO：輸入是掃描結果（8 個 byte，每列一個 bit mask），
 * 輸出是事件。板子上餵 74HC165 讀回來的東西，PC 上餵測試腳本 ——
 * 去彈跳、修飾鍵、連發這些真正容易寫錯的邏輯因此都能在 PC 上測。
 *
 * 對照表用 PicoApple2-KeyboardTester README 的**實測真值表**，
 * 不是 PicoApple2.ino 的 keymap_base（那張表有 4 組錯位與 3 個未定義鍵，
 * 見 docs/PLAN.md §2.2）。
 */
#ifndef KEYS_H
#define KEYS_H

#include <stdint.h>

/* 可列印的鍵直接用 ASCII。以下是其餘的。 */
#define KEY_NONE   0x00
#define KEY_BS     0x08
#define KEY_TAB    0x09
#define KEY_ENTER  0x0D
#define KEY_ESC    0x1B
#define KEY_DEL    0x7F
#define KEY_UP     0x81
#define KEY_DOWN   0x82
#define KEY_LEFT   0x83
#define KEY_RIGHT  0x84
#define KEY_PGUP   0x85
#define KEY_PGDN   0x86
#define KEY_F1     0x91          /* F1..F10 連號 */
#define KEY_F2     0x92
#define KEY_F3     0x93
#define KEY_F9     0x99
#define KEY_F10    0x9A

/* 修飾鍵位元 */
#define KEY_M_SHIFT 0x01
#define KEY_M_CTRL  0x02
#define KEY_M_ALT   0x04
#define KEY_M_FN    0x08
#define KEY_M_CAPS  0x10

#define KEYS_DEBOUNCE_MS 30      /* 與 PicoApple2.ino 同規格 */
#define KEYS_REPEAT_MS   400     /* 按住多久開始連發 */
#define KEYS_RATE_MS     60      /* 連發間隔 */
#define KEYS_MAX_EVENTS  8

typedef struct {
    uint8_t code;
    uint8_t mods;
    uint8_t repeat;              /* 1 = 連發出來的，不是新按下 */
} key_event;

typedef struct {
    uint8_t  stable[8];          /* 去彈跳後的狀態，每列 8 個 bit */
    uint8_t  pending[8];         /* 上一次掃到的原始狀態 */
    uint8_t  fresh[8];           /* 這一輪剛從放開變成按下的格子 */
    uint32_t changed_ms[64];     /* 每一格最後一次變動的時間 */
    uint8_t  caps;
    uint8_t  repeat_rc;          /* 目前在連發的那一格，0xFF = 沒有 */
    uint32_t repeat_at;
    uint32_t last_ms;
} keys;

void keys_init(keys *k);

/* 餵一次掃描結果。回傳填進 out 的事件數（0..max）。
 * rows[r] 的 bit c = 1 表示 [r][c] 被按住。 */
int keys_update(keys *k, const uint8_t rows[8], uint32_t now_ms,
                key_event *out, int max);

/* [r][c] 對應的鍵碼。修飾鍵本身回 0 —— 它們只改 mods，不產生事件。
 * 給自我檢查用：64 格必須一對一填滿。 */
uint8_t keys_code(int row, int col, int shift);
/* 這一格是哪個修飾鍵（KEY_M_*），不是修飾鍵回 0。 */
uint8_t keys_mod(int row, int col);
/* 這一格會不會受 CapsLock 影響（只有字母會）。 */
int keys_is_alpha(int row, int col);

#endif /* KEYS_H */
