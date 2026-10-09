#include "keys.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * 實測真值表（PicoApple2-KeyboardTester README「完整真值表」那一段）。
 *
 * **不要拿 PicoApple2.ino 的 keymap_base 來對**，那張表與硬體佈線有 4 組
 * 錯位（S/X 對調、`/`↔PGUP、`=`↔PGDN）與 3 個未定義鍵（TAB/CAPS/DEL）。
 * 遊戲很少按 `/`，查字典會一直按 —— 所以這裡從一開始就用量出來的那張。
 *
 * 這四張表是從 README 的真值表機械轉出來的，並由 test_keys 檢查 64 格
 * 一對一填滿、修飾鍵恰好 5 個、可列印鍵的 shift 版本沒有重複漏字。
 * ------------------------------------------------------------------------- */
static const uint8_t BASE[8][8] = {
    { '1'        , '3'        , '5'        , '7'        , '9'        , '-'        , KEY_TAB    , KEY_BS },   /* r0 */
    { 'q'        , 'e'        , 't'        , 'u'        , 'o'        , '['        , KEY_ESC    , '\\' },   /* r1 */
    { 'a'        , 'd'        , 'g'        , 'j'        , 'l'        , '\''       , 0          , 0 },   /* r2 */
    { 'z'        , 'c'        , 'b'        , 'm'        , '.'        , 0          , KEY_DOWN   , 0 },   /* r3 */
    { '2'        , '4'        , '6'        , '8'        , '0'        , '='        , '`'        , KEY_DEL },   /* r4 */
    { 'w'        , 'r'        , 'y'        , 'i'        , 'p'        , ']'        , KEY_UP     , KEY_PGUP },   /* r5 */
    { 's'        , 'f'        , 'h'        , 'k'        , ';'        , KEY_ENTER  , KEY_RIGHT  , KEY_PGDN },   /* r6 */
    { 'x'        , 'v'        , 'n'        , ','        , '/'        , ' '        , KEY_LEFT   , 0 },   /* r7 */
};

static const uint8_t SHIFTED[8][8] = {
    { '!'        , '#'        , '%'        , '&'        , '('        , '_'        , KEY_TAB    , KEY_BS },   /* r0 */
    { 'Q'        , 'E'        , 'T'        , 'U'        , 'O'        , '{'        , KEY_ESC    , '|' },   /* r1 */
    { 'A'        , 'D'        , 'G'        , 'J'        , 'L'        , '"'        , 0          , 0 },   /* r2 */
    { 'Z'        , 'C'        , 'B'        , 'M'        , '>'        , 0          , KEY_DOWN   , 0 },   /* r3 */
    { '@'        , '$'        , '^'        , '*'        , ')'        , '+'        , '~'        , KEY_DEL },   /* r4 */
    { 'W'        , 'R'        , 'Y'        , 'I'        , 'P'        , '}'        , KEY_UP     , KEY_PGUP },   /* r5 */
    { 'S'        , 'F'        , 'H'        , 'K'        , ':'        , KEY_ENTER  , KEY_RIGHT  , KEY_PGDN },   /* r6 */
    { 'X'        , 'V'        , 'N'        , '<'        , '?'        , ' '        , KEY_LEFT   , 0 },   /* r7 */
};

static const uint8_t MOD[8][8] = {
    { 0          , 0          , 0          , 0          , 0          , 0          , 0          , 0 },   /* r0 */
    { 0          , 0          , 0          , 0          , 0          , 0          , 0          , 0 },   /* r1 */
    { 0          , 0          , 0          , 0          , 0          , 0          , KEY_M_CTRL , KEY_M_CAPS },   /* r2 */
    { 0          , 0          , 0          , 0          , 0          , KEY_M_SHIFT, 0          , KEY_M_FN },   /* r3 */
    { 0          , 0          , 0          , 0          , 0          , 0          , 0          , 0 },   /* r4 */
    { 0          , 0          , 0          , 0          , 0          , 0          , 0          , 0 },   /* r5 */
    { 0          , 0          , 0          , 0          , 0          , 0          , 0          , 0 },   /* r6 */
    { 0          , 0          , 0          , 0          , 0          , 0          , 0          , KEY_M_ALT },   /* r7 */
};

static const uint8_t ALPHA[8][8] = {
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1, 1, 1, 1, 1, 0, 0, 0 },
    { 1, 1, 1, 1, 1, 0, 0, 0 },
    { 1, 1, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1, 1, 1, 1, 1, 0, 0, 0 },
    { 1, 1, 1, 1, 0, 0, 0, 0 },
    { 1, 1, 1, 0, 0, 0, 0, 0 },
};

uint8_t keys_code(int row, int col, int shift)
{
    return shift ? SHIFTED[row][col] : BASE[row][col];
}

uint8_t keys_mod(int row, int col)
{
    return MOD[row][col];
}

int keys_is_alpha(int row, int col)
{
    return ALPHA[row][col];
}

void keys_init(keys *k)
{
    memset(k, 0, sizeof(*k));
    k->repeat_rc = 0xFF;
}

/* 這些鍵是**命令**不是文字，連發沒有意義而且會出事：Fn+1 是發音，連發會
 * 讓一個字被打斷重唸好幾次（實機症狀是「ap ap approach」）。手指按著組合鍵
 * 本來就會比按字母久，再加上唸一次要先合成再重畫，很容易就超過連發門檻。
 * 方向鍵與退格則相反 —— 那兩個沒有連發會很難用。 */
static int is_command(uint8_t code)
{
    return (code >= KEY_F1 && code <= KEY_F10) ||
           code == KEY_ENTER || code == KEY_ESC || code == KEY_TAB;
}

/* FN + 數字 = F1..F10（生態系既有慣例，見 PLAN.md §2.2）。 */
static uint8_t fn_translate(uint8_t code)
{
    if (code >= '1' && code <= '9')
        return (uint8_t)(KEY_F1 + (code - '1'));
    if (code == '0')
        return KEY_F10;
    return code;
}

static int emit(key_event *out, int max, int n, uint8_t code, uint8_t mods,
                int repeat)
{
    if (n >= max)
        return n;               /* 佇列滿了就丟掉 —— 掃描頻率遠高於人手 */
    out[n].code = code;
    out[n].mods = mods;
    out[n].repeat = (uint8_t)repeat;
    return n + 1;
}

int keys_update(keys *k, const uint8_t rows[8], uint32_t now_ms,
                key_event *out, int max)
{
    int r, c, n = 0;
    uint8_t mods = 0;

    memset(k->fresh, 0, sizeof(k->fresh));

    /* --- 去彈跳：狀態要連續穩定 KEYS_DEBOUNCE_MS 才算數 --- */
    for (r = 0; r < 8; r++) {
        for (c = 0; c < 8; c++) {
            int rc = r * 8 + c;
            uint8_t now = (uint8_t)((rows[r] >> c) & 1);
            uint8_t was = (uint8_t)((k->pending[r] >> c) & 1);
            uint8_t stable = (uint8_t)((k->stable[r] >> c) & 1);
            if (now != was) {
                k->changed_ms[rc] = now_ms;
            } else if (now != stable &&
                       now_ms - k->changed_ms[rc] >= KEYS_DEBOUNCE_MS) {
                k->stable[r] = (uint8_t)(now ? (k->stable[r] | (1 << c))
                                             : (k->stable[r] & ~(1 << c)));
                if (now)
                    k->fresh[r] |= (uint8_t)(1 << c);
            }
        }
        k->pending[r] = rows[r];
    }

    /* --- 修飾鍵先算完，才知道同一次掃描裡的字母該不該大寫 --- */
    for (r = 0; r < 8; r++)
        for (c = 0; c < 8; c++)
            if (MOD[r][c] && ((k->stable[r] >> c) & 1))
                mods |= MOD[r][c];

    /* CapsLock 是切換不是按住：剛按下的那一瞬間翻面。 */
    for (r = 0; r < 8; r++)
        for (c = 0; c < 8; c++)
            if (MOD[r][c] == KEY_M_CAPS && ((k->fresh[r] >> c) & 1))
                k->caps = (uint8_t)!k->caps;
    mods = (uint8_t)((mods & ~KEY_M_CAPS) | (k->caps ? KEY_M_CAPS : 0));

    /* --- 一般鍵：新按下的送事件，按住的到時間就連發 --- */
    for (r = 0; r < 8; r++) {
        for (c = 0; c < 8; c++) {
            int rc = r * 8 + c;
            int down = (k->stable[r] >> c) & 1;
            /* **連發要看原始掃描值，不是去彈跳後的值。**
             *
             * 釋放要兩次掃描才會反映到 stable（第一次只記時間，第二次過了
             * KEYS_DEBOUNCE_MS 才翻）。掃描是每個主迴圈一次，而打一個字母
             * 要 8 筆隨機 SD 讀加重畫 —— 一圈可能 200ms，於是 stable 會在
             * 使用者早就放開之後還維持 1 長達數百毫秒，連發就在那時候觸發。
             * 實機症狀是打 m 變 mm、接著打 a 變 aa、連 BACK 都一次刪兩個。
             *
             * 去彈跳是為了濾掉假的**邊緣**；「現在還按著嗎」這個問題，最新
             * 的原始取樣才是最可信的證據。偶爾漏讀一次只會延後連發，無害。 */
            int raw_down = (k->pending[r] >> c) & 1;
            uint8_t code;
            int shift;

            if (MOD[r][c])
                continue;
            shift = (mods & KEY_M_SHIFT) ? 1 : 0;
            /* CapsLock 只影響字母，不影響數字與標點 —— 打 `1` 不會變 `!`。 */
            if (ALPHA[r][c] && (mods & KEY_M_CAPS))
                shift = !shift;
            code = keys_code(r, c, shift);
            if (!code)
                continue;
            if (mods & KEY_M_FN)
                code = fn_translate(code);

            if ((k->fresh[r] >> c) & 1) {
                n = emit(out, max, n, code, mods, 0);
                k->repeat_rc = (uint8_t)rc;
                k->repeat_at = now_ms + KEYS_REPEAT_MS;
            } else if (down && raw_down && k->repeat_rc == rc &&
                       !is_command(code) &&
                       (int32_t)(now_ms - k->repeat_at) >= 0) {
                n = emit(out, max, n, code, mods, 1);
                k->repeat_at = now_ms + KEYS_RATE_MS;
            }
        }
    }

    /* 正在連發的那顆放開了就停 —— 不論放開的是不是同一顆。 */
    if (k->repeat_rc != 0xFF) {
        int rc = k->repeat_rc;
        if (!((k->stable[rc >> 3] >> (rc & 7)) & 1) ||
            !((k->pending[rc >> 3] >> (rc & 7)) & 1)) {
            k->repeat_rc = 0xFF;
        }
    }

    k->last_ms = now_ms;
    return n;
}
