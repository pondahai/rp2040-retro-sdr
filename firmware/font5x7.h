#ifndef FONT5X7_H
#define FONT5X7_H

#include <stdint.h>

#define FONT_CW 6        /* 格寬（字 5 + 間距 1） */
#define FONT_CH 8        /* 格高（字 7 + 間距 1） */

/* 字元 c 第 row 列（0..6）的點陣，bit 4 = 最左邊。範圍外回 0。 */
uint8_t font5x7_row(char c, int row);

#endif /* FONT5X7_H */
