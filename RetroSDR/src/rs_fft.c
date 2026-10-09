/* 轉接檔：Arduino 會把整個 sketch 目錄複製到 build 目錄再編譯，所以
 * 相對路徑跳不出去。正本住在 firmware/，靠編譯旗標 -I 指過去（見
 * build_uf2.bat），這裡用角括號才不會自己 include 自己。
 * 做法與 rp2040-retro-dict 相同。 */
#include <fft.c>
