#ifndef __RGB_H__
#define __RGB_H__

#include "main.h"

#define CODE_1 (54)  // 66%
#define CODE_0 (26)  // 33%

typedef struct __RGB_t {
  uint8_t R;
  uint8_t G;
  uint8_t B;
} RGB_t;

#define Pixel_NUM 1  // LED?????,??????64?,

enum COLOR {
  WS_RED = 0,
  WS_GREEN,
  WS_BLUE,
  WS_SKYBLUE,
  WS_PINK,
  WS_YELLOW,
  WS_ORANGE,
  WS_WHITE,
  WS_BLACK
};

extern const RGB_t WS_colorTable[16];

void Reset_Load(void);
void RGB_SendArray(void);
void RGB_SetOne_Color(uint8_t LedId, RGB_t Color);
void RGB_Flush(void);  //??RGB??

#endif
