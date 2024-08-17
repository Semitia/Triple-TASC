#include "RGB.h"

#include "stdlib.h"
#include "tim.h"

uint32_t Pixel_Buf[Pixel_NUM + 1][24] = {0};
const RGB_t WS_colorTable[16] = {
    {255, 0, 0},      // RED
    {0, 255, 0},      // GREEN
    {0, 0, 255},      // BLUE
    {0, 255, 255},    // SKYBLUE
    {255, 0, 220},    // PINK
    {128, 255, 0},    // YELLOW
    {255, 106, 0},    // ORANGE
    {255, 255, 255},  // WHITE
    {0, 0, 0},        // BLACK
};

void Reset_Load(void) {
  uint8_t i;
  for (i = 0; i < 24; i++) {
    Pixel_Buf[Pixel_NUM][i] = 0;
  }
}

void RGB_SendArray(void) {
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)Pixel_Buf,
                        (Pixel_NUM + 1) * 24);
}

void RGB_Flush(void) {
  Reset_Load();
  RGB_SendArray();
}

void RGB_SetOne_Color(uint8_t LedId, RGB_t Color) {
  uint8_t i;
  for (i = 0; i < 8; i++)
    Pixel_Buf[LedId][i] =
        ((Color.G & (1 << (7 - i))) ? (CODE_1) : CODE_0);
  for (i = 8; i < 16; i++)
    Pixel_Buf[LedId][i] =
        ((Color.R & (1 << (15 - i))) ? (CODE_1) : CODE_0);
  for (i = 16; i < 24; i++)
    Pixel_Buf[LedId][i] =
        ((Color.B & (1 << (23 - i))) ? (CODE_1) : CODE_0);
}
