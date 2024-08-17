#ifndef JOINT_H
#define JOINT_H

#include "stdint.h"
#include "stm32f1xx_hal.h"

#define MAX_ADC_VALUE 4000
#define MIN_ADC_VALUE 2000
#define KEY HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define KEY_PRESSED 0   // 按下接低电平
#define KEY_RELEASED 1  // 上拉输入
#define DOUBLE_CLICK_INTERVAL_L 80
#define DOUBLE_CLICK_INTERVAL_H 350
#define LONG_PRESS_INTERVAL 700
#define INIT_INTERVAL 100
#define X 0
#define Y 1
#ifndef M_PI
#define M_PI 3.141592654
#endif

/**
 * @brief 用于按键检测的状态机
 */
enum KEY_STATE {
  INIT = 0,          // 初始状态
  PRESS_1,           // 按下第一次
  RELEASE_1,         // 释放第一次
  PRESS_2,           // 按下第二次
  RELEASE_2,         // 释放第二次
  WAIT_FOR_CLEAR,    // 等待读取并清除
  WAIT_FOR_RELEASE,  // 清除指令后等待释放才能开始新的循环
};

/**
 * @brief 按键检测结果，为向外提供的接口
 */
enum KEY_RESULT {
  IDLE = 0,
  SINGLE = 1,
  DOUBLE = 2,
  LONG = 3,
};

/**
 * @brief 摇杆方向状态
 */
enum StickState {
  UP = 0,
  DOWN = 1,
  LEFT = 2,
  RIGHT = 3,
  CENTER = 4,
};

extern uint16_t adc_buf[2];
extern enum KEY_RESULT key_result;
extern enum StickState stick_state;
// extern float stick[2];

void clear_result(void);
void click_cek(void);
void get_stick(void);
void jointTask(void *argument);
#endif /* JOINT_H */
