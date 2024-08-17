#include "joint.h"

#include <math.h>

#include "adcTask.h"

enum KEY_RESULT key_result = IDLE;
enum KEY_STATE key_state = INIT;
enum StickState stick_state = CENTER;

float limit(uint16_t x, uint16_t mx, uint16_t mn) {
  if (x > mx) {
    x = mx;
  }
  if (x < mn) {
    x = mn;
  }
  return x;
}

float stick[2];
void get_stick(void) {
  float raw_data, angle;
  const float alpha = 0.1f;
  uint16_t value_lim = limit(value[X], MAX_ADC_VALUE, MIN_ADC_VALUE);
  raw_data =
      (float)(value_lim - MIN_ADC_VALUE) / (MAX_ADC_VALUE - MIN_ADC_VALUE) -
      0.5f;
  stick[X] = stick[X] * (1 - alpha) + raw_data * alpha;
  value_lim = limit(value[Y], MAX_ADC_VALUE, MIN_ADC_VALUE);
  raw_data =
      (float)(value_lim - MIN_ADC_VALUE) / (MAX_ADC_VALUE - MIN_ADC_VALUE) -
      0.5f;
  stick[Y] = stick[Y] * (1 - alpha) + raw_data * alpha;

  if (fabs(stick[X]) < 0.2f && fabs(stick[Y]) < 0.2f) {
    stick_state = CENTER;
    return;
  }
  angle = atan2(stick[Y], stick[X]);
  if (angle > -M_PI / 4 && angle < M_PI / 4) {
    stick_state = UP;
  } else if (angle > M_PI / 4 && angle < 3 * M_PI / 4) {
    stick_state = LEFT;
  } else if (angle > -3 * M_PI / 4 && angle < -M_PI / 4) {
    stick_state = RIGHT;
  } else {
    stick_state = DOWN;
  }
}

uint8_t scan_key(void) {
  if (KEY == KEY_PRESSED) {
    HAL_Delay(10);
    if (KEY == KEY_PRESSED) {
      return KEY_PRESSED;
    }
  }
  return KEY_RELEASED;
}

void clear_result(void) {
  key_result = IDLE;
  key_state = WAIT_FOR_RELEASE;
}

/**
 * @brief 双击检测
 *
 */
static uint32_t last_press_ts = 0, last_release_ts = 0;  // 按下和释放的时间戳
uint32_t ts;
void click_cek(void) {
  ts = HAL_GetTick();

  if (key_state == WAIT_FOR_CLEAR) {
    return;
  }

  if (scan_key() == KEY_PRESSED) {
    switch (key_state) {
      case INIT: {
        if (ts - last_release_ts > INIT_INTERVAL) {
          key_state = PRESS_1;
          last_press_ts = ts;
        }
        break;
      }
      case PRESS_1:
        if (ts - last_press_ts > LONG_PRESS_INTERVAL) {
          key_result = LONG;
          key_state = WAIT_FOR_CLEAR;
        }
        break;
      case RELEASE_1:
        if ((ts - last_release_ts > DOUBLE_CLICK_INTERVAL_L) &&
            (ts - last_release_ts < DOUBLE_CLICK_INTERVAL_H)) {
          key_result = DOUBLE;
          key_state = WAIT_FOR_CLEAR;
          last_press_ts = ts;
        }
        break;
      default:
        break;
    }

  } else {
    switch (key_state) {
      case WAIT_FOR_RELEASE:
        key_state = INIT;
        last_release_ts = ts;
        break;
      case INIT:
        break;
      case PRESS_1:
        key_state = RELEASE_1;
        last_release_ts = ts;
        break;
      case RELEASE_1:
        if (ts - last_release_ts > DOUBLE_CLICK_INTERVAL_H) {
          key_result = SINGLE;
          key_state = WAIT_FOR_CLEAR;
        }
        break;
      case PRESS_2:
        break;
      case RELEASE_2:
        break;
      default:
        break;
    }
  }
  return;
}

void jointTask(void *argument) {
  while (1) {
    click_cek();
    get_stick();
    osDelay(10);
  }
}
