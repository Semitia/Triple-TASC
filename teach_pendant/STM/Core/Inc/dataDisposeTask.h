#ifndef __DATA_DISPOSE_TASK_H__
#define __DATA_DISPOSE_TASK_H__
#include "adcTask.h"
#include "flash.h"

/**
 * @brief 示教器状态机
 */
enum TP_STATE {
  STOP = 0,  // 中止
  TWO,       // 二指模式
  THREE_1,   // 三z指模式
  THREE_2,   // 三指模式
  TWONE_1,   // 二加一指控一指模式
  TWONE_2,   // 二加一指控二指模式
};

void dataDisposeTask(void *argument);
void ledTask(void *argument);
#endif  // __DATA_DISPOSE_TASK_H__
