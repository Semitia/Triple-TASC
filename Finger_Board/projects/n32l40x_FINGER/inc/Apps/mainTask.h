/**
 * @file mainTask.h
 * @author Alexavier
 * @brief 
 * @version 0.1
 * @date 2024-07-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __MAINTASK_H__
#define __MAINTASK_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "n32l40x.h"
#include "TIM6_CNT.h"

void taskLoop(void);
		
#ifdef __cplusplus
}
#endif
#endif /* __MAINTASK_H__ */
