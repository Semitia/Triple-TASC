/**
 * @file recvCmdTask.h
 * @author Alexavier
 * @brief
 * @version 0.1
 * @date 2024-07-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __RECV_CMD_TASK_H__
#define __RECV_CMD_TASK_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum sendMsgOrder_E {
    FINGER_STATE = 0,
    N20_SPD_ORDER_0,  // 电机数据
    N20_SPD_ORDER_1,
    N20_POS_ORDER_0,
    N20_POS_ORDER_1,
    Gage_ORGER_1,  // 应变片数据
    Gage_ORDER_2,
    /* 调试数据 */
    N20_SPD_TARGET_0,  // 电机目标速度
    N20_SPD_TARGET_1,
    N20_OUTPUT_ORDER_0,  // 电机输出
    N20_OUTPUT_ORDER_1,
};

#define SPD_RECV_SCALE (1000)
#define POS_RECV_SCALE (1000)

// 0x00-速度控制 ; 0x01-位置控制; 0x03-设为原点
enum MotorCmd_E { SET_SPD = 0, SET_POS, SET_ORIGIN, SHUT_DOWN_CMD };

enum msgRecvState_E {
    OK = 0,
    ERR,
    TIMEOUT,
};

#pragma pack(push, 1)

/**
 * @brief  Motor 控制数据包
 */
typedef struct __MotorMsg_t {
    uint8_t cmd;
    int16_t data;  // 设定速度值 或 位置值 (注意缩放)
} MotorMsg_t;

/**
 * @brief  数据包
 */
typedef struct __MsgPack_t {
    MotorMsg_t MotorMsg[2];
} MsgPack_t;

#pragma pack(pop)

/**
 * @brief  数据接收控制器
 */
typedef struct __MsgRecv_t {
    float ts;           // 上次成功接收时间戳
    uint8_t recvFlag;   // 接收标志
    uint16_t recvLen;   // 接收数据长度
    MsgPack_t recvMsg;  // 接收数据
} MsgRecv_t;

enum msgRecvState_E recvCmdTask(float ts);
void recvComplete(uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __RECV_CMD_TASK_H__ */
