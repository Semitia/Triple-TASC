/**
 * @file recvCmdTask.c
 * @author Alexavier
 * @brief
 * @version 0.1
 * @date 2024-07-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "recvCmdTask.h"

#include <string.h>

#include "N20.h"
#include "VOFA.h"
#include "user_lib.h"

MsgRecv_t MsgRecv = {0};

enum msgRecvState_E recvCmdTask(float ts) {
    uint8_t i;

    if (!MsgRecv.recvFlag) {
        if (ts - MsgRecv.ts > 0.2f) {
            return TIMEOUT;
        }
        return OK;
    }

    MsgRecv.recvFlag = FALSE;
    MsgRecv.ts       = ts;
    for (i = 0; i < N20_NUM; i++) {
        switch (MsgRecv.recvMsg.MotorMsg[i].cmd) {
            case SET_SPD: {
                float recv_spd = (float)MsgRecv.recvMsg.MotorMsg[i].data;
                recv_spd /= SPD_RECV_SCALE;
                setSpd(&n20[i], recv_spd);
                break;
            }
            case SET_POS: {
                float recv_pos = (float)MsgRecv.recvMsg.MotorMsg[i].data;
                recv_pos /= POS_RECV_SCALE;
                setPos(&n20[i], recv_pos);
                break;
            }
            case SET_ORIGIN: {
                break;
            }
            case SHUT_DOWN_CMD: {
                shutDown(&n20[i]);
                break;
            }
        }
    }
    return OK;
}

void recvComplete(uint16_t len) {
//    int msg_size     = sizeof(MotorMsg_t);
    MsgRecv.recvFlag = TRUE;
    memcpy(&MsgRecv.recvMsg, vofa_buf_Rx, len);
    MsgRecv.recvLen = len;
    return;
}
