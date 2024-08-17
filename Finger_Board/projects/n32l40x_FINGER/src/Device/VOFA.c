#include "VOFA.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "USART.h"
#include "recvCmdTask.h"

float vofa_justfloat[VOFA_MAX_CHANNEL];
uint8_t vofa_buf_Tx[VOFA_TX_LEN];
uint8_t vofa_buf_Rx[VOFA_RX_LEN];

void vofaInit(void) {
    usartInit(VOFA_BAUDRATE, vofa_buf_Tx, vofa_buf_Rx);
    vofa_buf_Tx[VOFA_MAX_CHANNEL * 4]     = 0x00;
    vofa_buf_Tx[VOFA_MAX_CHANNEL * 4 + 1] = 0x00;
    vofa_buf_Tx[VOFA_MAX_CHANNEL * 4 + 2] = 0x80;
    vofa_buf_Tx[VOFA_MAX_CHANNEL * 4 + 3] = 0x7f;
}

void vofaSend(void) {
    // 将32位的浮点数转换为4个8位的整型
    memcpy(vofa_buf_Tx, (uint8_t *)vofa_justfloat, sizeof(vofa_justfloat));
    dmaSend(VOFA_TX_LEN);
}
