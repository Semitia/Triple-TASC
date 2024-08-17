#ifndef _VOFA_H_
#define _VOFA_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
	
#define VOFA_BAUDRATE 230400
#define VOFA_IRQHandler USART1_IRQHandler
#define VOFA_MAX_CHANNEL 12  // 最大支持的变量数
#define VOFA_RX_LEN 128
#define VOFA_TX_LEN (VOFA_MAX_CHANNEL * 4 + 4)

extern float vofa_justfloat[VOFA_MAX_CHANNEL];
extern uint8_t vofa_buf_Rx[VOFA_RX_LEN];

void vofaSend(void);
void vofaInit(void);

#ifdef __cplusplus
}
#endif
#endif  // _VOFA_H_
