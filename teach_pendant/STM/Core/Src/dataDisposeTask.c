#include "dataDisposeTask.h"
#include "adcTask.h"
#include "RGB.h"
#include "joint.h"
#include "stdint.h"

extern DMA_HandleTypeDef hdma_adc1;

enum TP_STATE tpState = TWO;
void dataDisposeTask(void *argument) {
  enum TP_STATE last_tpState = STOP;
  enum TP_STATE led_last_tpState = STOP;
  uint8_t TxData[5];
	uint16_t i,j;
  while (1) {
		
		// __HAL_DMA_ENABLE(&hdma_adc1);
		for(i=0;i<ADC1_CHANNEL_CNT;i++){
				adc1_sum_val[i] = 0;
		}
		for(i=0;i<ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE;i++){
				j = i%ADC1_CHANNEL_CNT;
				adc1_sum_val[j] += adc1_val_buf[i];
		}
		for(i=0;i<ADC1_CHANNEL_CNT;i++){
				value[i] = adc1_sum_val[i]/ADC1_CHANNEL_FRE;
		}
		osDelay(1);
				
    // 模式切换
    switch (key_result) {
      case SINGLE: {
        switch (tpState) {
          case THREE_1: {
            tpState = THREE_2;
            break;
          }
          case THREE_2: {
            tpState = THREE_1;
            break;
          }
          case TWONE_1: {
            tpState = TWONE_2;
            break;
          }
          case TWONE_2: {
            tpState = TWONE_1;
            break;
          }
          default:
            break;
        }
        clear_result();
        break;
      }
      case DOUBLE: {
        switch (tpState) {
          case TWO: {
            tpState = THREE_1;
            break;
          }
          case THREE_1:
          case THREE_2: {
            tpState = TWONE_1;
            break;
          }
          case TWONE_1:
          case TWONE_2: {
            tpState = TWO;
            break;
          }
          case STOP: {
            tpState = last_tpState;
            break;
          }
          default:
            break;
        }
        clear_result();
        break;
      }
      case LONG: {
        if (tpState != STOP) {
          last_tpState = tpState;
        }
        tpState = STOP;
        clear_result();
        break;
      }
      default:
        break;
    }

//    if (tpState != led_last_tpState) {
//      led_last_tpState = tpState;
      switch (tpState) {
        case TWO: {
          RGB_SetOne_Color(0, WS_colorTable[WS_YELLOW]);
          break;
        }
        case THREE_1: {
          RGB_SetOne_Color(0, WS_colorTable[WS_GREEN]);
          break;
        }
        case THREE_2: {
          RGB_SetOne_Color(0, WS_colorTable[WS_RED]);
          break;
        }
        case TWONE_1: {
          RGB_SetOne_Color(0, WS_colorTable[WS_BLUE]);
          break;
        }
        case TWONE_2: {
          RGB_SetOne_Color(0, WS_colorTable[WS_PINK]);
          break;
        }
        case STOP: {
          RGB_SetOne_Color(0, WS_colorTable[WS_WHITE]);
          break;
        }
        default:
          break;
      }
      RGB_SendArray();
//    }

    TxData[0] = (uint8_t)tpState;
    TxData[1] = (uint8_t)stick_state;
    TxData[2] = (uint8_t)(value[X] >> 4);
    TxData[3] = (uint8_t)(value[X] & 0x0f) << 4 | (uint8_t)(value[Y] >> 8);
    TxData[4] = (uint8_t)(value[Y] & 0xff);

    CAN_SendState(TxData, 5);
    osDelay(20);
  }

}

void ledTask(void *argument) {
  // enum TP_STATE led_last_tpState = STOP;

  while (1) {
    // if (tpState == led_last_tpState) {
    //   continue;
    // }
    // led_last_tpState = tpState;
    // switch (tpState) {
    //   case TWO: {
    //     RGB_SetOne_Color(0, WS_colorTable[WS_YELLOW]);
    //     break;
    //   }
    //   case THREE_1: {
    //     RGB_SetOne_Color(0, WS_colorTable[WS_GREEN]);
    //     break;
    //   }
    //   case THREE_2: {
    //     RGB_SetOne_Color(0, WS_colorTable[WS_RED]);
    //     break;
    //   }
    //   case TWONE_1: {
    //     RGB_SetOne_Color(0, WS_colorTable[WS_BLUE]);
    //     break;
    //   }
    //   case TWONE_2: {
    //     RGB_SetOne_Color(0, WS_colorTable[WS_PINK]);
    //     break;
    //   }
    //   case STOP: {
    //     RGB_SetOne_Color(0, WS_colorTable[WS_WHITE]);
    //     break;
    //   }
    //   default:
    //     break;
    // }
    // RGB_SendArray();
    osDelay(1000);
  }
}
