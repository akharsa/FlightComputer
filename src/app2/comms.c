/*
 * Comms.c
 *
 *  Created on: Feb 6, 2012
 *      Author: Alan
 *
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "qUART.h"
#include "qCOMMS.h"

#include "string.h"
#include "DebugConsole.h"
#include "board.h"

#include "qESC.h"
#include "leds.h"
#include "qWDT.h"
#include "joystick.h"
#include "quadrotor.h"

//extern xSemaphoreHandle TelemetrySmphr;
//extern state_name_t systemState;

void UART_Rx_Handler(uint8_t * buff, size_t sz);

static Msg_t msg;
static uint8_t msgBuff[255];

xSemaphoreHandle DataSmphr;
traceLabel comms_trcLabel;

void Communications(void * pvParameters){
	ret_t ret;
	uint8_t ch;

	msg.Payload = msgBuff;

	vSemaphoreCreateBinary(DataSmphr);

    if (DataSmphr==NULL){
    	while(1);
    }

    qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);

    comms_trcLabel = xTraceOpenLabel("Comms task");

	for (;;){
		if (pdTRUE == xSemaphoreTake(DataSmphr,500/portTICK_RATE_MS)){
			vTracePrintF(comms_trcLabel,"Got joystick package");
			switch (msg.Type){
				case MSG_TYPE_CONTROL:
#if 0
					memcpy(&Joystick,msg.Payload,10);

					if (systemState == STATE_FLIGHT){
						if ((Joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) == 0){
							state_name_t newState=STATE_IDLE;
							qFSM_ChangeState(newState);
						}
					}else if (systemState == STATE_IDLE){
						if ((Joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) != 0){
							state_name_t newState=STATE_FLIGHT;
							qFSM_ChangeState(newState);
						}
					}
					break;
#endif
				case MSG_TYPE_DEBUG:
					 break;
				default:
					break;
			}
		}else{
			// Timeout to get a new joystick commands, values to 0
			vTracePrintF(comms_trcLabel,"Joystick package timeout");
			memset(&Joystick,0,sizeof(Joystick));
			//qLed_TurnOff(STATUS_LED);
#if 0
			if (systemState == STATE_FLIGHT){
				state_name_t newState=STATE_IDLE;
				qFSM_ChangeState(newState);
			}
#endif
		}
	}
}

uint8_t led = 0;
void error(uint8_t i){
	__NOP();
}
void UART_Rx_Handler(uint8_t * buff, size_t sz){
	uint32_t i;
	ret_t ret;
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	for (i=0;i<sz;i++){

		//qUART_ReadByte(UART_GROUNDCOMM,&buffer);
		ret=qComms_ParseByte(&msg,*(buff+i));

		switch (ret){
			case RET_MSG_BYTES_REMAINING:
				break;
			case RET_MSG_ERROR:
				error(0);
				break;
			case RET_MSG_OK:
				xSemaphoreGiveFromISR(DataSmphr,&xHigherPriorityTaskWoken);
/*
				if (led==0){
					qLed_TurnOn(STATUS_LED);
					led = 1;
				}else{
					qLed_TurnOff(STATUS_LED);
					led = 0;
				}
*/
				break;
			case RET_ERROR:
				// Problem with memory
				error(1);
				break;
			case RET_OK:
				// Never
				break;
		}

	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}

