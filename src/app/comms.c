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

#include "quadrotor.h"
extern xSemaphoreHandle TelemetrySmphr;

void UART_Rx_Handler(uint8_t * buff, size_t sz);

static Msg_t msg;
static uint8_t msgBuff[255];

xSemaphoreHandle DataSmphr;

void Communications(void * pvParameters){
	ret_t ret;
	uint8_t ch;

	msg.Payload = msgBuff;

	//DataSmphr = xSemaphoreCreateCounting(UINT32_MAX,0);
	vSemaphoreCreateBinary(DataSmphr);

    if (DataSmphr==NULL){
    	while(1);
    }

    //qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);

	for (;;){
		if (pdTRUE == xSemaphoreTake(DataSmphr,500/portTICK_RATE_MS)){
			switch (msg.Type){
				case MSG_TYPE_CONTROL:
					memcpy(&Joystick,msg.Payload,10);
					break;
				default:
					break;
			}
		}else{
			// Timeout to get a new joystick commands, values to 0
			memset(&Joystick,0,sizeof(Joystick));
		}
	}
}

uint8_t led = 0;

void UART_Rx_Handler(uint8_t * buff, size_t sz){
	uint32_t i;
	uint8_t buffer;
	ret_t ret;
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	for (i=0;i<sz;i++){

		qUART_ReadByte(UART_GROUNDCOMM,&buffer);
		ret=qComms_ParseByte(&msg,buffer);

		switch (ret){
			case RET_MSG_BYTES_REMAINING:
				break;
			case RET_MSG_ERROR:
				//qUART_SendByte(2,'E');
				break;
			case RET_MSG_OK:
				xSemaphoreGiveFromISR(DataSmphr,&xHigherPriorityTaskWoken);
				if (led==0){
					qLed_TurnOn(STATUS_LED);
					led = 1;
				}else{
					qLed_TurnOff(STATUS_LED);
					led = 0;
				}

				break;
			case RET_ERROR:
				// Problem with memory
				break;
			case RET_OK:
				// Never
				break;
		}

	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}

