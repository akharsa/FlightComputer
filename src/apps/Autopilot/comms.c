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

extern xSemaphoreHandle TelemetrySmphr;

void UART_Rx_Handler(uint8_t * buff, size_t sz);
void ControlDataHandle(void * pvParameters);

static Msg_t msg;
static uint8_t msgBuff[255];

xSemaphoreHandle DataSmphr;

void Communications(void * pvParameters){
	ret_t ret;
	uint8_t ch;

	msg.Payload = msgBuff;

	DataSmphr = xSemaphoreCreateCounting(UINT32_MAX,0);
    if (DataSmphr==NULL){
    	while(1);
    }

    qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);

	for (;;){
		xSemaphoreTake(DataSmphr,portMAX_DELAY);
		qUART_ReadByte(UART_GROUNDCOMM,&ch);

		ret=qComms_ParseByte(&msg,ch);
				switch (ret){
					case RET_MSG_BYTES_REMAINING:
						break;
					case RET_MSG_ERROR:
						qUART_SendByte(2,'E');
						break;
					case RET_MSG_OK:
						switch (msg.Type){
							case MSG_TYPE_CONTROL:
								qESC_SetOutput(MOTOR1,255-msg.Payload[3]);
								qESC_SetOutput(MOTOR2,255-msg.Payload[3]);
								qESC_SetOutput(MOTOR3,255-msg.Payload[3]);
								qESC_SetOutput(MOTOR4,255-msg.Payload[3]);

								if ((msg.Payload[9]&0x03)!=0) {
									qLed_TurnOn(STATUS_LED);
									qWDT_Feed();
								}else{
									qLed_TurnOff(STATUS_LED);
								}
								break;

							case MSG_TYPE_DEBUG:
								ConsolePuts("ECHO: ");
								ConsolePuts(msg.Payload);
								ConsolePuts("\r\n");
								break;
							default:
								break;
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
}

void UART_Rx_Handler(uint8_t * buff, size_t sz){
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	while (sz){
		xSemaphoreGiveFromISR(DataSmphr,&xHigherPriorityTaskWoken);
		sz--;
	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}

