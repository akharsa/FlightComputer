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


float control[4]={0.0};
uint16_t inputs[4]={0};

#define Z_C	0
#define PHI_C	1
#define THETA_C	2
#define PSI_C	3


#define K_Z		800
#define K_PHI	200
#define K_THETA	200
#define K_PSI	200
xSemaphoreHandle DataSmphr;


float map(long x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void Communications(void * pvParameters){
	ret_t ret;
	uint8_t ch;

	msg.Payload = msgBuff;

	DataSmphr = xSemaphoreCreateCounting(UINT32_MAX,0);
    if (DataSmphr==NULL){
    	while(1);
    }

    //qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);

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



								control[Z_C] = map(255-msg.Payload[1]-128,0,128,0.0,1.0);
								control[PHI_C] = map(255-msg.Payload[3],0,255,1.0,-1.0);
								control[THETA_C] = map(255-msg.Payload[2],0,255,-1.0,1.0);
								control[PSI_C] = map(255-msg.Payload[0],0,255,-1.0,1.0);

								inputs[0] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI - control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
								inputs[1] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI + control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);
								inputs[2] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI + control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
								inputs[3] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI - control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);

								qESC_SetOutput(MOTOR1,inputs[0]);
								qESC_SetOutput(MOTOR2,inputs[1]);
								qESC_SetOutput(MOTOR3,inputs[2]);
								qESC_SetOutput(MOTOR4,inputs[3]);

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

