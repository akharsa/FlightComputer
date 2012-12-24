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

extern xSemaphoreHandle TelemetrySmphr;

void UART_Rx_Handler(uint8_t * buff, size_t sz);
void ControlDataHandle(void * pvParameters);

static const uint8_t buff[]={"Hello world!"};
static Msg_t msg;
static uint8_t msgBuff[255];

xQueueHandle ControlQueue;
xQueueHandle SystemQueue;

void Communications(void * pvParameters){

	qLed_Init(FRONT_LEFT_LED);


	vTaskDelay(3000/portTICK_RATE_MS);
	//XXX: Should this be done in the comms api?
	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)==RET_ERROR){
		while(1);
	}
	qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);

	msg.Payload = msgBuff;

	ConsolePuts_("======================================\r\n",WHITE);
	ConsolePuts_("Autopilot @ FLC_v2p0 running...\r\n\r\n",WHITE);

	ControlQueue = xQueueCreate(4,sizeof(uint8_t)*255);
	xTaskCreate( ControlDataHandle, ( signed char * ) "COMMS/CONTROL", configMINIMAL_STACK_SIZE, ( void * ) NULL, 1, NULL );
	vTaskDelete(NULL);
}

void ControlDataHandle(void * pvParameters){
	uint8_t buff[10];

	for (;;){
		xQueueReceive(ControlQueue,buff,portMAX_DELAY);
		ConsolePutNumber_(buff[0],10,WHITE);
		ConsolePuts("\r\n");
	}
}

//XXX: Be carefull. This function is being called inside the UART IRQ so it must return quickly.
void UART_Rx_Handler(uint8_t * buff, size_t sz){

	int i;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	ret_t ret;

	for (i=0;i<sz;i++){
		ret=qComms_ParseByte(&msg,*(buff+i));
		switch (ret){
			case RET_MSG_BYTES_REMAINING:
				break;
			case RET_MSG_ERROR:
				qUART_SendByte(2,'E');
				break;
			case RET_MSG_OK:
				switch (msg.Type){
					case MSG_TYPE_CONTROL:
						//xSemaphoreGiveFromISR(TelemetrySmphr,&xHigherPriorityTaskWoken);
						//xQueueSendFromISR(ControlQueue,msg.Payload,&xHigherPriorityTaskWoken);
						qESC_SetOutput(MOTOR1,255-msg.Payload[3]);
						qESC_SetOutput(MOTOR2,255-msg.Payload[3]);
						qESC_SetOutput(MOTOR3,255-msg.Payload[3]);
						qESC_SetOutput(MOTOR4,255-msg.Payload[3]);

						if (msg.Payload[8]&0x40!=0){
							qLed_TurnOn(FRONT_LEFT_LED);
						}else{
							qLed_TurnOff(FRONT_LEFT_LED);
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

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}
