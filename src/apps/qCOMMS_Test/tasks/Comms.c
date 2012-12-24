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


extern xSemaphoreHandle TelemetrySmphr;

void UART_Rx_Handler(uint8_t * buff, size_t sz);
void ControlDataHandle(void * pvParameters);

static const uint8_t buff[]={"Hello world!"};
static Msg_t msg;
static uint8_t msgBuff[255];

xQueueHandle ControlQueue;
xQueueHandle SystemQueue;

void Communications(void * pvParameters){

	//XXX: Should this be done in the comms api?
	if (qUART_Init(0,115200,8,QUART_PARITY_NONE,8)==RET_ERROR){
		while(1);
	}

	msg.Payload = msgBuff;
	qUART_Register_RBR_Callback(0, UART_Rx_Handler);
	qComms_SendMsg(0,0xBB,MSG_TYPE_SYSTEM,strlen((char *)buff),buff);

	while(1){
		//qComms_SendMsg(0,0xBB,MSG_TYPE_DEBUG,strlen((char *)buff),buff);
		ConsolePuts_("Hello world!\r\n",CYAN);
		ConsolePutNumber_(4095,16,WHITE);
		ConsolePuts("\r\n");
		vTaskDelay(1000/portTICK_RATE_MS);
	}

	ControlQueue = xQueueCreate(4,sizeof(uint8_t)*255);
	xTaskCreate( ControlDataHandle, ( signed char * ) "COMMS/CONTROL", configMINIMAL_STACK_SIZE, ( void * ) NULL, 1, NULL );
	vTaskDelete(NULL);
}

void ControlDataHandle(void * pvParameters){
	uint8_t buff[255];
	for (;;){
		xQueueReceive(ControlQueue,buff,portMAX_DELAY);
		//qUART_Send(2,buff,strlen((char*)buff));
	}
}

//XXX: Be carefull. This function is being called inside the UART IRQ so it must return quickly.
void UART_Rx_Handler(uint8_t * buff, size_t sz){

	int i;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	for (i=0;i<sz;i++){
		switch (qComms_ParseByte(&msg,*(buff+i))){
			case RET_MSG_BYTES_REMAINING:
				break;
			case RET_MSG_ERROR:
				qUART_SendByte(2,'E');
				break;
			case RET_MSG_OK:
				switch (msg.Type){
					case MSG_TYPE_CONTROL:
						break;
					case MSG_TYPE_SYSTEM:
						xSemaphoreGiveFromISR(TelemetrySmphr,&xHigherPriorityTaskWoken);
						xQueueSendFromISR(ControlQueue,msg.Payload,&xHigherPriorityTaskWoken);
						break;
					default:
						break;
				}
				break;
			case RET_ERROR:
				// Problem with memory
				break;
			case RET_OK:
				// Neever
				break;
			default:
				break;
		}
	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}
