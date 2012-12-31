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

void Communications(void * pvParameters){
	qLed_Init(FRONT_LEFT_LED);
	for(;;){



	}


	//vTaskDelay(3000/portTICK_RATE_MS);
	//XXX: Should this be done in the comms api?
/*
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
	*/
}

