/*
 * Comms.c
 *
 *  Created on: Feb 6, 2012
 *      Author: Alan
 *
 */
#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
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



static Msg_t msg;
static uint8_t payload[255];
static uint8_t buffer[100];

void Communications(void * pvParameters){

	uint32_t i=0, len;
	msg.Payload = payload;
	ret_t ret;

	for(;;){
		if ((len = qUART_Read(UART_GROUNDCOMM,buffer,100))>0){

			for (i=0;i<len;i++){
				ret=qComms_ParseByte(&msg,*(buffer+i));
				switch (ret){
					case RET_MSG_BYTES_REMAINING:
						break;
					case RET_MSG_ERROR:
						break;
					case RET_MSG_OK:

						switch (msg.Type){
							case MSG_TYPE_CONTROL:
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

						i = 0;
						memset(msg.Payload,0,255);
						msg.Length = 0;
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

	}


	//vTaskDelay(3000/portTICK_RATE_MS);
	//XXX: Should this be done in the comms api?
/*
	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)==RET_ERROR){
		while(1);
	}
	qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);

	msg.Payload = msgBuff;


	ControlQueue = xQueueCreate(4,sizeof(uint8_t)*255);
	xTaskCreate( ControlDataHandle, ( signed char * ) "COMMS/CONTROL", configMINIMAL_STACK_SIZE, ( void * ) NULL, 1, NULL );
	vTaskDelete(NULL);
	*/
}

