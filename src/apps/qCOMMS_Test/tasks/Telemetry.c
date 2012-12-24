/*
 * Telemetry.c
 *
 *  Created on: Feb 6, 2012
 *      Author: Alan
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "qUART.h"
#include "string.h"

xSemaphoreHandle TelemetrySmphr;

void Telemetry(void * pvParameters){

	//int i;
	uint8_t buff[40*5];

	vSemaphoreCreateBinary(TelemetrySmphr);

	if (xSemaphoreTake(TelemetrySmphr,0)!=pdTRUE){
		//TODO: Handle error
		while(1);
	}

	for (;;){
		xSemaphoreTake(TelemetrySmphr,portMAX_DELAY);
		qUART_Send(2,&SystemCoreClock,sizeof(SystemCoreClock));

		//vTaskGetRunTimeStats(buff);
		//qUART_Send(2,buff,strlen(buff));
	}

}
