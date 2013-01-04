/*
 * Config.c
 *
 *  Created on: Mar 3, 2012
 *      Author: Alan
 */

#include "FreeRTOS.h"
#include "task.h"

#include "qESC.h"
#include "qFSM.h"
#include "leds.h"
#include "qWDT.h"

/* ================================ */
/* Prototypes	 					*/
/* ================================ */

void Idle_Task(void * pvParameters);
void Idle_onEntry(void * pvParameters);
void Idle_onExit(void * pvParameters);

/* ================================ */
/* Private globals 					*/
/* ================================ */
static xTaskHandle hnd;

void Idle_onEntry(void * p){
	//xTaskCreate(Idle_Task, ( signed char * ) "IDLE", 200, ( void * ) NULL, 1, &hnd );
	qWDT_Start(5000000);
}

void Idle_onExit(void * p){
	//vTaskDelete(hnd);
}

void Idle_Task(void * pvParameters){
	int i;

	for (i=1;i<=4;i++)
	{
		ESC_SetChannel(i);
		ESC_SetSpeed(i,0);
	}

	for (;;){
		qLedsFlash(2,50);
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

