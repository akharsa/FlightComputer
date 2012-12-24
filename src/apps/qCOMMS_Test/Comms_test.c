/*
===============================================================================
 Name        : main.c
 Author      : 
 Version     :
 Copyright   : Copyright (C) 
 Description : Quadrotor On board computer
===============================================================================
*/

#include "types.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tasks/tasklist.h"

void AppMain(void) {

	xTaskCreate( Communications, ( signed char * ) "COMMS", configMINIMAL_STACK_SIZE, ( void * ) NULL, 1, NULL );
	//xTaskCreate(Telemetry, ( signed char * ) "TELEMETRY", 300, ( void * ) NULL, 1, NULL );
	vTaskStartScheduler();

	for(;;);
}


