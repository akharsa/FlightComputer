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
#include "qESC.h"
#include "leds.h"
#include "board.h"

void AppMain(void) {

	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);
	qLed_Init(FRONT_LEFT_LED);
	qLed_Init(STATUS_LED);


	xTaskCreate( Communications, ( signed char * ) "COMMS", configMINIMAL_STACK_SIZE, ( void * ) NULL, 2, NULL );
	//xTaskCreate(Telemetry, ( signed char * ) "TELEMETRY", 300, ( void * ) NULL, 1, NULL );
	vTaskStartScheduler();

	for(;;);
}


