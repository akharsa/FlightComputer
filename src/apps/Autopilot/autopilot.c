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

#include "taskList.h"
#include "States.h"

#include "qESC.h"
#include "board.h"

void AppMain(void) {

	// Early init safety start
	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	// System tasks
	xTaskCreate( SystemController, ( signed char * ) "SYSCON", 500, ( void * ) STATE_INIT, 2, NULL);
	vTaskStartScheduler();
	for(;;);
}

