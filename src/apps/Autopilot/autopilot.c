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

void AppMain(void) {

	xTaskCreate( SystemController, ( signed char * ) "SYSCON", 500, ( void * ) STATE_INIT, 2, NULL);
	vTaskStartScheduler();
	for(;;);
}


