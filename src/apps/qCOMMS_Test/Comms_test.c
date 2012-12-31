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
#include "board.h"
#include "DebugConsole.h"
#include "qUART.h"


void SystemController(void * pvParameters){

	qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1);

	ConsolePuts_("======================================\r\n",BLUE);
	ConsolePuts_("Autopilot @ FLC_v2p0 running...\r\n\r\n",BLUE);

	ConsolePuts_("Initializint ESC...\t\t\t\t",BLUE);
	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	ConsolePuts_("[OK]\r\n",BLUE);

	xTaskCreate( Communications, ( signed char * ) "COMMS", configMINIMAL_STACK_SIZE, ( void * ) NULL, 2, NULL );
	//xTaskCreate(Telemetry, ( signed char * ) "TELEMETRY", 300, ( void * ) NULL, 1, NULL );

	vTaskDelete(NULL);
}


void AppMain(void) {


	xTaskCreate(SystemController, ( signed char * ) "SystemController", 300, ( void * ) NULL, 2, NULL );
	vTaskStartScheduler();

	for(;;);
}


