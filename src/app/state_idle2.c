#include "FreeRTOS.h"
#include "task.h"

#include "qESC.h"
#include "qFSM.h"
#include "leds.h"
#include "qWDT.h"

#include "board.h"
#include "DebugConsole.h"
#include "qCOMMS.h"
#include "MPU6050.h"

#include "qPIDs.h"
#include "quadrotor.h"
#include "taskList.h"

#include "telemetry.h"
#include "joystick.h"
#include "States.h"

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
	int i;
	for (i=0;i<10;i++){
		qESC_SetOutput(MOTOR1,1);
		qESC_SetOutput(MOTOR2,1);
		qESC_SetOutput(MOTOR3,1);
		qESC_SetOutput(MOTOR4,1);
	}

	ConsolePuts_("IDLE State: onEntry\r\n",BLUE);
	xTaskCreate( Idle_Task, ( signed char * ) "IDLE", 300, ( void * ) NULL, IDLE_PRIORITY, &hnd );
}

void Idle_onExit(void * p){
	ConsolePuts_("IDLE State: onExit\r\n",BLUE);
	vTaskDelete(hnd);
}

void Idle_Task(void * pvParameters){

	uint8_t i;

	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		if ((Joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) != 0){
			/* Terminate and go to Idle */
			state_name_t newState=STATE_FLIGHT;
			qFSM_ChangeState(newState);
		}
		vTaskDelayUntil( &xLastWakeTime, 10/portTICK_RATE_MS );
	}
}

