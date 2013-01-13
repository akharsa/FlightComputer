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
#include "qFSM.h"
#include "States.h"
#include "qWDT.h"
/* ================================ */
/* Prototypes	 					*/
/* ================================ */

void Flight_Task(void * pvParameters);
void Flight_onEntry(void * pvParameters);
void Fligth_onExit(void * pvParameters);

/* ================================ */
/* Private globals 					*/
/* ================================ */
static xTaskHandle hnd;
static xTaskHandle BeaconHnd;

void beacon(void * pvParameters){
	int i;

	for(;;){
		for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);

		vTaskDelay(800/portTICK_RATE_MS);
	}



}

void Flight_onEntry(void *p){
	ConsolePuts_("FLIGHT State: onEntry\r\n",BLUE);
	xTaskCreate( Flight_Task, ( signed char * ) "IDLE", 500, ( void * ) NULL, FLIGHT_PRIORITY, &hnd );
	xTaskCreate( beacon, ( signed char * ) "BEACON", 100, ( void * ) NULL, 1, &BeaconHnd );
}

void Flight_onExit(void *p){
	ConsolePuts_("FLIGHT State: onExit\r\n",BLUE);
	vTaskDelete(hnd);
	vTaskDelete(BeaconHnd);
}

void Flight_Task(void * pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		if ((Joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) == 0){
			state_name_t newState=STATE_IDLE;
			qFSM_ChangeState(newState);
		}

		qESC_SetOutput(MOTOR1,250);
		qESC_SetOutput(MOTOR2,250);
		qESC_SetOutput(MOTOR3,250);
		qESC_SetOutput(MOTOR4,250);

		vTaskDelayUntil( &xLastWakeTime, 5/portTICK_RATE_MS );
	}
}



