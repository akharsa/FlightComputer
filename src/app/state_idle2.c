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

uint32_t signal_t=0;
//uint32_t signal_time[]={0,200,600,1000,1400};
//float signal_values[]={0.0,360.0,0.0,-360.0,0.0};

uint32_t signal_time[]={0};
float signal_values[]={0.0};

float control_input;

float control[4]={0.0};
uint16_t inputs[4]={0};

int16_t buffer[3];
int16_t temperature;

qPID ctrl;


#define Z_C	0
#define PHI_C	1
#define THETA_C	2
#define PSI_C	3

#define K_Z		800
#define K_PHI	200
#define K_THETA	200
#define K_PSI	200

extern float yaw_control;

//#define TODEGSEC(x) (x/16.4f)
#define TODEGSEC(x) x


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

