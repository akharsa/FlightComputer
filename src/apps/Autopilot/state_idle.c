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

#include "board.h"
#include "DebugConsole.h"
#include "qCOMMS.h"
#include "MPU6050.h"

#include "qPIDs.h"

int16_t sensors[6];
int16_t temperature;

qPID ctrl;

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
	xTaskCreate(Idle_Task, ( signed char * ) "IDLE", 200, ( void * ) NULL, 2, &hnd );
}

void Idle_onExit(void * p){
	vTaskDelete(hnd);
}

uint32_t signal_t=0;
uint32_t signal_time[]={0,200,600,1000,1400};
float signal_values[]={0.0,720.0,0.0,-720.0,0.0};

float control_input;

float control[4]={0.0};
uint16_t inputs[4]={0};

#define Z_C	0
#define PHI_C	1
#define THETA_C	2
#define PSI_C	3

#define K_Z		800
#define K_PHI	200
#define K_THETA	200
#define K_PSI	200

float buffer[3];
uint8_t msg[]={"Hello world\r\n"};

extern float yaw_control;

void Idle_Task(void * pvParameters){
	int i,j;
	float gyroz;
	float CO;
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    ctrl.AntiWindup = ENABLED;
    ctrl.Bumpless = ENABLED;

    ctrl.Mode = AUTOMATIC;
    ctrl.OutputMax = 1.0;
    ctrl.OutputMin = -1.0;

    ctrl.Ts = 0.01;

    ctrl.b = 1.0;
    ctrl.c = 1.0;

    ctrl.K = 0.003;
    ctrl.Ti = 0.003/0.005; //conversion de paralelo a ideal
    ctrl.Td = 0.0000;
    ctrl.Nd = 5;

    qPID_Init(&ctrl);

//	qWDT_Start(500000);

	for (;;)
	{

		qLed_TurnOn(FRONT_LEFT_LED);

		MPU6050_getMotion6(&sensors[0],&sensors[1],&sensors[2],&sensors[3],&sensors[4],&sensors[5]);

		for (j=0;j<(sizeof(signal_time)/4);j++){
			if (signal_t>=signal_time[j]){
				control_input = signal_values[j];
			}
		}

		gyroz = sensors[5]/16.4;

		CO = qPID_Process(&ctrl,control_input,-gyroz,NULL);


		control[Z_C] = 0.2;
		control[PHI_C] = 0;
		control[THETA_C] = 0;
		control[PSI_C] = CO;

		inputs[0] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI - control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
		inputs[1] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI + control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);
		inputs[2] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI + control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
		inputs[3] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI - control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);

		qESC_SetOutput(MOTOR1,inputs[0]);
		qESC_SetOutput(MOTOR2,inputs[1]);
		qESC_SetOutput(MOTOR3,inputs[2]);
		qESC_SetOutput(MOTOR4,inputs[3]);

		buffer[0] = control_input;
		buffer[1] = gyroz;
		buffer[2] = CO;

		if (signal_t<2000){
			signal_t++;
			qComms_SendMsg(UART_GROUNDCOMM,0xBB,MSG_TYPE_TELEMETRY,sizeof(buffer),buffer);
		}else{
			qESC_SetOutput(MOTOR1,1);
			qESC_SetOutput(MOTOR2,1);
			qESC_SetOutput(MOTOR3,1);
			qESC_SetOutput(MOTOR4,1);
			qLed_TurnOn(FRONT_LEFT_LED);
			qLed_TurnOn(FRONT_RIGHT_LED);
			qLed_TurnOn(REAR_LEFT_LED);
			qLed_TurnOn(REAR_RIGHT_LED);
			vTaskDelete(NULL);
		}

		qLed_TurnOff(FRONT_LEFT_LED);
        vTaskDelayUntil( &xLastWakeTime, 10/portTICK_RATE_MS );
	}
}

