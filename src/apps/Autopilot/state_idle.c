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
#include "quadrotor.h"
#include "taskList.h"

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
	xTaskCreate(Idle_Task, ( signed char * ) "IDLE", 300, ( void * ) NULL, IDLE_PRIORITY, &hnd );
}

void Idle_onExit(void * p){
	vTaskDelete(hnd);
}

void Idle_Task(void * pvParameters){
	int i,j;

//	float gyroz;
//	float CO;

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
    ctrl.Td = 0.001;
    ctrl.Nd = 5;

    qPID_Init(&ctrl);

//	qWDT_Start(500000);

    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    for(;;){
    	vTaskDelayUntil( &xLastWakeTime, 10/portTICK_RATE_MS );
    }

	for (;;)
	{

		qLed_TurnOn(FRONT_LEFT_LED);

		MPU6050_getRotation(&buffer[0],&buffer[1],&buffer[2]);

		sv.omega[0] = TODEGSEC(buffer[0]-settings.gyroBias[0]);
		sv.omega[1] = TODEGSEC(buffer[1]-settings.gyroBias[1]);
		sv.omega[2] = TODEGSEC(buffer[2]-settings.gyroBias[2]);

		sv.omega[0] = sv.omega[0]/16.4;
		sv.omega[1] = sv.omega[1]/16.4;
		sv.omega[2] = sv.omega[2]/16.4;

		for (j=0;j<(sizeof(signal_time)/4);j++){
			if (signal_t>=signal_time[j]){
				sv.setpoint[PSI_C] = signal_values[j];
			}
		}

		sv.CO[PSI_C] = qPID_Process(&ctrl,sv.setpoint[PSI_C],-sv.omega[2],NULL);

		control[Z_C] = 0.2;
		control[PHI_C] = 0;
		control[THETA_C] = 0;
		control[PSI_C] = sv.CO[PSI_C];

		inputs[0] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI - control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
		inputs[1] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI + control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);
		inputs[2] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI + control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
		inputs[3] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI - control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);


		qESC_SetOutput(MOTOR1,inputs[0]);
		qESC_SetOutput(MOTOR2,inputs[1]);
		qESC_SetOutput(MOTOR3,inputs[2]);
		qESC_SetOutput(MOTOR4,inputs[3]);

/*
		if (signal_t>500 && signal_t<1000 ){
			MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);
		}else if (signal_t>1000 && signal_t<1500 ){
			MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
		}else if (signal_t>1500){
			MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
		}
*/
		if (signal_t<2000){
			signal_t++;
			qComms_SendMsg(UART_GROUNDCOMM,0xBB,MSG_TYPE_TELEMETRY,sizeof(float)*11,(uint8_t*)&sv);
		}else{
			for (i=0;i<10;i++){
				qESC_SetOutput(MOTOR1,1);
				qESC_SetOutput(MOTOR2,1);
				qESC_SetOutput(MOTOR3,1);
				qESC_SetOutput(MOTOR4,1);
				vTaskDelay(10/portTICK_RATE_MS);
			}

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

