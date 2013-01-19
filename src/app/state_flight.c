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

#define Z_C		0
#define PHI_C	1
#define THETA_C	2
#define PSI_C	3

#define K_Z		800
#define K_PHI	200
#define K_THETA	200
#define K_PSI	300

float control[4]={0.0};
uint16_t inputs[4]={0};

qPID ctrl[4];

//=======================================================
uint32_t signal_time[]={0,2*1000/5,6*1000/5,10*1000/5,14*1000/5};
float signal_values[]={0.0,180.0,0.0,-180.0,0.0};
int j;
uint32_t signal_t=0;
//=======================================================

float map(long x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
	StartTelemetry(20);
}

void Flight_onExit(void *p){
	ConsolePuts_("FLIGHT State: onExit\r\n",BLUE);
	vTaskDelete(hnd);
	vTaskDelete(BeaconHnd);
	StopTelemetry();
}


void Flight_Task(void * pvParameters){
	uint8_t i;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int16_t buffer[3];

	for (i=1;i<4;i++){
		ctrl[i].AntiWindup = ENABLED;
		ctrl[i].Bumpless = ENABLED;

		ctrl[i].Mode = AUTOMATIC;
		ctrl[i].OutputMax = 1.0;
		ctrl[i].OutputMin = -1.0;

		ctrl[i].Ts = 0.005;

		ctrl[i].b = 1.0;
		ctrl[i].c = 1.0;
	}

	ctrl[PHI_C].K = 0.02;
	ctrl[PHI_C].Ti = 1/0.03;
	ctrl[PHI_C].Td = 0.000;
	ctrl[PHI_C].Nd = 5;

    ctrl[THETA_C].K = 0.02;
    ctrl[THETA_C].Ti = 1/0.03;
    ctrl[THETA_C].Td = 0.000;
    ctrl[THETA_C].Nd = 5;

    ctrl[PSI_C].K = 0.1;
    ctrl[PSI_C].Ti = 1/0.2;
    ctrl[PSI_C].Td = 0.000;
    ctrl[PSI_C].Nd = 5;

    qPID_Init(&ctrl[PHI_C]);
    qPID_Init(&ctrl[THETA_C]);
    qPID_Init(&ctrl[PSI_C]);


    //uint32_t signal_t=0;

	for(;;){
		if ((Joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) == 0){
			state_name_t newState=STATE_IDLE;
			qFSM_ChangeState(newState);
		}

		sv.setpoint[PHI_C] = map(Joystick.right_pad.x,0,255,-90.0,90.0);
		sv.setpoint[THETA_C] = map(Joystick.right_pad.y,0,255,-90.0,90.0);
		sv.setpoint[PSI_C] = map(Joystick.left_pad.x,0,255,-180.0,180.0);

		//sv.setpoint[PHI_C] = 0.0;
/*
		for (j=0;j<(sizeof(signal_time)/4);j++){
			if (signal_t>=signal_time[j]){
				sv.setpoint[PSI_C] = signal_values[j];
			}
		}
*/

		// DAQ
		MPU6050_getRotation(&buffer[0],&buffer[1],&buffer[2]);

		sv.omega[0] = -(buffer[0]-settings.gyroBias[0]);
		sv.omega[1] = (buffer[1]-settings.gyroBias[1]);
		sv.omega[2] = -(buffer[2]-settings.gyroBias[2]);

		sv.omega[0] = sv.omega[0]/16.4;
		sv.omega[1] = sv.omega[1]/16.4;
		sv.omega[2] = sv.omega[2]/16.4;

		// PID Process
		sv.CO[PHI_C] = qPID_Process(&ctrl[PHI_C],sv.setpoint[PHI_C],sv.omega[0],NULL);
		sv.CO[THETA_C] = qPID_Process(&ctrl[THETA_C],sv.setpoint[THETA_C],sv.omega[1],NULL);
		sv.CO[PSI_C] = qPID_Process(&ctrl[PSI_C],sv.setpoint[PSI_C],sv.omega[2],NULL);

		control[Z_C] = 0.3;
		control[PHI_C] = sv.CO[PHI_C];
		control[THETA_C] = sv.CO[THETA_C];
		control[PSI_C] = 0.0;

		// Output stateg
		inputs[0] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI - control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
		inputs[1] = (	control[Z_C]*K_Z - control[PHI_C]*K_PHI + control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);
		inputs[2] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI + control[THETA_C]*K_THETA - control[PSI_C]*K_PSI	);
		inputs[3] = (	control[Z_C]*K_Z + control[PHI_C]*K_PHI - control[THETA_C]*K_THETA + control[PSI_C]*K_PSI	);

		qESC_SetOutput(MOTOR1,inputs[0]);
		qESC_SetOutput(MOTOR2,inputs[1]);
		qESC_SetOutput(MOTOR3,inputs[2]);
		qESC_SetOutput(MOTOR4,inputs[3]);
/*
		if (signal_t<(20*1000/5)){
			signal_t++;
		}else{
			for(j=0;j<10;j++){
				qESC_SetOutput(MOTOR1,1);
				qESC_SetOutput(MOTOR2,1);
				qESC_SetOutput(MOTOR3,1);
				qESC_SetOutput(MOTOR4,1);
			}
			vTaskDelete(NULL);
		}
*/
		vTaskDelayUntil( &xLastWakeTime, 5/portTICK_RATE_MS );
	}
}



