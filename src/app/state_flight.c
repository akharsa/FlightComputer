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
#define K_PSI	200

float control[4]={0.0};
uint16_t inputs[4]={0};

qPID ctrl;

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
	//StartTelemetry(20);
}

void Flight_onExit(void *p){
	ConsolePuts_("FLIGHT State: onExit\r\n",BLUE);
	vTaskDelete(hnd);
	vTaskDelete(BeaconHnd);
	//StopTelemetry();
}

void Flight_Task(void * pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int16_t buffer[3];

    ctrl.AntiWindup = ENABLED;
    ctrl.Bumpless = ENABLED;

    ctrl.Mode = AUTOMATIC;
    ctrl.OutputMax = 1.0;
    ctrl.OutputMin = -1.0;

    ctrl.Ts = 0.01;

    ctrl.b = 1.0;
    ctrl.c = 1.0;

    ctrl.K = 0.008;
    ctrl.Ti = ctrl.K/0.007; //conversion de paralelo a ideal
    ctrl.Td = 0.005;
    ctrl.Nd = 5;

    qPID_Init(&ctrl);

    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	for(;;){
		if ((Joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) == 0){
			state_name_t newState=STATE_IDLE;
			qFSM_ChangeState(newState);
		}

		sv.setpoint[PSI_C] = map(Joystick.left_pad.x,0,255,-360.0,360.0);

		MPU6050_getRotation(&buffer[0],&buffer[1],&buffer[2]);

		sv.omega[0] = -(buffer[0]-settings.gyroBias[0]);
		sv.omega[1] = (buffer[1]-settings.gyroBias[1]);
		sv.omega[2] = -(buffer[2]-settings.gyroBias[2]);

		sv.omega[0] = sv.omega[0]/16.4;
		sv.omega[1] = sv.omega[1]/16.4;
		sv.omega[2] = sv.omega[2]/16.4;

		sv.CO[PSI_C] = qPID_Process(&ctrl,sv.setpoint[PSI_C],sv.omega[2],NULL);

		control[Z_C] = 0.3;
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

		vTaskDelayUntil( &xLastWakeTime, 5/portTICK_RATE_MS );
	}
}



