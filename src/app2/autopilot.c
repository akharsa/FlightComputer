#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "qESC.h"
#include "leds.h"
#include "qWDT.h"

#include "board.h"
#include "DebugConsole.h"
#include "qCOMMS.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"


#include "quadrotor.h"
#include "taskList.h"

#include "qWDT.h"
#include "lpc17xx_gpio.h"

#include "math.h"

#include "timing.h"
/* ================================ */
/* Prototypes	 					*/
/* ================================ */

void Flight_Task();
void Flight_onEntry();
void Fligth_onExit();

/* ================================ */
/* Private globals 					*/
/* ================================ */
static xTaskHandle hnd;
extern xTaskHandle BeaconHnd;

float control[4]={0.0};

float atti_buffer[3];

xSemaphoreHandle mpuSempahore;

int16_t buffer[3];

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint32_t uxHighWaterMark=0;
uint16_t fifoCount;     // count of all bytes currently in FIFO

uint8_t systemArmed = 0;
uint8_t systemArmedOld = 0;

float phi_atti;


int16_t gyro[3];
int16_t accel[3];
int32_t quat[4];
int16_t sensors;
uint8_t more;

float scale;
float atti_bias[3];

#define 	HIST_SIZE	400
float history[HIST_SIZE];
uint32_t histindex = 0;

float map(long x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


typedef enum{ ATTI_MODE=0, STABILIZE_MODE, ACRO_MODE, NO_YAW_MODE} FlyingMode_t;

FlyingMode_t FlyingMode = STABILIZE_MODE;

//#define ATTITUDE_MODE
//#undef ATTITUDE_MODE

void Flight_onTimeStartup(void){
	uint8_t i;
	debug("Configuring PIDS...");

	for (i=0;i<3;i++){
		quadrotor.rateController[i].AntiWindup = ENABLED;
		quadrotor.rateController[i].Bumpless = ENABLED;
		quadrotor.rateController[i].Mode = AUTOMATIC;
		quadrotor.rateController[i].OutputMax = 1.0;
		quadrotor.rateController[i].OutputMin = -1.0;
		quadrotor.rateController[i].Ts = 0.005;
		quadrotor.rateController[i].b = 1.0;
		quadrotor.rateController[i].c = 1.0;
		qPID_Init(&quadrotor.rateController[i]);
	}

	for (i=0;i<3;i++){
		quadrotor.attiController[i].AntiWindup = ENABLED;
		quadrotor.attiController[i].Bumpless = ENABLED;
		quadrotor.attiController[i].Mode = AUTOMATIC;
		quadrotor.attiController[i].OutputMax = 250.0;
		quadrotor.attiController[i].OutputMin = -250.0;
		quadrotor.attiController[i].Ts = 0.005;
		quadrotor.attiController[i].b = 1.0;
		quadrotor.attiController[i].c = 1.0;
		qPID_Init(&quadrotor.attiController[i]);
	}

	vSemaphoreCreateBinary(mpuSempahore);

	xSemaphoreTake(mpuSempahore,0);
	NVIC_EnableIRQ(EINT3_IRQn);

	ConsolePuts_("[OK]\r\n",GREEN);
}

void Flight_onEntry(void){
	//debug("FLIGHT: On entry\r\n");

	if ((quadrotor.joystick.buttons & BTN_CROSS) != 0){
		FlyingMode = ACRO_MODE;
	}else if ((quadrotor.joystick.buttons & BTN_CIRCLE) != 0){
		FlyingMode = STABILIZE_MODE;
	}else if ((quadrotor.joystick.buttons & BTN_SQUARE) != 0){
		FlyingMode = ATTI_MODE;
	}else if ((quadrotor.joystick.buttons & BTN_TRIANGLE) != 0){
		FlyingMode = NO_YAW_MODE;
	}else{
		FlyingMode = STABILIZE_MODE;
	}
/*
 * Estos delays cagan TODO!
	for(j=0;j<mode;j++){
			for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
			vTaskDelay(200/portTICK_RATE_MS);
			for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
			vTaskDelay(200/portTICK_RATE_MS);
	}
*/
	vTaskResume(BeaconHnd);
}

void Flight_onExit(void){
	uint8_t i;

	//debug("FLIGHT: On exit\r\n");
	for (i=0;i<10;i++){
		qESC_SetOutput(MOTOR1,0);
		qESC_SetOutput(MOTOR2,0);
		qESC_SetOutput(MOTOR3,0);
		qESC_SetOutput(MOTOR4,0);
	}
	vTaskSuspend(BeaconHnd);
	qLed_TurnOff(STATUS_LED);
}


uint8_t MPU6050_dmpGetEuler(float *euler, int32_t q[]) {

	float q1[4];
	uint8_t i;

	for(i = 0; i < 4; i++ ) {
		q1[i] = ((float) (q[i]>>16)) / 16384.0f;
	}

	euler[0] = atan2(2*q1[1]*q1[2] - 2*q1[0]*q1[3], 2*q1[0]*q1[0] + 2*q1[1]*q1[1] - 1);
	euler[1] = -asin(2*q1[1]*q1[3] + 2*q1[0]*q1[2]);
	euler[2] = atan2(2*q1[2]*q1[3] - 2*q1[0]*q1[1], 2*q1[0]*q1[0] + 2*q1[3]*q1[3] - 1);

	return 0;
}


void Flight_Task(void){
	uint8_t firstTime = 0;
	uint8_t i;

	mpu_get_gyro_sens(&scale);

	timerHnd timer;
	timerConfig(1,&timer);

	for(;;){

		// =======================================================================
		// State things
		// =======================================================================
		systemArmedOld = systemArmed;
		if ((quadrotor.joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) != 0){
			systemArmed = 1;
		}else{
			systemArmed = 0;
		}

		// Transition handling
		if ((systemArmed == 1) && (systemArmedOld == 0)){
			//debug("Arming system");
			Flight_onEntry();
		}else if ((systemArmed == 0) && (systemArmedOld == 1)){
			//debug("Disarming system");
			Flight_onExit();
		}

		// =======================================================================
		// Data adquisition
		// =======================================================================

		// Wait here for MPU DMP interrupt at 200Hz
		xSemaphoreTake(mpuSempahore,portMAX_DELAY); //FIXME: instead of portMAX it would be nice to have a time out for errors

		//debug("Got DMP data!");
		//qLed_TurnOn(STATUS_LED);

		//-----------------------------------------------------------------------
		// MPU Data adquisition
		//-----------------------------------------------------------------------
		//debug("MPU download start");
		portENTER_CRITICAL();
        dmp_read_fifo(gyro, accel, quat, NULL, &sensors, &more);
		portEXIT_CRITICAL();
		MPU6050_dmpGetEuler(atti_buffer,quat);
		//debug("MPU download end");
		//qLed_TurnOff(STATUS_LED);

		//-----------------------------------------------------------------------
		// Joystick mapping
		//-----------------------------------------------------------------------
		//debug("JoyMap download start");
		switch (FlyingMode){
			case ACRO_MODE:
				quadrotor.sv.setpoint[ALTITUDE] = map((quadrotor.joystick.left_pad.y>127)?127:255-quadrotor.joystick.left_pad.y,127,255,0.0,1.0);
				quadrotor.sv.setpoint[ROLL] = map(quadrotor.joystick.right_pad.x,0,255,-90.0,90.0);
				quadrotor.sv.setpoint[PITCH] = map(quadrotor.joystick.right_pad.y,0,255,-90.0,90.0);
				// TODO: this should be deactivated via ground control stations and send always a 0
				quadrotor.sv.setpoint[YAW] = map(quadrotor.joystick.left_pad.x,0,255,-180.0,180.0);
				//quadrotor.sv.setpoint[YAW] = 0.0; //THIS IS FOR KILL-ROT ON YAW
				break;

			case STABILIZE_MODE:
				quadrotor.sv.setpoint[ALTITUDE] = map((quadrotor.joystick.left_pad.y>127)?127:255-quadrotor.joystick.left_pad.y,127,255,0.0,1.0);
				quadrotor.sv.setpoint[ROLL] = map(quadrotor.joystick.right_pad.x,0,255,-40.0,40.0);
				quadrotor.sv.setpoint[PITCH] = map(quadrotor.joystick.right_pad.y,0,255,-40.0,40.0);
				quadrotor.sv.setpoint[YAW] = map(quadrotor.joystick.left_pad.x,0,255,-180.0,180.0);
				break;
			case ATTI_MODE:
				quadrotor.sv.setpoint[ALTITUDE] = map((quadrotor.joystick.left_pad.y>127)?127:255-quadrotor.joystick.left_pad.y,127,255,0.0,1.0);
				quadrotor.sv.setpoint[ROLL] = map(quadrotor.joystick.right_pad.x,0,255,-40.0,40.0);
				quadrotor.sv.setpoint[PITCH] = map(quadrotor.joystick.right_pad.y,0,255,-40.0,40.0);
				quadrotor.sv.setpoint[YAW] = map(quadrotor.joystick.left_pad.x,0,255,-45.0,45.0);
				break;
			case NO_YAW_MODE:
				quadrotor.sv.setpoint[ALTITUDE] = map((quadrotor.joystick.left_pad.y>127)?127:255-quadrotor.joystick.left_pad.y,127,255,0.0,1.0);
				quadrotor.sv.setpoint[ROLL] = map(quadrotor.joystick.right_pad.x,0,255,-40.0,40.0);
				quadrotor.sv.setpoint[PITCH] = map(quadrotor.joystick.right_pad.y,0,255,-40.0,40.0);
				quadrotor.sv.setpoint[YAW] = map(quadrotor.joystick.left_pad.x,0,255,-180.0,180.0); // not being used
				break;
		}
		//debug("JoyMap download end");
		//-----------------------------------------------------------------------
		// Angular velocity data
		//-----------------------------------------------------------------------
		// Data scaling and axis rotatation
		quadrotor.sv.rate[ROLL] = -gyro[0]/scale;
		quadrotor.sv.rate[PITCH] = gyro[1]/scale;
		quadrotor.sv.rate[YAW] = -gyro[2]/scale;

		//-----------------------------------------------------------------------
		// Attitude data
		//-----------------------------------------------------------------------
		// Data scaling and axis rotatation
		quadrotor.sv.attitude[ROLL] = atti_buffer[2]*180/3.141519;
		quadrotor.sv.attitude[PITCH] = -atti_buffer[1]*180/3.141519;;
		quadrotor.sv.attitude[YAW] = atti_buffer[0]*180/3.141519;;

#define GYRO_THRESHOLD 0.8
#define ATTI_THRESHOLD 3.0

		if ((quadrotor.sv.rate[ROLL]<=GYRO_THRESHOLD) && (quadrotor.sv.rate[PITCH]<=GYRO_THRESHOLD) && (quadrotor.sv.rate[YAW]<=GYRO_THRESHOLD)){
			qLed_TurnOn(STATUS_LED);
		}else{
			qLed_TurnOff(STATUS_LED);
		}

		//-----------------------------------------------------------------------
		// Biasing
		//-----------------------------------------------------------------------
		if ((quadrotor.joystick.buttons & BTN_START) != 0){
			atti_bias[ROLL] = quadrotor.sv.attitude[ROLL];
			atti_bias[PITCH] = quadrotor.sv.attitude[PITCH];
			atti_bias[YAW] = quadrotor.sv.attitude[YAW];

			uint8_t i;
			for (i=0;i<3;i++){
				quadrotor.rateController[i].AntiWindup = ENABLED;
				quadrotor.rateController[i].Bumpless = ENABLED;
				quadrotor.rateController[i].Mode = AUTOMATIC;
				quadrotor.rateController[i].OutputMax = 1.0;
				quadrotor.rateController[i].OutputMin = -1.0;
				quadrotor.rateController[i].Ts = 0.005;
				quadrotor.rateController[i].b = 1.0;
				quadrotor.rateController[i].c = 1.0;
				qPID_Init(&quadrotor.rateController[i]);
			}

			for (i=0;i<3;i++){
				quadrotor.attiController[i].AntiWindup = ENABLED;
				quadrotor.attiController[i].Bumpless = ENABLED;
				quadrotor.attiController[i].Mode = AUTOMATIC;
				quadrotor.attiController[i].OutputMax = 250.0;
				quadrotor.attiController[i].OutputMin = -250.0;
				quadrotor.attiController[i].Ts = 0.005;
				quadrotor.attiController[i].b = 1.0;
				quadrotor.attiController[i].c = 1.0;
				qPID_Init(&quadrotor.attiController[i]);
			}
		}

		quadrotor.sv.attitude[ROLL] -= atti_bias[ROLL];
		quadrotor.sv.attitude[PITCH] -= atti_bias[PITCH];
		quadrotor.sv.attitude[YAW] -= atti_bias[YAW];

		if (systemArmed == 0){
			if ((fabsf(quadrotor.sv.attitude[ROLL])<=ATTI_THRESHOLD) && (fabsf(quadrotor.sv.attitude[PITCH])<=ATTI_THRESHOLD) && (fabsf(quadrotor.sv.attitude[YAW])<=ATTI_THRESHOLD)){
				for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
			}else{
				for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
			}
		}

		//-----------------------------------------------------------------------
		// PID Process
		//-----------------------------------------------------------------------
		//debug("PID Controller start");
		//debug("PIDs download start");
		switch (FlyingMode){
			case ACRO_MODE:
				quadrotor.sv.rateCtrlOutput[ROLL] = qPID_Procees(&quadrotor.rateController[ROLL],quadrotor.sv.setpoint[ROLL],quadrotor.sv.rate[ROLL]);
				quadrotor.sv.rateCtrlOutput[PITCH] = qPID_Procees(&quadrotor.rateController[PITCH],quadrotor.sv.setpoint[PITCH],quadrotor.sv.rate[PITCH]);
				quadrotor.sv.rateCtrlOutput[YAW] = qPID_Procees(&quadrotor.rateController[YAW],quadrotor.sv.setpoint[YAW],quadrotor.sv.rate[YAW]);
				break;

			case STABILIZE_MODE:
				// CAS
				//uint32_t exec;
				//timerStart(&timer);

				quadrotor.sv.attiCtrlOutput[ROLL] = qPID_Procees(&quadrotor.attiController[ROLL],quadrotor.sv.setpoint[ROLL],quadrotor.sv.attitude[ROLL]);

				//timerStop(&timer);
				//exec = timerGetElapsed(&timer);
				quadrotor.sv.attiCtrlOutput[PITCH] = qPID_Procees(&quadrotor.attiController[PITCH],quadrotor.sv.setpoint[PITCH],quadrotor.sv.attitude[PITCH]);
				//quadrotor.sv.attiCtrlOutput[YAW] = qPID_Procees(&quadrotor.attiController[YAW],quadrotor.sv.setpoint[YAW],quadrotor.sv.attitude[YAW]);

				//		quadrotor.sv.attiCtrlOutput[ROLL] = 0.0;

				// SAS
				quadrotor.sv.rateCtrlOutput[ROLL] = qPID_Procees(&quadrotor.rateController[ROLL],quadrotor.sv.attiCtrlOutput[ROLL],quadrotor.sv.rate[ROLL]);
				quadrotor.sv.rateCtrlOutput[PITCH] = qPID_Procees(&quadrotor.rateController[PITCH],quadrotor.sv.attiCtrlOutput[PITCH],quadrotor.sv.rate[PITCH]);
				//quadrotor.sv.rateCtrlOutput[YAW] = qPID_Procees(&quadrotor.rateController[YAW],quadrotor.sv.attiCtrlOutput[YAW],quadrotor.sv.rate[YAW]);
				quadrotor.sv.rateCtrlOutput[YAW] = qPID_Procees(&quadrotor.rateController[YAW],quadrotor.sv.setpoint[YAW],quadrotor.sv.rate[YAW]);
				/*
						history[histindex++]=quadrotor.sv.rateCtrlOutput[ROLL];

						if (histindex==HIST_SIZE){
							while(1);
						}
				 */

				break;

			case ATTI_MODE:
				// CAS
				quadrotor.sv.attiCtrlOutput[ROLL] = qPID_Procees(&quadrotor.attiController[ROLL],quadrotor.sv.setpoint[ROLL],quadrotor.sv.attitude[ROLL]);
				quadrotor.sv.attiCtrlOutput[PITCH] = qPID_Procees(&quadrotor.attiController[PITCH],quadrotor.sv.setpoint[PITCH],quadrotor.sv.attitude[PITCH]);
				quadrotor.sv.attiCtrlOutput[YAW] = qPID_Procees(&quadrotor.attiController[YAW],quadrotor.sv.setpoint[YAW],quadrotor.sv.attitude[YAW]);

				// SAS
				quadrotor.sv.rateCtrlOutput[ROLL] = qPID_Procees(&quadrotor.rateController[ROLL],quadrotor.sv.attiCtrlOutput[ROLL],quadrotor.sv.rate[ROLL]);
				quadrotor.sv.rateCtrlOutput[PITCH] = qPID_Procees(&quadrotor.rateController[PITCH],quadrotor.sv.attiCtrlOutput[PITCH],quadrotor.sv.rate[PITCH]);
				quadrotor.sv.rateCtrlOutput[YAW] = qPID_Procees(&quadrotor.rateController[YAW],quadrotor.sv.attiCtrlOutput[YAW],quadrotor.sv.rate[YAW]);

				break;
			case NO_YAW_MODE:
				// CAS
				quadrotor.sv.attiCtrlOutput[ROLL] = qPID_Procees(&quadrotor.attiController[ROLL],quadrotor.sv.setpoint[ROLL],quadrotor.sv.attitude[ROLL]);
				quadrotor.sv.attiCtrlOutput[PITCH] = qPID_Procees(&quadrotor.attiController[PITCH],quadrotor.sv.setpoint[PITCH],quadrotor.sv.attitude[PITCH]);
				quadrotor.sv.attiCtrlOutput[YAW] = qPID_Procees(&quadrotor.attiController[YAW],quadrotor.sv.setpoint[YAW],quadrotor.sv.attitude[YAW]);

				// SAS
				quadrotor.sv.rateCtrlOutput[ROLL] = qPID_Procees(&quadrotor.rateController[ROLL],quadrotor.sv.attiCtrlOutput[ROLL],quadrotor.sv.rate[ROLL]);
				quadrotor.sv.rateCtrlOutput[PITCH] = qPID_Procees(&quadrotor.rateController[PITCH],quadrotor.sv.attiCtrlOutput[PITCH],quadrotor.sv.rate[PITCH]);
				quadrotor.sv.rateCtrlOutput[YAW] = 0.0;

				break;

		}
		//debug("PIDS download finish");

		//-----------------------------------------------------------------------
		// Output stage
		//-----------------------------------------------------------------------

		control[ROLL] = quadrotor.sv.rateCtrlOutput[ROLL];
		control[PITCH] =  quadrotor.sv.rateCtrlOutput[PITCH];
		control[YAW] = -quadrotor.sv.rateCtrlOutput[YAW]; //FIXME: there is a problem with the sign (maybe in the Mq)
		control[ALTITUDE] = quadrotor.sv.setpoint[ALTITUDE];

		// Output state
		quadrotor.sv.motorOutput[0] = (	control[ALTITUDE]*K_Z - control[ROLL]*K_PHI - control[PITCH]*K_THETA - control[YAW]*K_PSI	);
		quadrotor.sv.motorOutput[1] = (	control[ALTITUDE]*K_Z - control[ROLL]*K_PHI + control[PITCH]*K_THETA + control[YAW]*K_PSI	);
		quadrotor.sv.motorOutput[2] = (	control[ALTITUDE]*K_Z + control[ROLL]*K_PHI + control[PITCH]*K_THETA - control[YAW]*K_PSI	);
		quadrotor.sv.motorOutput[3] = (	control[ALTITUDE]*K_Z + control[ROLL]*K_PHI - control[PITCH]*K_THETA + control[YAW]*K_PSI	);

		if (systemArmed == 0){
			qESC_SetOutput(MOTOR1,0);
			qESC_SetOutput(MOTOR2,0);
			qESC_SetOutput(MOTOR3,0);
			qESC_SetOutput(MOTOR4,0);
		}else{
			// Motor command
			qESC_SetOutput(MOTOR1,quadrotor.sv.motorOutput[0]);
			qESC_SetOutput(MOTOR2,quadrotor.sv.motorOutput[1]);
			qESC_SetOutput(MOTOR3,quadrotor.sv.motorOutput[2]);
			qESC_SetOutput(MOTOR4,quadrotor.sv.motorOutput[3]);

		}
	}
}

// Hardware IRQ for the MPU6050 DMP algorithm.
void EINT3_IRQHandler(void)
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

	if(GPIO_GetIntStatus(0, 4, 1))
	{
		GPIO_ClearInt(0,(1<<4));
		if (mpuSempahore!=NULL){
			xSemaphoreGiveFromISR(mpuSempahore,&xHigherPriorityTaskWoken);
		}
	}
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

