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
#include "MPU6050.h"

#include "quadrotor.h"
#include "taskList.h"

#include "qWDT.h"
#include "lpc17xx_gpio.h"
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

static 	uint8_t fifoBuffer[64]; // FIFO storage buffer
xSemaphoreHandle mpuSempahore;

int16_t buffer[3];

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint32_t uxHighWaterMark=0;
uint16_t fifoCount;     // count of all bytes currently in FIFO

uint8_t systemArmed = 0;
uint8_t systemArmedOld = 0;

float phi_atti;


float map(long x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


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

#ifdef ATTITUDE_MODE
	for (i=0;i<3;i++){
		quadrotor.attiController[i].AntiWindup = ENABLED;
		quadrotor.attiController[i].Bumpless = ENABLED;
		quadrotor.attiController[i].Mode = AUTOMATIC;
		quadrotor.attiController[i].OutputMax = 1.0;
		quadrotor.attiController[i].OutputMin = 1.0;
		quadrotor.attiController[i].Ts = 0.005;
		quadrotor.attiController[i].b = 1.0;
		quadrotor.attiController[i].c = 1.0;
		qPID_Init(&quadrotor.attiController[i]);
	}
#endif

	vSemaphoreCreateBinary(mpuSempahore);

	xSemaphoreTake(mpuSempahore,0);
	NVIC_EnableIRQ(EINT3_IRQn);
	MPU6050_setDMPEnabled(TRUE);

	mpuIntStatus = MPU6050_getIntStatus();
	packetSize = MPU6050_dmpGetFIFOPacketSize();

	ConsolePuts_("[OK]\r\n",GREEN);
}

void Flight_onEntry(void){
	//debug("FLIGHT: On entry\r\n");
	vTaskResume(BeaconHnd);

//	xSemaphoreTake(mpuSempahore,0);
//	NVIC_EnableIRQ(EINT3_IRQn);
//	MPU6050_setDMPEnabled(TRUE);

//	mpuIntStatus = MPU6050_getIntStatus();
//	packetSize = MPU6050_dmpGetFIFOPacketSize();
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
//	MPU6050_setDMPEnabled(FALSE);
//	NVIC_DisableIRQ(EINT3_IRQn);
	qLed_TurnOff(STATUS_LED);

}

void Flight_Task(void){
	uint8_t zc;

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
		qLed_TurnOn(STATUS_LED);

		//-----------------------------------------------------------------------
		// MPU Data adquisition
		//-----------------------------------------------------------------------

		//debug("Entering critical section for DMP");
		portENTER_CRITICAL();

		// reset interrupt flag and get INT_STATUS byte
		mpuIntStatus = MPU6050_getIntStatus();

		// get current FIFO count
		fifoCount = MPU6050_getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			MPU6050_resetFIFO();
			debug("DMP FIFO OVERFLOW!\r\n");

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = MPU6050_getFIFOCount();

			// read a packet from FIFO
			MPU6050_getFIFOBytes(fifoBuffer, packetSize);

			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

		}

		portEXIT_CRITICAL();
		//debug("Finished critical section");

		qLed_TurnOff(STATUS_LED);

		//-----------------------------------------------------------------------
		// Joystick mapping
		//-----------------------------------------------------------------------
		quadrotor.sv.setpoint[ALTITUDE] = map((quadrotor.joystick.left_pad.y>127)?127:255-quadrotor.joystick.left_pad.y,127,255,0.0,1.0);
		quadrotor.sv.setpoint[ROLL] = map(quadrotor.joystick.right_pad.x,0,255,-90.0,90.0);
		quadrotor.sv.setpoint[PITCH] = map(quadrotor.joystick.right_pad.y,0,255,-90.0,90.0);
		//quadrotor.sv.setpoint[YAW] = map(quadrotor.joystick.left_pad.x,0,255,-180.0,180.0);
		quadrotor.sv.setpoint[YAW] = 0.0; //THIS IS FOR KILL-ROT ON YAW

		//-----------------------------------------------------------------------
		// Angular velocity data
		//-----------------------------------------------------------------------

#define USE_GYRO_DMP

#ifndef USE_GYRO_RAW
		MPU6050_dmpGetGyro(&buffer[0],fifoBuffer);
		quadrotor.sv.rate[ROLL] = -buffer[0];
		quadrotor.sv.rate[PITCH] = buffer[1];
		quadrotor.sv.rate[YAW] = buffer[2];
#else
		// DAQ
		MPU6050_getRotation(&buffer[0],&buffer[1],&buffer[2]);

		// MPU axes aligment to Quad body axes
		quadrotor.sv.rate[ROLL] = -(buffer[0]-quadrotor.settings.gyroBias[ROLL]);
		quadrotor.sv.rate[PITCH] = (buffer[1]-quadrotor.settings.gyroBias[PITCH]);
		quadrotor.sv.rate[YAW] = -(buffer[2]-quadrotor.settings.gyroBias[YAW]);

		// MPU gyro scale transformation
		quadrotor.sv.rate[ROLL] = quadrotor.sv.rate[ROLL]/16.4;
		quadrotor.sv.rate[PITCH] = quadrotor.sv.rate[PITCH]/16.4;
		quadrotor.sv.rate[YAW] = quadrotor.sv.rate[YAW]/16.4;
#endif

		//-----------------------------------------------------------------------
		// Attitude data
		//-----------------------------------------------------------------------
		MPU6050_dmpGetEuler(&atti_buffer[0], fifoBuffer);
		//MPU6050_dmpGetYawPitchRoll(&atti_buffer[0], fifoBuffer);
		quadrotor.sv.attitude[ROLL] = atti_buffer[2]*180/3.141519;
		quadrotor.sv.attitude[PITCH] = -atti_buffer[1]*180/3.141519;;
		quadrotor.sv.attitude[YAW] = atti_buffer[0]*180/3.141519;;


		if (systemArmed == 0){
			qESC_SetOutput(MOTOR1,0);
			qESC_SetOutput(MOTOR2,0);
			qESC_SetOutput(MOTOR3,0);
			qESC_SetOutput(MOTOR4,0);
			//vTaskDelay(10/portTICK_RATE_MS);
			//debug("Idleing...");
		}else{
			//debug("Flying...");
#if 1

			//-----------------------------------------------------------------------
			// PID Process
			//-----------------------------------------------------------------------
			//debug("PID Controller start");

#ifdef ATTITUDE_MODE
			// ATTI
			phi_atti = qPID_Process(&ctrl[PHI_ATTI],sv.setpoint[PHI_C],sv.attitude[0],NULL);

			// RATE
			sv.CO[PHI_C] = qPID_Process(&ctrl[PHI_C],phi_atti,sv.rate[0],NULL);
			sv.CO[THETA_C] = qPID_Process(&ctrl[THETA_C],sv.setpoint[THETA_C],sv.rate[1],NULL);
			sv.CO[YAW] = qPID_Process(&ctrl[YAW],sv.setpoint[YAW],sv.rate[2],NULL);
			sv.CO[Z_C] = phi_atti;
#else
			quadrotor.sv.rateCtrlOutput[ROLL] = qPID_Procees(&quadrotor.rateController[ROLL],quadrotor.sv.setpoint[ROLL],quadrotor.sv.rate[ROLL]);
			quadrotor.sv.rateCtrlOutput[PITCH] = qPID_Procees(&quadrotor.rateController[PITCH],quadrotor.sv.setpoint[PITCH],quadrotor.sv.rate[PITCH]);
			quadrotor.sv.rateCtrlOutput[YAW] = qPID_Procees(&quadrotor.rateController[YAW],quadrotor.sv.setpoint[YAW],quadrotor.sv.rate[YAW]);
#endif
			//debug("PID Controller finish");

			//-----------------------------------------------------------------------
			// Output stage
			//-----------------------------------------------------------------------
#ifdef	USE_FULL_AUTOPILOT
			control[Z_C] = sv.setpoint[Z_C];
			control[PHI_C] = sv.CO[PHI_C];
			control[THETA_C] = sv.CO[THETA_C];
			control[YAW] = -sv.CO[YAW];

			control[ALTITUDE] = quadrotor.sv.setpoint[ALTITUDE];
			control[ROLL] = quadrotor.sv.rateCtrlOutput[ROLL];
			control[PITCH] = quadrotor.sv.rateCtrlOutput[PITCH];
			control[YAW] = 0.0; //quadrotor.sv.rateCtrlOutput[YAW];;
#else
			control[ALTITUDE] = quadrotor.sv.setpoint[ALTITUDE];
			control[ROLL] = quadrotor.sv.rateCtrlOutput[ROLL];
			control[PITCH] = quadrotor.sv.rateCtrlOutput[PITCH];
			control[YAW] = quadrotor.sv.rateCtrlOutput[YAW];
#endif
			// Output state
			quadrotor.sv.motorOutput[0] = (	control[ALTITUDE]*K_Z - control[ROLL]*K_PHI - control[PITCH]*K_THETA - control[YAW]*K_PSI	);
			quadrotor.sv.motorOutput[1] = (	control[ALTITUDE]*K_Z - control[ROLL]*K_PHI + control[PITCH]*K_THETA + control[YAW]*K_PSI	);
			quadrotor.sv.motorOutput[2] = (	control[ALTITUDE]*K_Z + control[ROLL]*K_PHI + control[PITCH]*K_THETA - control[YAW]*K_PSI	);
			quadrotor.sv.motorOutput[3] = (	control[ALTITUDE]*K_Z + control[ROLL]*K_PHI - control[PITCH]*K_THETA + control[YAW]*K_PSI	);

			// Motor command
			qESC_SetOutput(MOTOR1,quadrotor.sv.motorOutput[0]);
			qESC_SetOutput(MOTOR2,quadrotor.sv.motorOutput[1]);
			qESC_SetOutput(MOTOR3,quadrotor.sv.motorOutput[2]);
			qESC_SetOutput(MOTOR4,quadrotor.sv.motorOutput[3]);


#endif
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

