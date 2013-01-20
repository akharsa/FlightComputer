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
#include "MPU6050_6Axis_MotionApps20.h"

#include "qPIDs.h"
#include "quadrotor.h"
#include "taskList.h"

#include "telemetry.h"
#include "joystick.h"
#include "States.h"

#include "lpc17xx_gpio.h"

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

	//ConsolePuts_("IDLE State: onEntry\r\n",BLUE);
	xTaskCreate( Idle_Task, ( signed char * ) "IDLE", 300, ( void * ) NULL, IDLE_PRIORITY, &hnd );
}

void Idle_onExit(void * p){
	//ConsolePuts_("IDLE State: onExit\r\n",BLUE);
	vTaskDelete(hnd);
}

// MPU control/status vars
bool dmpReady = FALSE;  // set TRUE if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void Idle_Task(void * pvParameters){

	uint8_t i;

	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    GPIO_SetDir(1,(1<<8),0);


    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = MPU6050_dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
    	// turn on the DMP, now that it's ready
    	//Serial.println(F("Enabling DMP..."));
    	MPU6050_setDMPEnabled(TRUE);

    	mpuIntStatus = MPU6050_getIntStatus();

    	// get expected DMP packet size for later comparison
    	packetSize = MPU6050_dmpGetFIFOPacketSize();
    } else {
    	// ERROR!
    	// 1 = initial memory load failed
    	// 2 = DMP configuration updates failed
    	// (if it's going to break, usually the code will be 1)
    	//Serial.print(F("DMP Initialization failed (code "));
    	//Serial.print(devStatus);
    	//Serial.println(F(")"));
    }



    for (;;){
    	if ((GPIO_ReadValue(1)&(1<<8))==0){
    	    qLed_TurnOff(STATUS_LED);
    	}else{
    	    qLed_TurnOn(STATUS_LED);
    	}
    }


	for (;;)
	{

#if 0
		// ==================================================================
		if ((Joystick.buttons & (BTN_RIGHT2 | BTN_LEFT2)) != 0){
			state_name_t newState=STATE_FLIGHT;
			qFSM_ChangeState(newState);
		}
		// ==================================================================
#endif



	    // wait for MPU interrupt or extra packet(s) available
	    while ((GPIO_ReadValue(1)&(1<<8)==0) && fifoCount < packetSize) {
	    	vTaskDelay( 1/portTICK_RATE_MS);
	    }

	    while ((GPIO_ReadValue(1)&(1<<8)!=0)) {
	    	vTaskDelay( 1/portTICK_RATE_MS);
	    }

	    qLed_TurnOn(STATUS_LED);

	    // reset interrupt flag and get INT_STATUS byte
	    mpuIntStatus = MPU6050_getIntStatus();

	    // get current FIFO count
	    fifoCount = MPU6050_getFIFOCount();

	    // check for overflow (this should never happen unless our code is too inefficient)
	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        MPU6050_resetFIFO();
	        //Serial.println(F("FIFO overflow!"));

	    // otherwise, check for DMP data ready interrupt (this should happen frequently)
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount < packetSize) fifoCount = MPU6050_getFIFOCount();

	        // read a packet from FIFO
	        MPU6050_getFIFOBytes(fifoBuffer, packetSize);

	        // track FIFO count here in case there is > 1 packet available
	        // (this lets us immediately read more without waiting for an interrupt)
	        fifoCount -= packetSize;

            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            qUART_Send(UART_GROUNDCOMM,teapotPacket,14);
            //Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
	    }

	    qLed_TurnOff(STATUS_LED);
	    //vTaskDelayUntil( &xLastWakeTime, 10/portTICK_RATE_MS );
	}
}
