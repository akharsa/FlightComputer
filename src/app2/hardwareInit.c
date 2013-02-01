/*
 * hardwareInit.c
 *
 *  Created on: 01/02/2013
 *      Author: alan
 */


#include "qUART.h"
#include "qI2C.h"
#include "MPU6050.h"
#include "qESC.h"
#include "lpc17xx_gpio.h"
#include "DebugConsole.h"
#include "quadrotor.h"


void hardwareInit(void){
	uint8_t i,j;


	// Early init will turn the motors off for safety
	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	// --------------------------------------------------
	//	Leds Initialization
	// --------------------------------------------------
	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOn(leds[i]);
	}

	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);

	// --------------------------------------------------
	// MPU initializationand calibration
	// --------------------------------------------------
	int32_t sum[3];
	int16_t buffer[3];

	debug("Calibrating sensors...");

	if (qI2C_Init()!=SUCCESS) halt("I2C INIT ERROR");

	if (MPU6050_testConnection()==TRUE){
		MPU6050_initialize();
	}else{
		halt("MPU6050 Init ERROR\r\n");
	}

	debug("MPU6050 Found and initialized");

#if (GYRO_MODE == GYRO_RAW)
	debug("Using GYRO in RAW mode");
	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
	//FIXME: Don't know why this is happening but need to be called a few times
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	debug("Getting GYRO bias");
	for (i=0;i<128;i++){
		MPU6050_getRotation(&buffer[0],&buffer[1],&buffer[2]);
		sum[0] += buffer[0];
		sum[1] += buffer[1];
		sum[2] += buffer[2];
		vTaskDelay(10/portTICK_RATE_MS);
	}

	settings.gyroBias[0] = (int16_t)sum[0]/128;
	settings.gyroBias[1] = (int16_t)sum[1]/128;
	settings.gyroBias[2] = (int16_t)sum[2]/128;
#else
	debug("Using GYRO in DMP mode");
#endif

	// --------------------------------------------------
	// DMP configuration
	// --------------------------------------------------

	debug("Initializing MPU6050 DMP...");

	// GPIO0.4 as input with interrupt
	GPIO_SetDir(0,(1<<4),0);
	GPIO_IntCmd(0,(1<<4),1);
	GPIO_ClearInt(0,(1<<4));
	NVIC_EnableIRQ(EINT3_IRQn);

	// make sure it worked (returns 0 if so)
	if (MPU6050_dmpInitialize() == 0) {
		debug("MPU6050 DMP OK");
	} else {
		halt("MPU6050 DMP initilization failed");
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
	}

	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);

	debug("Waiting 8 seconds for Xbee..");

	for (j=0;j<80;j++){
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
	}

	// --------------------------------------------------
	// UART init
	// --------------------------------------------------
	debug("Initializing UART");

	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)==RET_ERROR){
		halt("UART Initialization error");
	}

	qUART_EnableTx(UART_GROUNDCOMM);
	debug("UART Enabled OK");

	ConsolePuts("\x1B[2J\x1B[0;0f");
	ConsolePuts_("FLC V2.0 Initialized...\r\n",BLUE);

	debug("Hardware initialization complete");
}
