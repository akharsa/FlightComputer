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
#include "eeprom.h"
#include "board.h"
#include "quadrotor.h"
#include "nvram.h"


void halt(void){
	int i=0;

	while(1){
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

void hardwareInit(void){
	uint8_t i,j;

	// =========================================================
	// Early init will turn the motors off for safety if a reset occurs
	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	// =========================================================
	//	Leds Initialization

	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOn(leds[i]);
	}
	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);

	// =========================================================
	// Startup waiting for xbee
	for (j=0;j<80;j++){
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
	}

	// =========================================================
	// UART init
	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)!=RET_OK){
		halt();
	}

	qUART_EnableTx(UART_GROUNDCOMM);

	ConsolePuts("\x1B[2J\x1B[0;0f");
	ConsolePuts_("FLC V2.0 Initialized...\r\n",BLUE);

	// =========================================================
	// I2C initialization
	debug("Initializing I2C interface...");
	if (qI2C_Init()!=SUCCESS){
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}
	ConsolePuts_("[OK]\r\n",GREEN);

	// =========================================================
	// NVRAM
#undef WRITE_NVRAM
#ifdef WRITE_NVRAM
	debug("Writing defaults to NVRAM...");
	if (qNVRAM_setDefaults(&nvramBuffer)!=SUCCESS){
		halt();
		ConsolePuts_("[ERROR]\r\n",RED);
	}
	if (qNVRAM_Save(&nvramBuffer)!=SUCCES){
		halt();
		ConsolePuts_("[ERROR]\r\n",RED);
	}
	ConsolePuts_("[OK]\r\n",GREEN);
#endif
	debug("Loading configuration from NVRAM...");
	if (qNVRAM_Load(&nvramBuffer)!=SUCCESS){
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}
	ConsolePuts_("[OK]\r\n",GREEN);

	// =========================================================
	// MPU initialization and calibration
	int32_t sum[3];
	int16_t buffer[3];

	//debug("Calibrating sensors...\r\n");

	debug("Searching for MPU6050 IMU...");
	if (MPU6050_testConnection()==TRUE){
		ConsolePuts_("[OK]\r\n",GREEN);
		MPU6050_initialize();
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}

	debug("MPU6050 initialized\r\n");

#if (GYRO_MODE == GYRO_RAW)
	debug("Using GYRO in RAW mode\r\n");
	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
	//FIXME: Don't know why this is happening but need to be called a few times
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	debug("Getting GYRO bias\r\n");
	for (i=0;i<128;i++){
		MPU6050_getRotation(&buffer[0],&buffer[1],&buffer[2]);
		sum[0] += buffer[0];
		sum[1] += buffer[1];
		sum[2] += buffer[2];
		vTaskDelay(10/portTICK_RATE_MS);
	}

	quadrotor.settings.gyroBias[ROLL] = (int16_t)sum[0]/128;
	quadrotor.settings.gyroBias[PITCH] = (int16_t)sum[1]/128;
	quadrotor.settings.gyroBias[YAW] = (int16_t)sum[2]/128;

#else
	debug("Using GYRO in DMP mode\r\n");
#endif

	// DMP configuration
	debug("Initializing MPU6050 DMP...\r\n");

	// GPIO0.4 as input with interrupt
	GPIO_SetDir(0,(1<<4),0);
	GPIO_IntCmd(0,(1<<4),1);
	GPIO_ClearInt(0,(1<<4));
	NVIC_SetPriority(EINT3_IRQn, 6);
	NVIC_EnableIRQ(EINT3_IRQn);

	// make sure it worked (returns 0 if so)
	if (MPU6050_dmpInitialize() == 0) {
		debug("MPU6050 DMP OK\r\n");
	} else {
		halt();
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
	}

	//=======================================================================

	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);

	debug("Hardware initialization complete!\r\n");
}
