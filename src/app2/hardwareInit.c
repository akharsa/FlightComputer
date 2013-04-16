/*
 * hardwareInit.c
 *
 *  Created on: 01/02/2013
 *      Author: alan
 */


#include "qUART.h"
#include "qI2C.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"
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
#if 0
	debug("Searching for MPU6050 IMU...");
	if (MPU6050_testConnection()==TRUE){
		ConsolePuts_("[OK]\r\n",GREEN);
		MPU6050_initialize();
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}
#endif

	debug("Initializing MPU6050 IMU...");
	if (mpu_init(NULL)==0){
		ConsolePuts_("[OK]\r\n",GREEN);
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}

#ifdef RUN_SELF_TEST
	long int gyro[3],accel[3];
	int result;
	result = mpu_run_self_test(gyro,accel);
#endif

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(200);

    dmp_load_motion_driver_firmware();

    mpu_set_gyro_fsr(2000);
    mpu_set_accel_fsr(2);

    mpu_set_lpf(5);

    //dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    //dmp_register_tap_cb(tap_cb);
    //dmp_register_android_orient_cb(android_orient_cb);
    //hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
    //    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
    //    DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL);

    dmp_set_fifo_rate(200);

    dmp_enable_gyro_cal(1);

    mpu_set_dmp_state(1);

    // GPIO0.4 as input with interrupt
	GPIO_SetDir(0,(1<<4),0);
	GPIO_IntCmd(0,(1<<4),1);
	GPIO_ClearInt(0,(1<<4));
	NVIC_SetPriority(EINT3_IRQn, 6);
	NVIC_EnableIRQ(EINT3_IRQn);

#if 0
	debug("MPU6050 initialized\r\n");

	// DMP configuration
	debug("Initializing MPU6050 DMP...");

	// GPIO0.4 as input with interrupt
	GPIO_SetDir(0,(1<<4),0);
	GPIO_IntCmd(0,(1<<4),1);
	GPIO_ClearInt(0,(1<<4));
	NVIC_SetPriority(EINT3_IRQn, 6);
	NVIC_EnableIRQ(EINT3_IRQn);

	// make sure it worked (returns 0 if so)
	if (MPU6050_dmpInitialize() != 0) {
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
	}
	ConsolePuts_("[OK]\r\n",GREEN);
#endif
	//=======================================================================
	// =========================================================
	// Startup waiting for xbee

	for (j=0;j<100;j++){
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
	}


	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);

	debug("Hardware initialization complete!\r\n");
}
