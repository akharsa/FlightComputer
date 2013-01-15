
#include "FreeRTOS.h"
#include "task.h"

#include "qESC.h"
#include "qFSM.h"
#include "States.h"
#include "leds.h"
#include "DebugConsole.h"
#include "qUART.h"

#include "board.h"
#include "taskList.h"

#include "MPU6050.h"
#include "quadrotor.h"

/* ================================ */
/* Prototypes	 					*/
/* ================================ */
void Init_Task(void * pvParameters);
void Init_onEntry(void * pvParameters);
void Init_onExit(void * pvParameters);

/* ================================ */
/* Private globals 					*/
/* ================================ */
static xTaskHandle hnd;

void Init_onEntry(void * p){
	xTaskCreate(Init_Task, ( signed char * ) "INIT", 200, ( void * ) NULL, INIT_PRIORITY, &hnd );
}

void Init_onExit(void * p){
	vTaskDelete(hnd);
}

void Init_Task(void * pvParameters){
	uint8_t i,j;
	int16_t buffer[3];
	int32_t sum[3];

	// --------------------------------------------------
	//	Leds Initialization
	// --------------------------------------------------
	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOff(leds[i]);
	}

	for (j=0;j<3;j++){
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		//vTaskDelay(1000/portTICK_RATE_MS);
	}

	vTaskDelay(10000/portTICK_RATE_MS);

	// --------------------------------------------------

	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)==RET_ERROR){
		while(1);
	}

	ConsolePuts_("===================================\r\n",BLUE);
	ConsolePuts("\x1B[2J\x1B[0;0f");
	ConsolePuts_("FLC V2.0 Initialized...\r\n",BLUE);

	// --------------------------------------------------
	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
	ConsolePuts_("Calibrating sensors...\t\t\t\t",BLUE);

	if (qI2C_Init()!=SUCCESS) halt("I2C INIT ERROR");

	if (MPU6050_testConnection()==TRUE){
		MPU6050_initialize();
	}else{
		halt("MPU6050 Init ERROR\r\n");
	}

	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;

	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

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

	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
	ConsolePuts_("[OK]\r\n",GREEN);
	// --------------------------------------------------

	xTaskCreate( Communications, ( signed char * ) "COMMS", 500, ( void * ) NULL, COMMS_PRIORITY, NULL);

	/* Terminate and go to Idle */
	state_name_t newState=STATE_IDLE;
	qFSM_ChangeState(newState);

}

