
#include "FreeRTOS.h"
#include "task.h"

#include "qESC.h"
#include "qFSM.h"
#include "States.h"
#include "leds.h"
#include "DebugConsole.h"

#include "board.h"
#include "taskList.h"

#include "MPU6050.h"
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
	xTaskCreate(Init_Task, ( signed char * ) "INIT", 200, ( void * ) NULL, 1, &hnd );
}

void Init_onExit(void * p){
	vTaskDelete(hnd);
}

void Init_Task(void * pvParameters){
	uint8_t i,j;

	// --------------------------------------------------
	//	Leds Initialization
	// --------------------------------------------------
	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOff(leds[i]);
	}

	for (j=0;j<3;j++){
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(100/portTICK_RATE_MS);
		for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(100/portTICK_RATE_MS);
	}

	vTaskDelay(10000/portTICK_RATE_MS);

	ConsolePuts_("===================================\r\n",BLUE);
	ConsolePuts("\x1B[2J\x1B[0;0f");
	ConsolePuts("FLC V2.0 Initialized...\r\n");
	ConsolePuts("Initializing I2C driver...\t\t\t\t");


	if (qI2C_Init()==SUCCESS){
		ConsolePuts_("[OK]\r\n",GREEN);
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
	}


	if (MPU6050_testConnection()==TRUE){
		ConsolePuts_("MPU6050 Init OK\r\n",GREEN);
		MPU6050_initialize();
	}else{
		ConsolePuts_("MPU6050 Init ERROR\r\n",RED);
	}

	xTaskCreate( Communications, ( signed char * ) "COMMS", 500, ( void * ) NULL, 3, NULL);

	/* Terminate and go to Idle */
	state_name_t newState=STATE_IDLE;
	qFSM_ChangeState(newState);

}

