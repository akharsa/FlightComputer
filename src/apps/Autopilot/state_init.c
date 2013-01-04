
#include "FreeRTOS.h"
#include "task.h"

#include "qESC.h"
#include "qFSM.h"
#include "States.h"
#include "leds.h"
#include "DebugConsole.h"

#include "board.h"

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


	// --------------------------------------------------
	//	ESC Initialization
	// --------------------------------------------------
	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	/* Terminate and go to Idle */
	state_name_t newState=STATE_IDLE;
	qFSM_ChangeState(newState);

}

