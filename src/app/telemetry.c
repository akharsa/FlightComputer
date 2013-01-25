

#include "quadrotor.h"
#include "telemetry.h"
#include <stdint.h>

static xTaskHandle tlm_hnd;
static uint8_t taskRunning = 0;

void StartTelemetry(portTickType interval){
	taskRunning = 1;
	xTaskCreate( Telemetry, ( signed char * ) "TLM", 300, ( void * ) interval, TLM_PRIORITY, &tlm_hnd);
}

void StopTelemetry(void){
	if (taskRunning==1){
		vTaskDelete(tlm_hnd);
		taskRunning = 0;
	}
}

void Telemetry(void * pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();



	for (;;)
	{
		sv.time = xTaskGetTickCount();
		//TODO: Mutex here!
		qComms_SendMsg(UART_GROUNDCOMM,0xBB,MSG_TYPE_TELEMETRY,sizeof(sv),(uint8_t*)&sv);
		vTaskDelayUntil( &xLastWakeTime, ((portTickType) pvParameters) /portTICK_RATE_MS );
	}

}
