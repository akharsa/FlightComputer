#include "telemetry.h"

#include "FreeRTOS.h"
#include "task.h"

#include "quadrotor.h"
#include "board.h"

#include  "qCOMMS.h"

void Telemetry(void * pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		quadrotor.sv.time = xTaskGetTickCount();
		//TODO: Mutex here!
		qComms_SendMsg(UART_GROUNDCOMM,0xBB,MSG_TYPE_TELEMETRY,sizeof(quadrotor.sv),(uint8_t*)&quadrotor.sv);
		vTaskDelayUntil( &xLastWakeTime, ((portTickType) pvParameters) /portTICK_RATE_MS );
	}

}
