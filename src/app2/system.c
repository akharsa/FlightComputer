#include "types.h"
#include "FreeRTOS.h"
#include "task.h"

#include "taskList.h"

#include "board.h"
#include "trcUser.h"
#include "quadrotor.h"
#include "DebugConsole.h"
#include "telemetry.h"
#include "configuration.h"


static xTaskHandle tlm_hnd;
static xTaskHandle comms_hnd;


void system(void * pvParameters){

	hardwareInit();
	//xTaskCreate( Communications, ( signed char * ) "COMMS", 500, ( void * ) NULL, COMMS_PRIORITY, &comms_hnd);
	//xTaskCreate( Telemetry, ( signed char * ) "TLM", 300, ( void * ) TLM_PERIOD, TLM_PRIORITY, &tlm_hnd);
	debug("Systim initialization complete");

	for(;;){
		vTaskDelay(10/portTICK_RATE_MS);
	}
}


void AppMain(void) {

	uiTraceStart();
	trcLabel = xTraceOpenLabel("FLCv2p0");
	vTracePrintF(trcLabel,"FLCv2p0 Startup");

	xTaskCreate( system, ( signed char * ) "AUTOPILOT", 1000, ( void * ) NULL, AUTOPILOT_PRIORITY, NULL);
	vTaskStartScheduler();
	for(;;);
}



void vApplicationStackOverflowHook( xTaskHandle xTask, signed portCHAR *pcTaskName ){
	while(1);
}

void EINT3_IRQHandler(void)
{
//	static signed portBASE_TYPE xHigherPriorityTaskWoken;
//    xHigherPriorityTaskWoken = pdFALSE;

	if(GPIO_GetIntStatus(0, 4, 1))
	{
		GPIO_ClearInt(0,(1<<4));
#if 0
		if (mpuSempahore!=NULL){
			xSemaphoreGiveFromISR(mpuSempahore,&xHigherPriorityTaskWoken);
		}
#endif
	}
	//portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



