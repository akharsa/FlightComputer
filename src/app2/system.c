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
#include "trcUser.h"

static xTaskHandle tlm_hnd;
static xTaskHandle comms_hnd;
xTaskHandle BeaconHnd;

#define FLIGHT_STATE	1
#define IDLE_STATE		2

uint8_t state = IDLE_STATE;
uint8_t lastState = IDLE_STATE;

void system(void * pvParameters){
	hardwareInit();
	xTaskCreate( Communications, ( signed char * ) "COMMS", 500, ( void * ) NULL, COMMS_PRIORITY, &comms_hnd);
	xTaskCreate( Telemetry, ( signed char * ) "TLM", 300, ( void * ) TLM_PERIOD, TLM_PRIORITY, &tlm_hnd);
	xTaskCreate( beacon, ( signed char * ) "BEACON", 100, ( void * ) NULL, BEACON_PRIORITY, &BeaconHnd );
	vTaskSuspend(BeaconHnd);
	Flight_onTimeStartup();
	debug("Systim initialization complete");
	Flight_Task();
}


void AppMain(void) {

	uiTraceStart();
	trcLabel = xTraceOpenLabel("System");

	debug("FLCv2p0 Startup");

	xTaskCreate( system, ( signed char * ) "AUTOPILOT", 1000, ( void * ) NULL, AUTOPILOT_PRIORITY, NULL);
	vTaskStartScheduler();
	for(;;);
}



void vApplicationStackOverflowHook( xTaskHandle xTask, signed portCHAR *pcTaskName ){
	while(1);
}

//traceLabel stack = xTraceOpenLabel("STACK");
	//vTraceUserEvent(stack);
	//vTracePrintF(stack, "STACK: %d",uxTaskGetStackHighWaterMark(NULL));
