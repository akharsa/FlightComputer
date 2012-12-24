/*
 * main2.c
 *
 *  Created on: 23/12/2012
 *      Author: alan
 */
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "leds.h"

void SystemController(void * p){
	int i;
	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOff(leds[i]);
	}

	for(;;){
		qLed_TurnOn(FRONT_LEFT_LED);
		qLed_TurnOn(FRONT_RIGHT_LED);
		qLed_TurnOn(REAR_LEFT_LED);
		qLed_TurnOn(REAR_RIGHT_LED);
		vTaskDelay(50/portTICK_RATE_MS);
		qLed_TurnOff(FRONT_LEFT_LED);
		qLed_TurnOff(FRONT_RIGHT_LED);
		qLed_TurnOff(REAR_LEFT_LED);
		qLed_TurnOff(REAR_RIGHT_LED);
		vTaskDelay(50/portTICK_RATE_MS);
		qLed_TurnOn(FRONT_LEFT_LED);
		qLed_TurnOn(FRONT_RIGHT_LED);
		qLed_TurnOn(REAR_LEFT_LED);
		qLed_TurnOn(REAR_RIGHT_LED);
		vTaskDelay(50/portTICK_RATE_MS);
		qLed_TurnOff(FRONT_LEFT_LED);
		qLed_TurnOff(FRONT_RIGHT_LED);
		qLed_TurnOff(REAR_LEFT_LED);
		qLed_TurnOff(REAR_RIGHT_LED);

		vTaskDelay(800/portTICK_RATE_MS);
	}
}


int main(void) {

	xTaskCreate(SystemController, ( signed char * ) "System Controller", 1000, ( void * ) NULL, 3, NULL );
	vTaskStartScheduler();

	return 0 ;
}


