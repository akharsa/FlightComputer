/*
 * beacon.c
 *
 *  Created on: 02/02/2013
 *      Author: alan
 */

#include "FreeRTOS.h"
#include "task.h"
#include "leds.h"
#include "board.h"

void beacon(void * pvParameters){
	int i;

	for(;;){
		for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);
		for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
		vTaskDelay(50/portTICK_RATE_MS);

		vTaskDelay(800/portTICK_RATE_MS);
	}
}

void calibratingBeacon(void * pvParameters){
	int i;

	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOn(leds[i]);
	vTaskDelay(50/portTICK_RATE_MS);
	for (i=0;i<TOTAL_LEDS;i++) qLed_TurnOff(leds[i]);
	vTaskDelay(50/portTICK_RATE_MS);
}
