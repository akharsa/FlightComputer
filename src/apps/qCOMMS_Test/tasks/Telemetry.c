/*
 * Telemetry.c
 *
 *  Created on: Feb 6, 2012
 *      Author: Alan
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "qUART.h"
#include "MPU6050.h"
#include "qI2C.h"
#include "DebugConsole.h"
#include "leds.h"
#include "board.h"
#include "timing.h"

timerHnd t1;
int rotation=1000;
int rotation_old=1000;
int period;

void Telemetry(void * pvParameters){
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

	int16_t sensors[9];
	int16_t temperature;

	qLed_Init(REAR_LEFT_LED);
	Status res;
	res = qI2C_Init();

	ConsolePuts_("Testing MPU6050...\r\n", BLUE);
	ConsolePuts("Connection...\t\t\t\t");

	qLed_Init(REAR_LEFT_LED);

	if (MPU6050_testConnection()==TRUE){
		ConsolePuts_("[OK]\r\n",GREEN);
		ConsolePuts("Initializing...\r\n");
		MPU6050_initialize();
		timerConfig(1,&t1);
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
		while(1);
	}

	timerStart(&t1);

	for (;;){

		//qLed_TurnOn(REAR_LEFT_LED);




		MPU6050_getMotion6(&sensors[0],&sensors[1],&sensors[2],&sensors[3],&sensors[4],&sensors[5]);

		rotation = sensors[3];

		if (rotation!=rotation_old){
			if ((rotation<10) && (rotation>-10)){
				qLed_TurnOn(REAR_LEFT_LED);

				timerStop(&t1);
				period = timerGetElapsed(&t1);
				ConsolePutNumber_(period,10,BLACK);
				ConsolePuts("\r\n");
				timerStart(&t1);

				//vTaskDelay(100/portTICK_RATE_MS);
				qLed_TurnOff(REAR_LEFT_LED);
			}
		}
		rotation_old = rotation;


		/*
		printf("Total time: %i\n",timerGetElapsed(&t1));


		MPU6050_getMotion6(&sensors[0],&sensors[1],&sensors[2],&sensors[3],&sensors[4],&sensors[5]);

		ConsolePutNumber(sensors[5],10);
		ConsolePuts("\r\n");

		qLed_TurnOff(REAR_LEFT_LED);

        vTaskDelayUntil( &xLastWakeTime, 100/portTICK_RATE_MS);*/

	}

}
