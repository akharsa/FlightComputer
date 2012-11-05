/*
 * leds.c
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#include "leds.h"
#include "lpc17xx_gpio.h"
#include "types.h"

Status qLed_Init(qLed led){
	GPIO_SetDir(led.pinNum,(1<<led.portNum),1);
	return SUCCESS;
}

Status qLed_DeInit(qLed led){
	GPIO_SetDir(led.pinNum,(1<<led.portNum),0);
	return SUCCESS;
}

Status qLed_TurnOn(qLed led){
	if (led.polarity==ACTIVE_HIGH){
		//FIXME: no estan al reves pin y port?
		GPIO_SetValue(led.pinNum,(1<<led.portNum));
	}else{
		GPIO_ClearValue(led.pinNum,(1<<led.portNum));
	}
	return SUCCESS;
}

Status qLed_TurnOff(qLed led){
	if (led.polarity==ACTIVE_LOW){
		GPIO_SetValue(led.pinNum,(1<<led.portNum));
	}else{
		GPIO_ClearValue(led.pinNum,(1<<led.portNum));
	}
	return SUCCESS;
}
