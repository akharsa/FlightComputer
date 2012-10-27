/*
 * leds.h
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#ifndef _LED_H__
#define _LED_H__

#include "types.h"

typedef enum{
	ACTIVE_LOW,
	ACTIVE_HIGH
}qLed_polarity;

typedef struct {
	uint8_t			pinNum;
	uint8_t			portNum;
	qLed_polarity	polarity;

}qLed;

Status qLed_Init(qLed led);
Status qLed_DeInit(qLed led);
Status qLed_TurnOn(qLed led);
Status qLed_TurnOff(qLed led);

#endif
