/*
 * board.c
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#include "leds.h"
#include "qAnalog.h"
#include "board.h"

qLed leds[] = {	{0,22,ACTIVE_HIGH},
				{1,28,ACTIVE_HIGH},
				{1,29,ACTIVE_HIGH},
				{4,28,ACTIVE_HIGH},
				{4,29,ACTIVE_HIGH},
				{1,22,ACTIVE_HIGH},
				{1,25,ACTIVE_HIGH},
				};

qAnalogInput analog[] = {
		{30,1,3,ADC_CHANNEL_4},
		{31,1,3,ADC_CHANNEL_5},
};

qPWM_Channel pwm[] = {
		{0,2,1},//PWM1
		{1,2,2},//PWM2
		{2,2,3},//PWM3
		{3,2,4},//PWM4
		{4,2,5},//PWM5
		{5,2,5},//PWM6
};
