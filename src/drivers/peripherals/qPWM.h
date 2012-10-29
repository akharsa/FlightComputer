/*
 * pwm.h
 *
 *  Created on: 28/10/2012
 *      Author: alan
 */

#ifndef qPWM_H_
#define qPWM_H_

#include "lpc17xx_pwm.h"


typedef struct {
	uint8_t			pinNum;
	uint8_t			portNum;
	uint8_t 		channel;
}qPWM_Channel;

Status qPWM_Init(uint32_t uS_Period);
Status qPWM_InitChannel(qPWM_Channel * q);
Status qPWM_SetDuty(qPWM_Channel * q, uint32_t uS_duty);

#endif /* PWM_H_ */
