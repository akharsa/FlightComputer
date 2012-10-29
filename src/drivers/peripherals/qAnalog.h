/*
 * qAnalog.h
 *
 *  Created on: 28/10/2012
 *      Author: alan
 */

#ifndef QANALOG_H_
#define QANALOG_H_

#include "lpc17xx_adc.h"

typedef struct {
	uint8_t					pinNum;
	uint8_t					portNum;
	uint8_t					funcNum;
	ADC_CHANNEL_SELECTION	adcChannel;
}qAnalogInput;

Status qAnalog_Init();
Status qAnalog_InitPin(qAnalogInput * q);
uint16_t qAnalog_Read(qAnalogInput * q);


#endif /* QANALOG_H_ */
