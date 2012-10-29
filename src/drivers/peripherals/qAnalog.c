/*
 * qAnalog.c
 *
 *  Created on: 28/10/2012
 *      Author: alan
 */

#include "qAnalog.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"

Status qAnalog_Init(){
	ADC_Init(LPC_ADC, 200000);
	return SUCCESS;
}


Status qAnalog_InitPin(qAnalogInput * q){
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = q->funcNum;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum = q->pinNum;
	PinCfg.Portnum = q->portNum;
	PINSEL_ConfigPin(&PinCfg);

	ADC_IntConfig(LPC_ADC,q->adcChannel,DISABLE);

	return SUCCESS;
}
// FIXME: de qAnalogInit is not really necessary, but without it consecutive readings are faulty (the second one is corrupted)
//        Maybe a BURST mode is preferible in this case but only 2 analogs are beign used right now.
uint16_t qAnalog_Read(qAnalogInput * q){

	uint16_t ret;
	qAnalog_Init();
	ADC_ChannelCmd(LPC_ADC,q->adcChannel,ENABLE);

	ADC_StartCmd(LPC_ADC,ADC_START_NOW);

	//Wait conversion complete
	while (!(ADC_ChannelGetStatus(LPC_ADC,q->adcChannel,ADC_DATA_DONE)));
	ret = ADC_ChannelGetData(LPC_ADC,q->adcChannel);

	ADC_ChannelCmd(LPC_ADC,q->adcChannel,DISABLE);

	return ret;

}
