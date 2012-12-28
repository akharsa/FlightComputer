/*
 * timing.c
 *
 *  Created on: 15/10/2011
 *      Author: Alan
 */

#include "timing.h"
#include "lpc17xx_timer.h"

#define TIMER	LPC_TIM0

void timerConfig(uint32_t resolution, timerHnd * t){

	TIM_TIMERCFG_Type TIM_ConfigStruct;

	// Timer init
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= resolution;

	TIM_Init(TIMER, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_Cmd(TIMER,ENABLE);

	t->resolution = resolution;
	//t->ticksToSeconds = ;

}

void timerStart(timerHnd * t){
	t->startTime = TIMER->TC;
}

void timerStop(timerHnd * t){
	t->endTime = TIMER->TC;
}

uint32_t timerGetElapsed(timerHnd * t){
	return ((t->endTime - t->startTime));
}
