/*
 * delay.c
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#include "delay.h"
#include "types.h"
#include "lpc17xx_systick.h"

static uint32_t delay_counter=0;

void delayInit(){
	SYSTICK_InternalInit(1);
	SYSTICK_IntCmd(ENABLE);
	SYSTICK_Cmd(DISABLE);
}

void delay(uint32_t ms){
	delay_counter = ms;
	SYSTICK_Cmd(ENABLE);
	while (delay_counter>0);
	SYSTICK_Cmd(DISABLE);
}

void _SysTick_Handler(void)
{
	if (delay_counter>0)
		delay_counter--;
}
