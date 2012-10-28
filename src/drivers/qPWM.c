/*
 * pwm.c
 *
 *  Created on: 28/10/2012
 *      Author: alan
 */

#include "qPWM.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_pinsel.h"
#include "types.h"


Status qPWM_Init(uint32_t uS_period){

	PWM_TIMERCFG_Type PWMCfgDat;
	PWM_MATCHCFG_Type PWMMatchCfgDat;
	uint8_t temp;

	/* PWM block section -------------------------------------------- */
	/* Initialize PWM peripheral, timer mode
	 * PWM prescale value = 1 (absolute value - tick value) */
	PWMCfgDat.PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
	PWMCfgDat.PrescaleValue = 1;
	PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat);

	/* Set match value for PWM match channel 0 = 256, update immediately */
	PWM_MatchUpdate(LPC_PWM1, 0, uS_period, PWM_MATCH_UPDATE_NOW);
	/* PWM Timer/Counter will be reset when channel 0 matching
	 * no interrupt when match
	 * no stop when match */
	PWMMatchCfgDat.IntOnMatch = DISABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

	/* Configure PWM channel edge option
	 * Note: PWM Channel 1 is in single mode as default state and
	 * can not be changed to double edge mode */
	for (temp = 2; temp < 7; temp++)
	{
		PWM_ChannelConfig(LPC_PWM1, temp, PWM_CHANNEL_SINGLE_EDGE);
	}

	/* Reset and Start counter */
	PWM_ResetCounter(LPC_PWM1);
	PWM_CounterCmd(LPC_PWM1, ENABLE);

	/* Start PWM now */
	PWM_Cmd(LPC_PWM1, ENABLE);
	return SUCCESS;
}

Status qPWM_InitChannel(qPWM_Channel * q){
	PINSEL_CFG_Type PinCfg;
	PWM_MATCHCFG_Type PWMMatchCfgDat;

	/*
	 * Initialize PWM pin connect
	 */
	PinCfg.Funcnum = 1; //son todos 1
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = q->portNum;
	PinCfg.Pinnum = q->pinNum;
	PINSEL_ConfigPin(&PinCfg);

	/* Configure match option */
	PWMMatchCfgDat.IntOnMatch = DISABLE;
	PWMMatchCfgDat.MatchChannel = q->channel;
	PWMMatchCfgDat.ResetOnMatch = DISABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

	PWM_MatchUpdate(LPC_PWM1, q->channel, 0, PWM_MATCH_UPDATE_NEXT_RST);

	/* Enable PWM Channel Output */
	PWM_ChannelCmd(LPC_PWM1, q->channel, ENABLE);
	/* Increase match value by 10 */
	return SUCCESS;
}

Status qPWM_SetDuty(qPWM_Channel * q, uint32_t uS_duty){
	/* Set up match value */
	PWM_MatchUpdate(LPC_PWM1, q->channel, uS_duty, PWM_MATCH_UPDATE_NEXT_RST);
	return SUCCESS;
}

