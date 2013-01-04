/*
 * qWDT.c
 *
 *  Created on: Apr 3, 2012
 *      Author: Alan
 */

#include "qWDT.h"
#include "lpc17xx_wdt.h"

qWDT_ResetSource qWDT_GetResetSource(){

	if (WDT_ReadTimeOutFlag()){

		WDT_ClrTimeOutFlag();
		return RESET_WDT;
	} else{
		return RESET_EXTERNAL;
	}
}

void qWDT_Start(uint32_t timeout){
	// Initialize WDT, IRC OSC, interrupt mode, timeout = 5000000us = 5s
	WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
	// Start watchdog with timeout given
	WDT_Start(timeout);
	//infinite loop to wait chip reset from WDT
}

void qWDT_Stop(){
	while(1);
	//WDT_UpdateTimeOut(0);
	//LPC_WDT->WDMOD &= !WDT_WDMOD_WDEN;
}


void qWDT_Feed(){
	WDT_Feed();
}
