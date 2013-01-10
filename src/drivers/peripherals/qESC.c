/*
 * qESC.c
 *
 *  Created on: 29/10/2012
 *      Author: alan
 */

#include "types.h"
#include "qESC.h"
#include "qPWM.h"

Status qESC_Init(){
	qPWM_Init(ESC_FREC_RATE);
	return SUCCESS;
}

Status qESC_InitChannel(qPWM_Channel * q){
	int i;
	qPWM_InitChannel(q);
	//delay(10);
	for (i=0;i<100000;i++);
	qPWM_SetDuty(q,ESC_MIN_VALUE);
	return SUCCESS;
}

Status qESC_SetOutput(qPWM_Channel * q, uint16_t duty){

	if (duty<=INPUT_MAX){
		uint32_t buffer = (duty*(ESC_MAX_VALUE-ESC_MIN_VALUE));
		buffer = buffer / INPUT_MAX;
		qPWM_SetDuty(q,ESC_MIN_VALUE+buffer);
		return SUCCESS;
	}else if (duty<0){
		qPWM_SetDuty(q,ESC_MIN_VALUE);
	}else{
		return ERROR;
	}
}
