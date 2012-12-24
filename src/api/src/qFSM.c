/*
 * qFSM.c
 *
 *  Created on: Mar 3, 2012
 *      Author: Alan
 */

#include "qFSM.h"
#include "string.h"
#include <stdint.h>

void qFSM_registerState(uint8_t state_ID, const char * name, void (*onEntry)(void *), void (*onExit)(void *)){
	sysStates[state_ID].onEntry = onEntry;
	sysStates[state_ID].onExit = onExit;
	strcpy(sysStates[state_ID].stateName,name);
}

