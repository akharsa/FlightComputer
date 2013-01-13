/*
 * States.h
 *
 *  Created on: Apr 3, 2012
 *      Author: Alan
 */

//========================================================
//	App specific
//========================================================

#ifndef STATES_H_
#define STATES_H_

#include "qFSM.h"

#define TOTAL_STATES	4

typedef enum{
	STATE_RESET=0,
	STATE_INIT,
	STATE_IDLE,
	STATE_FLIGHT,
} state_name_t;


extern const transition_t transitionTable[TOTAL_STATES-1][TOTAL_STATES-1]; //-1 because of the reset

void Init_onEntry(void *);
void Init_onExit(void *);
void Idle_onEntry(void *);
void Idle_onExit(void *);
void Flight_onEntry(void *);
void Flight_onExit(void *);

#endif /* STATES_H_ */
