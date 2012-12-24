/*
 * qFSM.h
 *
 *  Created on: Mar 3, 2012
 *      Author: Alan
 */

#ifndef QFSM_H_
#define QFSM_H_

#include <stdint.h>

#define MAX_STATES 20

#define TransitionValid(from,to,table) table[from-1][to-1]

typedef enum{
	NO=0,
	YES
}transition_t;

typedef struct{
	char stateName[20];
	void (*onEntry)(void *);
	void (*onExit)(void *);
} State_t;

State_t sysStates[MAX_STATES];
uint8_t systemState;

void qFSM_registerState(uint8_t state_ID, const char * name, void (*onEntry)(void *), void (*onExit)(void *));
void qFSM_ChangeState(uint8_t state_ID);

#endif /* QFSM_H_ */
