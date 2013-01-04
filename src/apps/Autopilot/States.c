/*
 * States.c
 *
 *  Created on: Apr 3, 2012
 *      Author: Alan
 */

#include "States.h"

const transition_t	/* 		|  STATE_INIT  | 	STATE_IDLE  |	STATE_FLYNG		| */
transitionTable[3][3]={
/*	STATE_INIT			*/{		NO,					YES,				NO,			}	,
/*	STATE_IDLE			*/{		NO,					NO,					YES,		}	,
/*	STATE_FLYNG			*/{		NO,					YES,				NO,			}};

