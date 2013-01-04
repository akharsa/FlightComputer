/*
 * qWDT.h
 *
 *  Created on: Apr 3, 2012
 *      Author: Alan
 */

#ifndef QWDT_H_
#define QWDT_H_

#include <stdint.h>

typedef enum{
	RESET_EXTERNAL,
	RESET_WDT
}qWDT_ResetSource;

qWDT_ResetSource qWDT_GetResetSource();
void qWDT_Start(uint32_t timeout);
void qWDT_Feed();
void qWDT_Stop();

#endif /* QWDT_H_ */
