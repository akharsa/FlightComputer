/*
 * delay.h
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "types.h"

void delayInit();
void delay(uint32_t ms);
void SysTick_Handler(void);

#endif /* DELAY_H_ */
