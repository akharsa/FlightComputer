/*
 * qESC.h
 *
 *  Created on: 29/10/2012
 *      Author: alan
 */

#ifndef QESC_H_
#define QESC_H_

#include "types.h"
#include "qPWM.h"

#define ESC_FREC_RATE	2500 // 1/400Hz
#define ESC_MIN_VALUE	1000
#define ESC_MAX_VALUE	2000
#define INPUT_MAX		1000


Status qESC_Init();
Status qESC_InitChannel(qPWM_Channel * q);
Status qESC_SetOutput(qPWM_Channel * q, uint16_t duty);

#endif /* QESC_H_ */
