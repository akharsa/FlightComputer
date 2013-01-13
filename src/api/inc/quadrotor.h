/*
 * quadrotor.h
 *
 *  Created on: 11/01/2013
 *      Author: alan
 */

#ifndef QUADROTOR_H_
#define QUADROTOR_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "qCOMMS.h"
#include "board.h"

typedef struct {
	float omega[3];
	float setpoint[4];
	float CO[4];
	uint32_t time;
} SV_t;

typedef struct {
	float gyroBias[3];
} settings_t;

extern SV_t sv;
extern settings_t settings;


#endif /* QUADROTOR_H_ */
