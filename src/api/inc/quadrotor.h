/*
 * quadrotor.h
 *
 *  Created on: 11/01/2013
 *      Author: alan
 */

#ifndef QUADROTOR_H_
#define QUADROTOR_H_

#include "joystick.h"
#include "qPIDs.h"
#include "nvram.h"

#define ROLL		0
#define PITCH		1
#define YAW			2
#define ALTITUDE	3

#define K_Z		700
#define K_PHI	200
#define K_THETA	200
#define K_PSI	300

typedef struct {
	float rate[3];
	float attitude[3];
	float setpoint[4];
	float rateCtrlOutput[3];
	float attiCtrlOutput[3];
	uint16_t motorOutput[4];
	uint32_t time;
} SV_t;

typedef struct {
	float gyroBias[3];
} settings_t;

typedef struct {
	SV_t sv;
	settings_t settings;
	joystick_t joystick;
	qPID rateController[3];
	qPID attiController[3];
	qPID altitudeController;
}quadrotor_t;

extern quadrotor_t quadrotor;
extern nvram_t nvramBuffer;

#endif /* QUADROTOR_H_ */
