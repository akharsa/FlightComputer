/*
 * quadrotor.h
 *
 *  Created on: 11/01/2013
 *      Author: alan
 */

#ifndef QUADROTOR_H_
#define QUADROTOR_H_

typedef struct {
	float omega[3];
	float setpoint[4];
	float CO[4];
	float angle[3];
	float omega_bias[3];
	//float vb[3];
	//float xb[3];

	//float temperature[3];
	//float height;
	//float batteryVoltage;
} SV_t;

typedef struct {
	float gyroBias[3];
} settings_t;

extern SV_t sv;
extern settings_t settings;


#endif /* QUADROTOR_H_ */
