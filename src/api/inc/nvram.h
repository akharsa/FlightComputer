/*
 * nvram.h
 *
 *  Created on: 24/03/2013
 *      Author: alan
 */

#ifndef NVRAM_H_
#define NVRAM_H_

#include "types.h"

typedef struct{
	float K, Ti, Td;	// For use in NON-INT or INT modes
	float Nd;			// For derivator
}pid_config_t;

typedef struct{
	uint8_t sha[20];
	pid_config_t rateController[3];
	pid_config_t attiController[3];
	pid_config_t altitudeController;
}nvram_t;

Status qNVRAM_Load(nvram_t * p);
Status qNVRAM_Save(nvram_t * p);
Status qNVRAM_setDefaults(nvram_t * p);

#endif /* NVRAM_H_ */
