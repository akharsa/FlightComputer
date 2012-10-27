/*
 * board.h
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "leds.h"

#define	STATUS_LED					leds[0]
#define	REAR_RIGHT_LED				leds[1]
#define	REAR_LEFT_LED				leds[2]
#define	FRONT_RIGHT_LED				leds[3]
#define	FRONT_LEFT_LED				leds[4]
#define	EXTERNAL_1_LED				leds[5] //LED5 - P7
#define	EXTERNAL_2_LED				leds[6] //LED6 - P8

#define TOTAL_LEDS					7

extern qLed leds[];

#endif /* BOARD_H_ */
