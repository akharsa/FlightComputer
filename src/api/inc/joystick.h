/*
 * joystick.h
 *
 *  Created on: 13/01/2013
 *      Author: alan
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>

#define	BTN_SELECT	 	0x0001
#define	BTN_L3			0x0002
#define	BTN_R3			0x0004
#define	BTN_START		0x0008
#define	BTN_UP			0x0010
#define	BTN_RIGHT		0x0020
#define	BTN_DOWN		0x0040
#define	BTN_LEFT		0x0080
#define	BTN_LEFT2		0x0100
#define	BTN_RIGHT2		0x0200
#define	BTN_LEFT1		0x0400
#define	BTN_RIGHT1		0x0800
#define	BTN_TRIANGLE	0x1000
#define	BTN_CIRCLE		0x2000
#define	BTN_CROSS		0x4000
#define	BTN_SQUARE		0x8000

typedef struct{
	uint8_t x;
	uint8_t y;
}joystick_pad_t;

typedef struct{
	joystick_pad_t		left_pad;
	joystick_pad_t		right_pad;
	uint8_t 	L1;
	uint8_t 	L2;
	uint8_t 	R1;
	uint8_t 	R2;
	uint16_t 	buttons;
}joystick_t;


#endif /* JOYSTICK_H_ */
