/*
 * board.h
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "leds.h"
#include "qAnalog.h"
#include "qPWM.h"

#define EEPROM_ADDRESS				0xA0
#define MPU6050_ADDRESS				0xD0
#define HMC5883L_ADDRESS			0x3C
#define BMP085_ADDRESS				0xEE

#define	STATUS_LED					leds[0]
#define	REAR_RIGHT_LED				leds[1]
#define	REAR_LEFT_LED				leds[2]
#define	FRONT_RIGHT_LED				leds[3]
#define	FRONT_LEFT_LED				leds[4]
#define	EXTERNAL_1_LED				leds[5] //LED5 - P7
#define	EXTERNAL_2_LED				leds[6] //LED6 - P8
#define TOTAL_LEDS					7

#define TEMPERATURE_ANALOG			&analog[1]
#define VOLTAGE_ANALOG				&analog[0]

#define MOTOR1						&pwm[3]
#define MOTOR2						&pwm[1]
#define MOTOR3						&pwm[0]
#define MOTOR4						&pwm[4]

#define UART_GROUNDCOMM				0 //LPC_UART0

extern qAnalogInput analog[];
extern qLed leds[];
extern qPWM_Channel pwm[];

#endif /* BOARD_H_ */
