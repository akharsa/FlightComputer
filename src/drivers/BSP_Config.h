/*
 * BSP_Config.h
 *
 *  Created on: Feb 1, 2012
 *      Author: Alan
 */

#ifndef BSP_CONFIG_H_
#define BSP_CONFIG_H_

#define qUART_TOTAL			3
#define qUART_0				LPC_UART0
#define qUART_1				LPC_UART2
#define qUART_2				LPC_UART3

#define qPWM_TOTAL			1
#define qPWM_1				LPC_PWM1
#define qPWM_CHANNELS		6

#define qGPIO_TOTAL			4

#define qGPIO_PIN0_PIN		9
#define qGPIO_PIN0_PORT		0

#define qGPIO_PIN1_PIN		8
#define qGPIO_PIN1_PORT		0

#define qGPIO_PIN2_PIN		7
#define qGPIO_PIN2_PORT		0

#define qGPIO_PIN3_PIN		6
#define qGPIO_PIN3_PORT		0

#endif/* BSP_CONFIG_H_ */
