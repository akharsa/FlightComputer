/*
===============================================================================
 Name        : main.c
 Author      : Alan Kharsansky <akharsa@gmail.com>
 Version     : 1.0
 Copyright   : Copyright (C) 
 Description : Blinky led using the SysTick and GPIO drivers
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_gpio.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

#include <cr_section_macros.h>
#include <NXP/crp.h>
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

FunctionalState Cur_State = ENABLE;
const char welcome[]={"LPC1768 encendido...\r\n"};

int main(void) {
	
	PINSEL_CFG_Type PinCfg;
	UART_CFG_Type UARTConfigStruct;
	char c;

	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.Pinnum = PINSEL_PIN_2;
	PinCfg.Portnum = PINSEL_PORT_0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = PINSEL_PIN_3;
	PINSEL_ConfigPin(&PinCfg);

	// Configuramos la UART
	UARTConfigStruct.Baud_rate = 9600;
	UARTConfigStruct.Databits = UART_DATABIT_8;
	UARTConfigStruct.Parity = UART_PARITY_NONE;
	UARTConfigStruct.Stopbits = UART_STOPBIT_1;

	// Inicializamos la UART
	UART_Init(LPC_UART0, &UARTConfigStruct);
	UART_TxCmd(LPC_UART0, ENABLE);

	//Use P0.22 to test System Tick interrupt
	GPIO_SetDir(1, (1<<28), 1);
	GPIO_SetDir(1, (1<<29), 1);
	GPIO_SetDir(4, (1<<28), 1);
	GPIO_SetDir(4, (1<<29), 1);


	UART_Send(LPC_UART0, welcome, strlen(welcome), BLOCKING);

	//Initialize System Tick with 10ms time interval
	//WARNING: Check the param allowed range!!!
	SYSTICK_InternalInit(100);
	//Enable System Tick interrupt
	SYSTICK_IntCmd(ENABLE);
	//Enable System Tick Counter
	SYSTICK_Cmd(ENABLE);



	// Endless loop
	while(1);

	return 1;
}


void SysTick_Handler(void)
{
	static led=0;

	//Clear System Tick counter flag
	SYSTICK_ClearCounterFlag();

	UART_SendByte(LPC_UART0,'a');

	//toggle P0.22
	if (Cur_State == ENABLE)
	{
		// Apagar pin
		GPIO_ClearValue(1 + 3 * (led & 1), (1<<(28 + (led >> 1))));
		Cur_State = DISABLE;
		led += 1;
		led %= 4;
	}
	else
	{
		// Prender pinc
		GPIO_SetValue(1 + 3 * (led & 1), (1<<(28 + (led >> 1))));
		Cur_State = ENABLE;
	}
}
