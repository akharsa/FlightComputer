#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif


#include "types.h"
#include "DebugConsole.h"
#include "qI2C.h"
#include "eeprom.h"
#include "board.h"
#include "delay.h"
#include "qAnalog.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "bmp085.h"
#include "math.h"
#include "qESC.h"

#include "lpc17xx_uart.h"

int16_t speed = 0;
char wdt=0;

void halt(){
	ConsolePuts_("EXECUTION HALTED DUE TO AN ERROR\r\n",RED);
	for(;;);
}



int main(void) {
	uint16_t board_temp,voltage, mpu_temp;
	int32_t bmp_temp, pressure, altitude;
	int16_t sensors[6];
	uint8_t i;

	//---------------------------------------------------------------
	// Inits
	//---------------------------------------------------------------


	delayInit();
	ConsoleInit();

	UART_IntConfig(LPC_UART0, UART_INTCFG_RBR, ENABLE);
	NVIC_EnableIRQ(UART0_IRQn);

	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOff(leds[i]);
	}

	for (i=0;i<20;i++){
		qLed_TurnOn(FRONT_LEFT_LED);
		qLed_TurnOn(FRONT_RIGHT_LED);
		qLed_TurnOn(REAR_LEFT_LED);
		qLed_TurnOn(REAR_RIGHT_LED);
		delay(100);
		qLed_TurnOff(FRONT_LEFT_LED);
		qLed_TurnOff(FRONT_RIGHT_LED);
		qLed_TurnOff(REAR_LEFT_LED);
		qLed_TurnOff(REAR_RIGHT_LED);
		delay(400);
	}

	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	for(;;){
		ConsolePuts("Speed: ");
		ConsolePutNumber(speed,10);
		ConsolePuts("               \r");
		delay(100);
	}


	return 0;
}

void UART0_IRQHandler(void){
	char c;

	c = UART_ReceiveByte(LPC_UART0);
	wdt = c;
	switch(c){
		case 'Q':
		case 'q':
			if (speed<=(1000-10))
				speed += 10;
			break;

		case 'A':
		case 'a':
			if (speed>=10)
				speed -= 10;
			break;
		default:
			speed = 0;
			break;
	}

	qESC_SetOutput(MOTOR1,speed);
	//qESC_SetOutput(MOTOR2,speed);
	//qESC_SetOutput(MOTOR3,speed);
	//qESC_SetOutput(MOTOR4,speed);


}

