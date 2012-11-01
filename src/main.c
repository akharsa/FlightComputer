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
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"

int16_t speed = 0;
char wdt=0;

void halt(){
	ConsolePuts_("EXECUTION HALTED DUE TO AN ERROR\r\n",RED);
	for(;;);
}

TIM_TIMERCFG_Type TIM_ConfigStruct;
TIM_CAPTURECFG_Type TIM_CaptureConfigStruct;

uint32_t capture;
uint32_t speed_capture;
uint8_t avg = 0;

void TIMER3_IRQHandler(void)
{
	if (TIM_GetIntCaptureStatus(LPC_TIM3,0))
	{
		TIM_ClearIntCapturePending(LPC_TIM3,0);
		TIM_Cmd(LPC_TIM3,DISABLE);
		TIM_ResetCounter(LPC_TIM3);
		if (++avg == 5){
			speed_capture = capture / 5;
			capture = TIM_GetCaptureValue(LPC_TIM3,0);
			avg = 0;
		}else{
			capture = capture + TIM_GetCaptureValue(LPC_TIM3,0);
		}
		TIM_Cmd(LPC_TIM3,ENABLE);

	}
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
#if 0
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
#endif

	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	//---------------------------------------------------------

	PINSEL_CFG_Type PinCfg;

	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_CAPTURECFG_Type TIM_CaptureConfigStruct;

	//CAP3[0], P0[23]

	PinCfg.Funcnum = 3;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 23;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize timer 0, prescale count time of 1uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;

	// use channel 0, CAPn.0
	TIM_CaptureConfigStruct.CaptureChannel = 0;
	// Enable capture on CAPn.0 rising edge
	TIM_CaptureConfigStruct.RisingEdge = ENABLE;
	// Enable capture on CAPn.0 falling edge
	TIM_CaptureConfigStruct.FallingEdge = DISABLE;
	// Generate capture interrupt
	TIM_CaptureConfigStruct.IntOnCaption = ENABLE;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM3, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigCapture(LPC_TIM3, &TIM_CaptureConfigStruct);
	TIM_ResetCounter(LPC_TIM3);

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER3_IRQn, ((0x01<<3)|0x01));

	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER3_IRQn);

	// To start timer 0
	TIM_Cmd(LPC_TIM3,ENABLE);


	//---------------------------------------------------------

	for(;;){
		ConsolePuts("Speed: ");
		ConsolePutNumber(speed,10);
		ConsolePuts("               ");
		ConsolePuts("Capture: ");
		ConsolePutNumber(speed_capture,10);
		ConsolePuts("              \r\n");
		delay(500);
	}


	return 0;
}
#define STEP	1

void UART0_IRQHandler(void){
	char c;

	c = UART_ReceiveByte(LPC_UART0);
	wdt = c;
	switch(c){
		case 'Q':
		case 'q':
			if (speed<=(1000-STEP))
				speed += STEP;
			break;

		case 'A':
		case 'a':
			if (speed>=STEP)
				speed -= STEP;
			break;
		default:
			speed = 0;
			break;
	}

	qESC_SetOutput(MOTOR4,speed);
	//qESC_SetOutput(MOTOR2,speed);
	//qESC_SetOutput(MOTOR3,speed);
	//qESC_SetOutput(MOTOR4,speed);


}

