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

#define MIN_INPUT 		100
#define MAX_INPUT 		1000
#define WAIT_TIME 		1000


#define STEP			5
#define SAMPLE_TIME		10


int16_t speed = 0;
uint16_t input;
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
	//uint16_t board_temp,voltage, mpu_temp;
	//int32_t bmp_temp, pressure, altitude;
	//int16_t sensors[6];
	uint8_t i;

	//---------------------------------------------------------------
	// Inits
	//---------------------------------------------------------------


	delayInit();
	ConsoleInit();

	ConsolePuts("=================================================================\r\n");

	UART_IntConfig(LPC_UART0, UART_INTCFG_RBR, ENABLE);
	NVIC_EnableIRQ(UART0_IRQn);

	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOff(leds[i]);
	}

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


//	while(1);

	GPIO_SetDir(0,(1<<24),1);
	GPIO_ClearValue(0,(1<<24));
	int p;

	delay(1000);

#if 0
	GPIO_SetDir(0,(1<<24),1);
	while(1){
		GPIO_SetValue(0,(1<<24));
		delay(500);
		GPIO_ClearValue(0,(1<<24));
		delay(500);
	}
#endif
/*
	int j;
	GPIO_SetValue(0,(1<<24));

	for (j=200;j<=500;j=j+100){
		GPIO_SetValue(0,(1<<24));
		qESC_SetOutput(MOTOR3,j);
		delay(5000);
		qESC_SetOutput(MOTOR3,0);
		GPIO_ClearValue(0,(1<<24));
		delay(5000);
	}

*/
	//SYSTICK_InternalInit(1);
	//SYSTICK_IntCmd(ENABLE);
	//SYSTICK_Cmd(ENABLE);

/*
	for (j=0;j<(MAX_INPUT-MIN_INPUT)*(WAIT_TIME/SAMPLE_TIME);j++){
		delay(SAMPLE_TIME);
	}
*/
//	while(1);



	for(;;){
		//ConsolePuts("Speed: ");
		ConsolePutNumber(speed,10);
		ConsolePuts("\t");
		//ConsolePuts("Capture: ");
		ConsolePutNumber(speed_capture,10);
		ConsolePuts("\r\n");
		delay(10);
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
			speed = 600;
			break;

		case 'A':
		case 'a':
			speed = 550;
			break;

		case 'Z':
		case 'z':
			speed = 500;
			break;

	/*
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
	*/
		default:
			speed = 0;
			break;
	}

	qESC_SetOutput(MOTOR4,speed);
	//qESC_SetOutput(MOTOR2,speed);
	//qESC_SetOutput(MOTOR3,speed);
	//qESC_SetOutput(MOTOR4,speed);


}


uint32_t j=0;
uint32_t time = 0;
typedef enum{IDLE, SETTLING, LOGGING} state_t;
state_t state = IDLE;
#if 0
void SysTick_Handler(void)
{
	if ((time%WAIT_TIME) == 0){ // 3 seconds interval
		switch (state){
			case IDLE:
				input = MIN_INPUT;
				state = SETTLING;
				break;
			case SETTLING:
				input += STEP;
				qESC_SetOutput(MOTOR4,input);
				GPIO_ClearValue(0,(1<<24));
				state = LOGGING;
				break;
			case LOGGING:
				GPIO_SetValue(0,(1<<24));
				state = SETTLING;
				break;

		}
	}

	if ((time%SAMPLE_TIME) == 0){ // 3 seconds interval
		switch (state){
			case SETTLING:
				//ConsolePuts("LOG:\t");
				ConsolePutNumber(time,10);
				ConsolePuts("\t");
				ConsolePutNumber(input,10);
				ConsolePuts("\t");
				ConsolePutNumber(speed_capture,10);
				ConsolePuts("\r\n");
				break;
		}
	}

	time = time + 1;

	if (input == (MAX_INPUT + STEP)){
		SYSTICK_Cmd(DISABLE);
		qESC_SetOutput(MOTOR4,0);
	}

	/*
	GPIO_SetValue(0,(1<<24));
	ConsolePutNumber(j,10);
	ConsolePuts("\t");
	ConsolePutNumber(speed_capture,10);
	ConsolePuts("\t");
	ConsolePutNumber(input,10);
	ConsolePuts("\r\n");

	if (j%(WAIT_TIME/SAMPLE_TIME) == 0){
		qESC_SetOutput(MOTOR4,input);
		input++;
	}

	j++;
	if (j==(MAX_INPUT-MIN_INPUT)*(WAIT_TIME/SAMPLE_TIME)){
		SYSTICK_Cmd(DISABLE);
		qESC_SetOutput(MOTOR4,0);
		GPIO_ClearValue(0,(1<<24));
	}
	*/
}
#endif

