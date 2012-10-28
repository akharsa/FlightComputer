#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif


#include "types.h"
#include "DebugConsole.h"
#include "qI2C.h"
#include "eeprom.h"
#include "board.h"
#include "delay.h"

#include "lpc17xx_pwm.h"
#include "lpc17xx_pinsel.h"

#include "qAnalog.h"


void halt(){
	ConsolePuts_("EXECUTION HALTED DUE TO AN ERROR\r\n",RED);
	for(;;);
}

void I2C_Scanner(){
	uint8_t address;

	ConsolePuts_("Starting I2C scan...\r\n",BLUE);
	address = 0;
	do {
		if (qI2C_Write(address,NULL,0x00,0)==SUCCESS){
			ConsolePuts("0x");
			ConsolePutNumber(address,16);
			ConsolePuts(" Address found\r\n");
		}
	} while (address++ != 255);
	ConsolePuts_("I2C scan finished\r\n", BLUE);

}

void EEPROM_Test(){
	const uint16_t test_length = 10;
	const uint16_t start_position = 20;

	uint8_t txbuffer[test_length];
	uint8_t rxbuffer[test_length];
	uint16_t i;

	for (i=0;i<test_length;i++){
		txbuffer[i] = i%256;
		rxbuffer[i] = 0;
	}

	ConsolePuts_("Starting EEPROM Test ...\r\n",BLUE);
	ConsolePuts("Writing to EEPROM 8K buffer...\t\t\t\t");
	if(eeprom_write(EEPROM_ADDRESS,txbuffer,start_position,test_length)==SUCCESS){
		ConsolePuts_("[OK]\r\n",GREEN);
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}

	ConsolePuts("Reading from EEPROM 8K buffer...\t\t\t");
	if (eeprom_read(EEPROM_ADDRESS,rxbuffer,start_position,test_length)==SUCCESS){
		ConsolePuts_("[OK]\r\n",GREEN);
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}

	ConsolePuts("Comparing buffers...\t\t\t\t\t");
	for (i=0;i<test_length;i++){
		if (rxbuffer[i]!=txbuffer[i]){
			ConsolePuts_("[ERROR]\r\n",RED);
			halt();
		}
	}
	ConsolePuts_("[OK]\r\n",GREEN);
	ConsolePuts_("EEPROM read test finished\r\n", BLUE);
}

void ledTests(){
	uint8_t i;

	ConsolePuts_("Starting LEDs Test ...\r\n",BLUE);

	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOff(leds[i]);
	}

	ConsolePuts("Status led test\r\n");
	for (i=0;i<5;i++){
		qLed_TurnOn(STATUS_LED);
		delay(100);
		qLed_TurnOff(STATUS_LED);
		delay(100);
	}

	ConsolePuts("Sideleds test\r\n");
	for (i=0;i<5;i++){
		qLed_TurnOn(FRONT_LEFT_LED);
		delay(50);
		qLed_TurnOff(FRONT_LEFT_LED);
		qLed_TurnOn(FRONT_RIGHT_LED);
		delay(50);
		qLed_TurnOff(FRONT_RIGHT_LED);
		qLed_TurnOn(REAR_RIGHT_LED);
		delay(50);
		qLed_TurnOff(REAR_RIGHT_LED);
		qLed_TurnOn(REAR_LEFT_LED);
		delay(50);
		qLed_TurnOff(REAR_LEFT_LED);
	}

	ConsolePuts("External test\r\n");
	qLed_TurnOn(EXTERNAL_1_LED);
	delay(100);
	qLed_TurnOff(EXTERNAL_1_LED);
	qLed_TurnOn(EXTERNAL_2_LED);
	delay(100);
	qLed_TurnOff(EXTERNAL_2_LED);

	ConsolePuts_("LEDs test finished\r\n", BLUE);

}

void analogTest(){

	uint16_t temp,voltage;

	ConsolePuts_("Analog test starting\r\n", BLUE);

	qAnalog_InitPin(TEMPERATURE_ANALOG);
	qAnalog_InitPin(VOLTAGE_ANALOG);

	temp = qAnalog_Read(TEMPERATURE_ANALOG);
	temp = temp*3300/4096;
	temp = ((temp - 424)*100) / (625);

	voltage = qAnalog_Read(VOLTAGE_ANALOG);
	voltage = voltage*3300/4096;
	voltage = (voltage*764)/100;


	ConsolePuts("Temperature [C]: ");
	ConsolePutNumber(temp,10);
	ConsolePuts("\r\n Battery voltag [mV]: ");
	ConsolePutNumber(voltage,10);
	ConsolePuts("\r\n");

	ConsolePuts_("Analog test finished\r\n", BLUE);
}

int main(void) {


	//---------------------------------------------------------------
	// Inits
	//---------------------------------------------------------------
	delayInit();
	ConsoleInit();
	qAnalog_Init();

	ConsolePuts("\x1B[2J\x1B[0;0f");
	ConsolePuts("FLC V2.0 Initialized...\r\n");
	ConsolePuts("Initializing I2C driver...\t\t\t\t");

	if (qI2C_Init()==SUCCESS){
		ConsolePuts_("[OK]\r\n",GREEN);
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
		halt();
	}

/*
	ConsolePuts("------------------------------------------------------------\r\n");
	I2C_Scanner();
	ConsolePuts("------------------------------------------------------------\r\n");
	EEPROM_Test();
	ConsolePuts("------------------------------------------------------------\r\n");
	ledTests();
	ConsolePuts("------------------------------------------------------------\r\n");
	analogTest();
	ConsolePuts("------------------------------------------------------------\r\n");
*/
	PWM_TIMERCFG_Type PWMCfgDat;
	PWM_MATCHCFG_Type PWMMatchCfgDat;
	PINSEL_CFG_Type PinCfg;
	uint8_t temp;

	/* PWM block section -------------------------------------------- */
	/* Initialize PWM peripheral, timer mode
	 * PWM prescale value = 1 (absolute value - tick value) */
	PWMCfgDat.PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
	PWMCfgDat.PrescaleValue = 1;
	PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat);

	/*
	 * Initialize PWM pin connect
	 */
	PinCfg.Funcnum = 1; //son todos 1
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 4;
	PINSEL_ConfigPin(&PinCfg);

	/* Set match value for PWM match channel 0 = 256, update immediately */
	PWM_MatchUpdate(LPC_PWM1, 0, 100000, PWM_MATCH_UPDATE_NOW);
	/* PWM Timer/Counter will be reset when channel 0 matching
	 * no interrupt when match
	 * no stop when match */
	PWMMatchCfgDat.IntOnMatch = DISABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

	/* Configure PWM channel edge option
	 * Note: PWM Channel 1 is in single mode as default state and
	 * can not be changed to double edge mode */
	for (temp = 2; temp < 7; temp++)
	{
		PWM_ChannelConfig(LPC_PWM1, temp, PWM_CHANNEL_SINGLE_EDGE);
	}

	/* Set up match value */
	PWM_MatchUpdate(LPC_PWM1, 5, 25000, PWM_MATCH_UPDATE_NOW);

	/* Configure match option */
	PWMMatchCfgDat.IntOnMatch = DISABLE;
	PWMMatchCfgDat.MatchChannel = 5;
	PWMMatchCfgDat.ResetOnMatch = DISABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
	/* Enable PWM Channel Output */
	PWM_ChannelCmd(LPC_PWM1, 5, ENABLE);
	/* Increase match value by 10 */


	/* Reset and Start counter */
	PWM_ResetCounter(LPC_PWM1);
	PWM_CounterCmd(LPC_PWM1, ENABLE);

	/* Start PWM now */
	PWM_Cmd(LPC_PWM1, ENABLE);



	for(;;);
	return 0;
}



