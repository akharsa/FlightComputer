#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif


#include "types.h"
#include "DebugConsole.h"
#include "qI2C.h"
#include "eeprom.h"
#include "board.h"
#include "delay.h"

#include "lpc17xx_adc.h"
#include "lpc17xx_pinsel.h"

#define EEPROM_ADDRESS		0xA0
#define MPU6050_ADDRESS		0xD0
#define HMC5883L_ADDRESS	0x3C
#define BMP085_ADDRESS		0xEE

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

int main(void) {


	//---------------------------------------------------------------
	// Inits
	//---------------------------------------------------------------
	delayInit();
	ConsoleInit();

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
	 */

	PINSEL_CFG_Type PinCfg;
	uint16_t value;

	PinCfg.Funcnum = PINSEL_FUNC_3;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum = PINSEL_PIN_31;
	PinCfg.Portnum = PINSEL_PORT_1;
	PINSEL_ConfigPin(&PinCfg);

	// Inicializamos el Conversor A/D
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN5,DISABLE);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_5,ENABLE);

	while(1){
		// Start conversion
		ADC_StartCmd(LPC_ADC,ADC_START_NOW);

		//Wait conversion complete
		while (!(ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_5,ADC_DATA_DONE)));
		value =  ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_5);
		ConsolePuts("Channel 5 [mV]: ");
		ConsolePutNumber((value*3300)/4096,10);
		//ConsolePutNumber(value,10);
		ConsolePuts("      \r");
		delay(200);
	}

	for(;;);
	return 0;
}



