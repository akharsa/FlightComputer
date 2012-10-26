#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "types.h"
#include "DebugConsole.h"
#include "qI2C.h"

#define EEPROM_ADDRESS		0xA0
#define MPU6050_ADDRESS		0xD0
#define HMC5883L_ADDRESS	0x3C
#define BMP085_ADDRESS		0xEE

void halt(){
	ConsolePuts_("Execution halted\r\n",RED);
	for(;;);
}

int main(void) {
	Status res;
	uint8_t whoiam;
	uint8_t address;

	//---------------------------------------------------------------
	// Inits
	//---------------------------------------------------------------

	ConsoleInit();

	ConsolePuts("\x1B[2J\x1B[0;0f");
	ConsolePuts("FLC V2.0 Initialized...\r\n");
	ConsolePuts("Initializing I2C driver...\t\t\t\t");

	if (qI2C_Init()==SUCCESS){
		ConsolePuts_("[OK]\r\n",GREEN);
	}else{
		ConsolePuts_("[ERROR]\r\n",RED);
	}

	//---------------------------------------------------------------
	// I2C Scanner
	//---------------------------------------------------------------
	ConsolePuts("Initializing I2C driver...\t\t\t\t");
	address = 0;
	do {
		if (qI2C_Write(address,NULL,0x00,0)==SUCCESS){
			ConsolePuts("0x");
			ConsolePutNumber(address,16);
			ConsolePuts(" Address found\r\n");
		}
	} while (address++ != 255);

	//---------------------------------------------------------------
	// EEPROM Test
	//---------------------------------------------------------------

	for(;;);
	return 0;
}


void SysTick_Handler(void)
{

}
