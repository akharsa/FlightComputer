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

#include "lpc17xx_uart.h"

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

	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOff(leds[i]);
	}

	qPWM_Init(10000);
	qPWM_InitChannel(MOTOR1);
	qPWM_InitChannel(MOTOR2);
	qPWM_InitChannel(MOTOR3);
	qPWM_InitChannel(MOTOR4);

	qAnalog_Init();
	qAnalog_InitPin(TEMPERATURE_ANALOG);
	qAnalog_InitPin(VOLTAGE_ANALOG);

	if (qI2C_Init()!=SUCCESS) halt();

	BMP085_Init();
	if (BMP085_TestConnection()!=SUCCESS) halt();


	if (MPU6050_testConnection()==TRUE){
		MPU6050_initialize();
		//uint8_t dlpf = MPU6050_getDLPFMode();
		MPU6050_setDLPFMode(5);
	}else{
		halt();
	}

	int16_t mpucal[6];
	int32_t bias[6];
	int j;

	for (i=0;i<100;i++){

		MPU6050_getMotion6(&mpucal[0],&mpucal[1],&mpucal[2],&mpucal[3],&mpucal[4],&mpucal[5]);

		mpucal[2] = mpucal[2]-16536;

		for (j=0;j<6;j++){
			bias[j] = bias[j]+mpucal[j];
		}
	}

	for (j=0;j<6;j++){
		bias[j] = bias[j]/100;
	}

	while(1){
		qLed_TurnOn(STATUS_LED);

		/*
		board_temp = qAnalog_Read(TEMPERATURE_ANALOG);
		board_temp = board_temp*3300/4096;
		board_temp = ((board_temp - 424)*100) / (625);
		voltage = qAnalog_Read(VOLTAGE_ANALOG);
		voltage = voltage*3300/4096;
		voltage = (voltage*764)/100;
		bmp_temp = BMP085_GetTemperature();
		pressure = BMP085_GetPressure();
		altitude = ceil(BMP085_CalculateAltitude(101016,pressure)*100.0);
		 */

		MPU6050_getMotion6(&sensors[0],&sensors[1],&sensors[2],&sensors[3],&sensors[4],&sensors[5]);
		mpu_temp = MPU6050_getTemperature();
		mpu_temp = (mpu_temp+12421)/340;

		for (j=0;j<6;j++){
			sensors[j] = sensors[j] - bias[j];
		}

		UARTPuts(LPC_UART0,"M");
		UART_Send(LPC_UART0,sensors,sizeof(sensors),BLOCKING);
		UARTPuts(LPC_UART0,"\r\n");

		qLed_TurnOff(STATUS_LED);
		delay(100);

	}
	for(;;);
	return 0;
}


