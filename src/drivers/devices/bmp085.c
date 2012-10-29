/*
 * bmp085HAL.c
 *
 *  Created on: 29/10/2012
 *      Author: alan
 */
#include "lpc17xx_gpio.h"
#include "types.h"
#include "bmp085.h"
#include "_bmp085.h"
#include "delay.h"
#include "qI2C.h"
#include "math.h"

static bmp085_t bmp;

static char read( unsigned char device_addr,unsigned char register_addr, unsigned char * register_data,  unsigned char read_length ){
	qI2C_Read(device_addr,register_data,register_addr,read_length);
	return 0;
}

static char write(unsigned char device_addr,unsigned char register_addr, unsigned char * register_data, unsigned char write_length ){
	qI2C_Write(device_addr,register_data,register_addr,write_length);
	return 0;
}


int32_t BMP085_GetPressure(){
	unsigned long up = bmp085_get_up();
	return bmp085_get_pressure(up);
}

int32_t BMP085_GetTemperature(){
	unsigned long ut = bmp085_get_ut();
	return bmp085_get_temperature(ut);
}

// in Pascals
float BMP085_CalculateAltitude(long sealevel, long actual){
	float altitude;
	altitude = 44330.0 * (1.0 - pow((float)actual /(float)sealevel,0.1903));
	return altitude;
}


Status BMP085_TestConnection(){
	if (bmp.chip_id==BMP085_CHIP_ID){
		return SUCCESS;
	}else{
		return ERROR;
	}
}

Status BMP085_Init(){
	// Reset pin as output
	GPIO_SetDir(1,(1<<1),1);

	// Short reset
	GPIO_ClearValue(1,(1<<1));
	delay(1);
	GPIO_SetValue(1,(1<<1));
	delay(10); //Datasheet spec T_start

	bmp.bus_write = write;
	bmp.bus_read = read;
	bmp.delay_msec = delay;

	bmp085_init(&bmp);

	return SUCCESS;
}
