/*
 * bmp085_HAL.h
 *
 *  Created on: 29/10/2012
 *      Author: alan
 */

#ifndef BMP085_HAL_H_
#define BMP085_HAL_H_


float BMP085_CalculateAltitude(long sealevel, long actual);
int32_t BMP085_GetTemperature();
int32_t BMP085_GetPressure();
Status BMP085_Init();
Status BMP085_TestConnection();

#endif /* BMP085_HAL_H_ */
