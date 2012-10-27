/*
 * eeprom.h
 *
 *  Created on: 26/10/2012
 *      Author: alan
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "types.h"

#define EEPROM_TOTAL_SIZE 8000		// in KBytes
#define EEPROM_PAGE_SIZE    32

Status eeprom_init (void);
Status eeprom_read(uint8_t devAddr, uint8_t* buf, uint16_t position, uint16_t len);
Status eeprom_write(uint8_t devAddr, uint8_t* buf, uint16_t position, uint16_t len);

#endif /* EEPROM_H_ */
