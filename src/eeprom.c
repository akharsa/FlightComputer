
#include "eeprom.h"
#include "qI2C.h"
#include "types.h"

Status eeprom_init (void)
{
	return SUCCESS;
}

Status eeprom_read(uint8_t devAddr, uint8_t* buf, uint16_t position, uint16_t len)
{
	Status res;

	if (((position+len)>EEPROM_TOTAL_SIZE) || (len>EEPROM_TOTAL_SIZE)){
		// Out of range
		res = ERROR;
	}else{
		res = qI2C_Read_(devAddr,buf,position,len);
	}

	return res;
}

//FIXME: No paging considered. Not boundary case handled.
Status eeprom_write(uint8_t devAddr, uint8_t* buf, uint16_t position, uint16_t len)
{
	Status res;

	if (((position+len)>EEPROM_TOTAL_SIZE) || (len>EEPROM_TOTAL_SIZE)){
        res = ERROR;
    }else{
    	res = qI2C_Write_(devAddr,buf,position,len);
    }

	return res;
}
