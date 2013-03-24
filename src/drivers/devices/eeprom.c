
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

#if 0
//FIXME: No paging considered. Not boundary case handled.
//FIXME: Add delay for writing?
Status eeprom_write(uint8_t devAddr, uint8_t* buf, uint16_t position, uint16_t len)
{
	Status res;

	if (((position+len)>EEPROM_TOTAL_SIZE) || (len>EEPROM_TOTAL_SIZE)){
        res = ERROR;
    }else{
    	uint8_t pagestoWrite;
    	uint16_t bytesWritten = 0;

    	if (len<=32){
    		pagestoWrite = 1;
    	}else{
    		pagestoWrite = (position+len) / EEPROM_PAGE_SIZE;
    		if (((position+len) % EEPROM_PAGE_SIZE) >0) pagestoWrite++;
    	}
    	for (;pagestoWrite>0;pagestoWrite--){
    		res = qI2C_Write_(devAddr,buf,position,len);
    	}
    	//res = qI2C_Write_(devAddr,buf,position,len);
    }

	return res;
}
#endif

// FIXME: Write only work with aligned page write (position multiple of pagesize or 0)
Status eeprom_write(uint8_t devAddr, uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite){
	uint8_t NumOfPage = 0, NumOfSingle = 0;
	uint16_t Addr = 0,count = 0;
	uint8_t *ptr = (uint8_t *)pBuffer;

	Addr = (uint16_t)(WriteAddr&0xFFFF);
	Status res;
	count = (uint16_t)(NumByteToWrite&0xFFFF);

	if ((WriteAddr + NumByteToWrite) > EEPROM_TOTAL_SIZE)
			return ERROR;

	while (count >= EEPROM_PAGE_SIZE)
	{
			res = qI2C_Write_(devAddr,ptr,Addr,EEPROM_PAGE_SIZE);
			if (res!=SUCCESS){
				return res;
			}
			Addr += EEPROM_PAGE_SIZE;
			count -= EEPROM_PAGE_SIZE;
			ptr += EEPROM_PAGE_SIZE;
			DELAY(5);
	}

	while (count)
	{
			//EE_WriteByte(ptr++, Addr++);
			res = qI2C_Write_(devAddr,ptr++,Addr++,1);
			if (res!=SUCCESS){
				return res;
			}
			count--;
			DELAY(5);
	}

	return SUCCESS;
}
