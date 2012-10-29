#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>
#include <lpc_types.h>
#define MAX_RETRANSMISSION	3
#define I2C_BITRATE			400000


Status qI2C_Init();

// 8 bit addressing on device
Status qI2C_Read(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead);
Status qI2C_Write(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t writeAddr, uint16_t NumByteToWrite);

// 16 bit addressing on device
Status qI2C_Write_(uint8_t slaveAddr,uint8_t* pBuffer, uint16_t writeAddr, uint16_t NumByteToWrite);
Status qI2C_Read_(uint8_t slaveAddr, uint8_t* pBuffer, uint16_t readAddr, uint16_t NumByteToRead);

// 1 Bit read-write
Status qI2C_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
Status qI2C_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t * data);

// Multiple bits read-write
Status qI2C_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
Status qI2C_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);


#endif
