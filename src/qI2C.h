#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>
#include <lpc_types.h>
#define MAX_RETRANSMISSION	3
#define I2C_BITRATE			400000

#if 0
#define MPU6050_ByteRead(addr,reg,buffer) MPU6050_I2C_BufferRead(addr,buffer,reg,1)
#define MPU6050_ByteReads(slv,reg,size,buff) MPU6050_I2C_BufferRead(slv,buff,reg,size)


void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr);
#endif

Status qI2C_Init();
Status qI2C_Read(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead);
Status qI2C_Write(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t writeAddr, uint16_t NumByteToWrite);



#endif
