#ifndef _MPU6050_H_
#define _MPU6050_H_

/* Includes */
#include "MPU6050_registers.h"
#include "qI2C.h"
#include "types.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     (MPU6050_ADDRESS_AD0_LOW<<1)

#define MPU6050_ByteRead(devAddr, regAddr, buffer) 					qI2C_Read(devAddr,buffer,regAddr,1)
#define MPU6050_ByteWrite(devAddr, regAddr, buffer) 				qI2C_Write(devAddr,&buffer,regAddr,1) //aca hay un problema con el puntero, tengo qeu hace un wrapper
#define MPU6050_ByteReads(devAddr, regAddr,size,  buffer) 			qI2C_Read(devAddr,buffer,regAddr,size)
#define MPU6050_WordWrite(devAddr, regAddr, buffer) 				qI2C_Write(devAddr,&buffer,regAddr,2)
#define MPU6050_ReadBit(devAddr, regAddr, bits, buffer) 			qI2C_ReadBit(devAddr,  regAddr,  bits, buffer)
#define MPU6050_ReadBits(devAddr, regAddr, bits, length, buffer)	qI2C_ReadBits(devAddr, regAddr, bits, length, buffer)
#define MPU6050_WriteBit(devAddr, regAddr, bits, buffer) 			qI2C_WriteBit(devAddr,  regAddr,  bits, buffer)
#define MPU6050_WriteBits(devAddr, regAddr, bits, length, buffer) 	qI2C_WriteBits(devAddr, regAddr, bits, length, buffer)


void MPU6050_initialize();
bool MPU6050_testConnection();

// PWR_MGMT_1 register
bool MPU6050_GetSleepModeStatus();
void MPU6050_SetSleepModeStatus(FunctionalState NewState);
void MPU6050_SetClockSource(uint8_t source);

// WHO_AM_I register
uint8_t MPU6050_GetDeviceID();

// AUX_VDDIO register
uint8_t MPU6050_getAuxVDDIOLevel();
void MPU6050_setAuxVDDIOLevel(uint8_t level);

// SMPLRT_DIV register
uint8_t MPU6050_getRate();
void MPU6050_setRate(uint8_t rate);

// CONFIG register
uint8_t MPU6050_getExternalFrameSync();
void MPU6050_setExternalFrameSync(uint8_t sync);
uint8_t MPU6050_getDLPFMode();
void MPU6050_setDLPFMode(uint8_t bandwidth);

// GYRO_CONFIG register
uint8_t MPU6050_getFullScaleGyroRange();
void MPU6050_setFullScaleGyroRange(uint8_t range);

// ACCEL_CONFIG register
bool MPU6050_getAccelXSelfTest();
void MPU6050_setAccelXSelfTest(bool enabled);
bool MPU6050_getAccelYSelfTest();
void MPU6050_setAccelYSelfTest(bool enabled);
bool MPU6050_getAccelZSelfTest();
void MPU6050_setAccelZSelfTest(bool enabled);
uint8_t MPU6050_getFullScaleAccelRange();
void MPU6050_setFullScaleAccelRange(uint8_t range);
uint8_t MPU6050_getDHPFMode();
void MPU6050_setDHPFMode(uint8_t mode);

// FF_THR register
uint8_t MPU6050_getFreefallDetectionThreshold();
void MPU6050_setFreefallDetectionThreshold(uint8_t threshold);

// FF_DUR register
uint8_t MPU6050_getFreefallDetectionDuration();
void MPU6050_setFreefallDetectionDuration(uint8_t duration);

// MOT_THR register
uint8_t MPU6050_getMotionDetectionThreshold();
void MPU6050_setMotionDetectionThreshold(uint8_t threshold);

// MOT_DUR register
uint8_t MPU6050_getMotionDetectionDuration();
void MPU6050_setMotionDetectionDuration(uint8_t duration);

// ZRMOT_THR register
uint8_t MPU6050_getZeroMotionDetectionThreshold();
void MPU6050_setZeroMotionDetectionThreshold(uint8_t threshold);

// ZRMOT_DUR register
uint8_t MPU6050_getZeroMotionDetectionDuration();
void MPU6050_setZeroMotionDetectionDuration(uint8_t duration);

// FIFO_EN register
bool MPU6050_getTempFIFOEnabled();
void MPU6050_setTempFIFOEnabled(bool enabled);
bool MPU6050_getXGyroFIFOEnabled();
void MPU6050_setXGyroFIFOEnabled(bool enabled);
bool MPU6050_getYGyroFIFOEnabled();
void MPU6050_setYGyroFIFOEnabled(bool enabled);
bool MPU6050_getZGyroFIFOEnabled();
void MPU6050_setZGyroFIFOEnabled(bool enabled);
bool MPU6050_getAccelFIFOEnabled();
void MPU6050_setAccelFIFOEnabled(bool enabled);
bool MPU6050_getSlave2FIFOEnabled();
void MPU6050_setSlave2FIFOEnabled(bool enabled);
bool MPU6050_getSlave1FIFOEnabled();
void MPU6050_setSlave1FIFOEnabled(bool enabled);
bool MPU6050_getSlave0FIFOEnabled();
void MPU6050_setSlave0FIFOEnabled(bool enabled);

// I2C_MST_CTRL register
bool MPU6050_getMultiMasterEnabled();
void MPU6050_setMultiMasterEnabled(bool enabled);
bool MPU6050_getWaitForExternalSensorEnabled();
void MPU6050_setWaitForExternalSensorEnabled(bool enabled);
bool MPU6050_getSlave3FIFOEnabled();
void MPU6050_setSlave3FIFOEnabled(bool enabled);
bool MPU6050_getSlaveReadWriteTransitionEnabled();
void MPU6050_setSlaveReadWriteTransitionEnabled(bool enabled);
uint8_t MPU6050_getMasterClockSpeed();
void MPU6050_setMasterClockSpeed(uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t MPU6050_getSlaveAddress(uint8_t num);
void MPU6050_setSlaveAddress(uint8_t num, uint8_t address);
uint8_t MPU6050_getSlaveRegister(uint8_t num);
void MPU6050_setSlaveRegister(uint8_t num, uint8_t reg);
bool MPU6050_getSlaveEnabled(uint8_t num);
void MPU6050_setSlaveEnabled(uint8_t num, bool enabled);
bool MPU6050_getSlaveWordByteSwap(uint8_t num);
void MPU6050_setSlaveWordByteSwap(uint8_t num, bool enabled);
bool MPU6050_getSlaveWriteMode(uint8_t num);
void MPU6050_setSlaveWriteMode(uint8_t num, bool mode);
bool MPU6050_getSlaveWordGroupOffset(uint8_t num);
void MPU6050_setSlaveWordGroupOffset(uint8_t num, bool enabled);
uint8_t MPU6050_getSlaveDataLength(uint8_t num);
void MPU6050_setSlaveDataLength(uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
uint8_t MPU6050_getSlave4Address();
void MPU6050_setSlave4Address(uint8_t address);
uint8_t MPU6050_getSlave4Register();
void MPU6050_setSlave4Register(uint8_t reg);
void MPU6050_setSlave4OutputByte(uint8_t data);
bool MPU6050_getSlave4Enabled();
void MPU6050_setSlave4Enabled(bool enabled);
bool MPU6050_getSlave4InterruptEnabled();
void MPU6050_setSlave4InterruptEnabled(bool enabled);
bool MPU6050_getSlave4WriteMode();
void MPU6050_setSlave4WriteMode(bool mode);
uint8_t MPU6050_getSlave4MasterDelay();
void MPU6050_setSlave4MasterDelay(uint8_t delay);
uint8_t MPU6050_getSlate4InputByte();

// I2C_MST_STATUS register
bool MPU6050_getPassthroughStatus();
bool MPU6050_getSlave4IsDone();
bool MPU6050_getLostArbitration();
bool MPU6050_getSlave4Nack();
bool MPU6050_getSlave3Nack();
bool MPU6050_getSlave2Nack();
bool MPU6050_getSlave1Nack();
bool MPU6050_getSlave0Nack();

// INT_PIN_CFG register
bool MPU6050_getInterruptMode();
void MPU6050_setInterruptMode(bool mode);
bool MPU6050_getInterruptDrive();
void MPU6050_setInterruptDrive(bool drive);
bool MPU6050_getInterruptLatch();
void MPU6050_setInterruptLatch(bool latch);
bool MPU6050_getInterruptLatchClear();
void MPU6050_setInterruptLatchClear(bool clear);
bool MPU6050_getFSyncInterruptLevel();
void MPU6050_setFSyncInterruptLevel(bool level);
bool MPU6050_getFSyncInterruptEnabled();
void MPU6050_setFSyncInterruptEnabled(bool enabled);
bool MPU6050_getI2CBypassEnabled();
void MPU6050_setI2CBypassEnabled(bool enabled);
bool MPU6050_getClockOutputEnabled();
void MPU6050_setClockOutputEnabled(bool enabled);

// INT_ENABLE register
uint8_t MPU6050_getIntEnabled();
void MPU6050_setIntEnabled(uint8_t enabled);
bool MPU6050_getIntFreefallEnabled();
void MPU6050_setIntFreefallEnabled(bool enabled);
bool MPU6050_getIntMotionEnabled();
void MPU6050_setIntMotionEnabled(bool enabled);
bool MPU6050_getIntZeroMotionEnabled();
void MPU6050_setIntZeroMotionEnabled(bool enabled);
bool MPU6050_getIntFIFOBufferOverflowEnabled();
void MPU6050_setIntFIFOBufferOverflowEnabled(bool enabled);
bool MPU6050_getIntI2CMasterEnabled();
void MPU6050_setIntI2CMasterEnabled(bool enabled);
bool MPU6050_getIntDataReadyEnabled();
void MPU6050_setIntDataReadyEnabled(bool enabled);

// INT_STATUS register
uint8_t MPU6050_getIntStatus();
bool MPU6050_getIntFreefallStatus();
bool MPU6050_getIntMotionStatus();
bool MPU6050_getIntZeroMotionStatus();
bool MPU6050_getIntFIFOBufferOverflowStatus();
bool MPU6050_getIntI2CMasterStatus();
bool MPU6050_getIntDataReadyStatus();

// ACCEL_*OUT_* registers
void MPU6050_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx,
		int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx,
		int16_t* gy, int16_t* gz);
void MPU6050_getAcceleration(int16_t* x, int16_t* y, int16_t* z);
int16_t MPU6050_getAccelerationX();
int16_t MPU6050_getAccelerationY();
int16_t MPU6050_getAccelerationZ();

// TEMP_OUT_* registers
int16_t MPU6050_getTemperature();

// GYRO_*OUT_* registers
void MPU6050_getRotation(int16_t* x, int16_t* y, int16_t* z);
int16_t MPU6050_getRotationX();
int16_t MPU6050_getRotationY();
int16_t MPU6050_getRotationZ();

// EXT_SENS_DATA_* registers
uint8_t MPU6050_getExternalSensorByte(int position);
uint16_t MPU6050_getExternalSensorWord(int position);
uint32_t MPU6050_getExternalSensorDWord(int position);

// MOT_DETECT_STATUS register
bool MPU6050_getXNegMotionDetected();
bool MPU6050_getXPosMotionDetected();
bool MPU6050_getYNegMotionDetected();
bool MPU6050_getYPosMotionDetected();
bool MPU6050_getZNegMotionDetected();
bool MPU6050_getZPosMotionDetected();
bool MPU6050_getZeroMotionDetected();

// I2C_SLV*_DO register
void MPU6050_setSlaveOutputByte(uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
bool MPU6050_getExternalShadowDelayEnabled();
void MPU6050_setExternalShadowDelayEnabled(bool enabled);
bool MPU6050_getSlaveDelayEnabled(uint8_t num);
void MPU6050_setSlaveDelayEnabled(uint8_t num, bool enabled);

// SIGNAL_PATH_RESET register
void MPU6050_resetGyroscopePath();
void MPU6050_resetAccelerometerPath();
void MPU6050_resetTemperaturePath();

// MOT_DETECT_CTRL register
uint8_t MPU6050_getAccelerometerPowerOnDelay();
void MPU6050_setAccelerometerPowerOnDelay(uint8_t delay);
uint8_t MPU6050_getFreefallDetectionCounterDecrement();
void MPU6050_setFreefallDetectionCounterDecrement(uint8_t decrement);
uint8_t MPU6050_getMotionDetectionCounterDecrement();
void MPU6050_setMotionDetectionCounterDecrement(uint8_t decrement);

// USER_CTRL register
bool MPU6050_getFIFOEnabled();
void MPU6050_setFIFOEnabled(bool enabled);
bool MPU6050_getI2CMasterModeEnabled();
void MPU6050_setI2CMasterModeEnabled(bool enabled);
void MPU6050_switchSPIEnabled(bool enabled);
void MPU6050_resetFIFO();
void MPU6050_resetI2CMaster();
void MPU6050_resetSensors();

// PWR_MGMT_1 register
void MPU6050_reset();
bool MPU6050_getSleepEnabled();
void MPU6050_setSleepEnabled(bool enabled);
bool MPU6050_getWakeCycleEnabled();
void MPU6050_setWakeCycleEnabled(bool enabled);
bool MPU6050_getTempSensorEnabled();
void MPU6050_setTempSensorEnabled(bool enabled);
uint8_t MPU6050_getClockSource();
void MPU6050_setClockSource(uint8_t source);

// PWR_MGMT_2 register
uint8_t MPU6050_getWakeFrequency();
void MPU6050_setWakeFrequency(uint8_t frequency);
bool MPU6050_getStandbyXAccelEnabled();
void MPU6050_setStandbyXAccelEnabled(bool enabled);
bool MPU6050_getStandbyYAccelEnabled();
void MPU6050_setStandbyYAccelEnabled(bool enabled);
bool MPU6050_getStandbyZAccelEnabled();
void MPU6050_setStandbyZAccelEnabled(bool enabled);
bool MPU6050_getStandbyXGyroEnabled();
void MPU6050_setStandbyXGyroEnabled(bool enabled);
bool MPU6050_getStandbyYGyroEnabled();
void MPU6050_setStandbyYGyroEnabled(bool enabled);
bool MPU6050_getStandbyZGyroEnabled();
void MPU6050_setStandbyZGyroEnabled(bool enabled);

// FIFO_COUNT_* registers
uint16_t MPU6050_MPU6050_getFIFOCount();

// FIFO_R_W register
uint8_t MPU6050_getFIFOByte();
void MPU6050_setFIFOByte(uint8_t data);
void MPU6050_getFIFOBytes(uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t MPU6050_getDeviceID();
void MPU6050_setDeviceID(uint8_t id);

#endif /* __MPU6050_H */

