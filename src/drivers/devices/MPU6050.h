// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


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
//#define MPU6050_ByteWrite(devAddr, regAddr, buffer) 				qI2C_Write(devAddr,&buffer,regAddr,1) //aca hay un problema con el puntero, tengo qeu hace un wrapper
#define MPU6050_ByteReads(devAddr, regAddr,size,  buffer) 			qI2C_Read(devAddr,buffer,regAddr,size)
#define MPU6050_WordWrite(devAddr, regAddr, buffer) 				qI2C_Write(devAddr,&buffer,regAddr,2)
#define MPU6050_ReadBit(devAddr, regAddr, bits, buffer) 			qI2C_ReadBit(devAddr,  regAddr,  bits, buffer)
#define MPU6050_ReadBits(devAddr, regAddr, bits, length, buffer)	qI2C_ReadBits(devAddr, regAddr, bits, length, buffer)
#define MPU6050_WriteBit(devAddr, regAddr, bits, buffer) 			qI2C_WriteBit(devAddr,  regAddr,  bits, buffer)
#define MPU6050_WriteBits(devAddr, regAddr, bits, length, buffer) 	qI2C_WriteBits(devAddr, regAddr, bits, length, buffer)

Status MPU6050_ByteWrite(uint8_t devAddr, uint8_t regAddr, uint8_t buffer);
Status MPU6050_ByteWrites(uint8_t devAddr, uint8_t regAddr,uint8_t size, uint8_t * buffer);

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
uint16_t MPU6050_getFIFOCount();

// FIFO_R_W register
uint8_t MPU6050_getFIFOByte();
void MPU6050_setFIFOByte(uint8_t data);
void MPU6050_getFIFOBytes(uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t MPU6050_getDeviceID();
void MPU6050_setDeviceID(uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
uint8_t MPU6050_getOTPBankValid();
void MPU6050_setOTPBankValid(bool enabled);
int8_t MPU6050_getXGyroOffset();
void MPU6050_setXGyroOffset(int8_t offset);

// YG_OFFS_TC register
int8_t MPU6050_getYGyroOffset();
void MPU6050_setYGyroOffset(int8_t offset);

// ZG_OFFS_TC register
int8_t MPU6050_getZGyroOffset();
void MPU6050_setZGyroOffset(int8_t offset);

// X_FINE_GAIN register
int8_t MPU6050_getXFineGain();
void MPU6050_setXFineGain(int8_t gain);

// Y_FINE_GAIN register
int8_t MPU6050_getYFineGain();
void MPU6050_setYFineGain(int8_t gain);

// Z_FINE_GAIN register
int8_t MPU6050_getZFineGain();
void MPU6050_setZFineGain(int8_t gain);

// XA_OFFS_* registers
int16_t MPU6050_getXAccelOffset();
void MPU6050_setXAccelOffset(int16_t offset);

// YA_OFFS_* register
int16_t MPU6050_getYAccelOffset();
void MPU6050_setYAccelOffset(int16_t offset);

// ZA_OFFS_* register
int16_t MPU6050_getZAccelOffset();
void MPU6050_setZAccelOffset(int16_t offset);

// XG_OFFS_USR* registers
int16_t MPU6050_getXGyroOffsetUser();
void MPU6050_setXGyroOffsetUser(int16_t offset);

// YG_OFFS_USR* register
int16_t MPU6050_getYGyroOffsetUser();
void MPU6050_setYGyroOffsetUser(int16_t offset);

// ZG_OFFS_USR* register
int16_t MPU6050_getZGyroOffsetUser();
void MPU6050_setZGyroOffsetUser(int16_t offset);

// INT_ENABLE register (DMP functions)
bool MPU6050_getIntPLLReadyEnabled();
void MPU6050_setIntPLLReadyEnabled(bool enabled);
bool MPU6050_getIntDMPEnabled();
void MPU6050_setIntDMPEnabled(bool enabled);

// DMP_INT_STATUS
bool MPU6050_getDMPInt5Status();
bool MPU6050_getDMPInt4Status();
bool MPU6050_getDMPInt3Status();
bool MPU6050_getDMPInt2Status();
bool MPU6050_getDMPInt1Status();
bool MPU6050_getDMPInt0Status();

// INT_STATUS register (DMP functions)
bool MPU6050_getIntPLLReadyStatus();
bool MPU6050_getIntDMPStatus();

// USER_CTRL register (DMP functions)
bool MPU6050_getDMPEnabled();
void MPU6050_setDMPEnabled(bool enabled);
void MPU6050_resetDMP();

// BANK_SEL register
void MPU6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank);

// MEM_START_ADDR register
void MPU6050_setMemoryStartAddress(uint8_t address);

// MEM_R_W register
uint8_t MPU6050_readMemoryByte();
void MPU6050_writeMemoryByte(uint8_t data);
void MPU6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address); // Los defaults eran 0 y 0 al final
bool MPU6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem);
bool MPU6050_writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);
bool MPU6050_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem);
bool MPU6050_writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

// DMP_CFG_1 register
uint8_t MPU6050_getDMPConfig1();
void MPU6050_setDMPConfig1(uint8_t config);

// DMP_CFG_2 register
uint8_t MPU6050_getDMPConfig2();
void MPU6050_setDMPConfig2(uint8_t config);

#ifndef MPU6050_INCLUDE_DMP_MOTIONAPPS20
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#endif

// special methods for MotionApps 2.0 implementation
#ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS20
    uint8_t *dmpPacketBuffer;
    uint16_t dmpPacketSize;

    uint8_t MPU6050_dmpInitialize();
    bool MPU6050_dmpPacketAvailable();

    uint8_t MPU6050_dmpSetFIFORate(uint8_t fifoRate);
    uint8_t MPU6050_dmpGetFIFORate();
    uint8_t MPU6050_dmpGetSampleStepSizeMS();
    uint8_t MPU6050_dmpGetSampleFrequency();
    int32_t MPU6050_dmpDecodeTemperature(int8_t tempReg);

    // Register callbacks after a packet of FIFO data is processed
    //uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
    //uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
    uint8_t MPU6050_dmpRunFIFORateProcesses();

    // Setup FIFO for various output
    uint8_t MPU6050_dmpSendQuaternion(uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendPacketNumber(uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t MPU6050_dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

    // Get Fixed Point data from FIFO
    uint8_t MPU6050_dmpGetAccel(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetAccel(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetAccel(VectorInt16 *v, const uint8_t* packet);
    uint8_t MPU6050_dmpGetQuaternion(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetQuaternion(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
    uint8_t MPU6050_dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet);
    uint8_t MPU6050_dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet);
    uint8_t MPU6050_dmpGetGyro(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGyro(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGyro(VectorInt16 *v, const uint8_t* packet);
    uint8_t MPU6050_dmpSetLinearAccelFilterCoefficient(float coef);
    uint8_t MPU6050_dmpGetLinearAccel(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetLinearAccel(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
    uint8_t MPU6050_dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
    uint8_t MPU6050_dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet);
    uint8_t MPU6050_dmpGetGyroSensor(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGyroSensor(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet);
    uint8_t MPU6050_dmpGetControlData(int32_t *data, const uint8_t* packet);
    uint8_t MPU6050_dmpGetTemperature(int32_t *data, const uint8_t* packet);
    uint8_t MPU6050_dmpGetGravity(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGravity(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGravity(VectorInt16 *v, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetGravity(VectorFloat *v, Quaternion *q);
    uint8_t MPU6050_dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet);
    uint8_t MPU6050_dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet);
    //uint8_t MPU6050_dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet);
    uint8_t MPU6050_dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet);
    uint8_t MPU6050_dmpGetEIS(int32_t *data, const uint8_t* packet);

    //uint8_t MPU6050_dmpGetEuler(float *data, Quaternion *q);
    //uint8_t MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

    // Get Floating Point data from FIFO
    uint8_t MPU6050_dmpGetAccelFloat(float *data, const uint8_t* packet);
    uint8_t MPU6050_dmpGetQuaternionFloat(float *data, const uint8_t* packet);

    uint8_t MPU6050_dmpProcessFIFOPacket(const unsigned char *dmpData);
    uint8_t MPU6050_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);
    //uint8_t MPU6050_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

    uint8_t MPU6050_dmpSetFIFOProcessedCallback(void (*func) (void));

    uint8_t MPU6050_dmpInitFIFOParam();
    uint8_t MPU6050_dmpCloseFIFO();
    uint8_t MPU6050_dmpSetGyroDataSource(uint8_t source);
    uint8_t MPU6050_dmpDecodeQuantizedAccel();
    uint32_t MPU6050_dmpGetGyroSumOfSquare();
    uint32_t MPU6050_dmpGetAccelSumOfSquare();
    void MPU6050_dmpOverrideQuaternion(long *q);
    uint16_t MPU6050_dmpGetFIFOPacketSize();
#endif

// special methods for MotionApps 4.1 implementation
#ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS41
    uint8_t *dmpPacketBuffer;
    uint16_t dmpPacketSize;

    uint8_t dmpInitialize();
    bool dmpPacketAvailable();

    uint8_t dmpSetFIFORate(uint8_t fifoRate);
    uint8_t dmpGetFIFORate();
    uint8_t dmpGetSampleStepSizeMS();
    uint8_t dmpGetSampleFrequency();
    int32_t dmpDecodeTemperature(int8_t tempReg);

    // Register callbacks after a packet of FIFO data is processed
    //uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
    //uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
    uint8_t dmpRunFIFORateProcesses();

    // Setup FIFO for various output
    uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
    uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
    uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

    // Get Fixed Point data from FIFO
    uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
    uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
    uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
    uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetMag(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
    uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
    uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
    uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
    uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
    uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
    uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
    uint8_t dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
    uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
    uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet=0);

    uint8_t dmpGetEuler(float *data, Quaternion *q);
    uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

    // Get Floating Point data from FIFO
    uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet=0);
    uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

    uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
    uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

    uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

    uint8_t dmpInitFIFOParam();
    uint8_t dmpCloseFIFO();
    uint8_t dmpSetGyroDataSource(uint8_t source);
    uint8_t dmpDecodeQuantizedAccel();
    uint32_t dmpGetGyroSumOfSquare();
    uint32_t dmpGetAccelSumOfSquare();
    void dmpOverrideQuaternion(long *q);
    uint16_t dmpGetFIFOPacketSize();


#endif /* __MPU6050_H */
#endif
