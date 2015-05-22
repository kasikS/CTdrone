/*
 * drv_mpu6050.h
 *
 *  Created on:
 *      Author: Katarzyna Stachyra <kas.stachyra@gmail.com>
 */
 
#pragma once
#include <semphr.h>
#include <stdbool.h>

#define M_PI       3.14159265358979323846f

typedef enum
{
	MPU6050_SCALE_2000DPS = 0b11,
	MPU6050_SCALE_1000DPS = 0b10,
	MPU6050_SCALE_500DPS = 0b01,
	MPU6050_SCALE_250DPS = 0b00
} mpu6050_dps_t;

typedef enum
{
	MPU6050_RANGE_16G = 0b11,
	MPU6050_RANGE_8G = 0b10,
	MPU6050_RANGE_4G = 0b01,
	MPU6050_RANGE_2G = 0b00,
} mpu6050_range_t;

typedef struct axis{
	float x;
	float y;
	float z;
}axis;

typedef struct angles{
	float yaw;
	float pitch;
	float roll;
}angles;


typedef struct quaternion{
	float qw;
	float qx;
	float qy;
	float qz;
}quaternion;

extern xSemaphoreHandle imu_data_rdy, imu_data_update;
extern volatile angles imu_position;

bool sensorsAutodetect(void);
bool mpu6050Detect();
void mpu6050AccInit(mpu6050_range_t range);
void mpu6050AccRead(int16_t *accData);
void mpu6050AccReadScaled(int16_t *accData);
void mpu6050AccReadNormalized(int16_t *accData);
void mpu6050GyroInit(mpu6050_dps_t scale);
void mpu6050GyroReadScaled(int16_t * gyroData);
void mpu6050GyroRead(int16_t * gyroData);
void mpu6050GyroCalibrate(int16_t * gyroData, uint8_t samples);
void mpu6050GyroReadNormalized(int16_t * gyroData);
void mpu6050GyroDetThreshold(int16_t * gyroData, uint8_t samples, uint8_t multiple);

uint8_t mpu6050_dmpInitialize(void);
uint8_t mpu6050_dmpInitialize6(void);
void mpu6050_dmpEnable(void);
void mpu6050_dmpDisable(void);
void mpu6050_setMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank);
void mpu6050_setMemoryStartAddress(uint8_t address);
void mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
uint8_t mpu6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
uint8_t mpu6050_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);
uint16_t mpu6050_getFIFOCount(void);
void mpu6050_getFIFOBytes(uint8_t *data, uint8_t length);
uint8_t mpu6050_getIntStatus(void);
void mpu6050_resetFIFO(void);

int8_t mpu6050_getXGyroOffsetTC(void);
void mpu6050_setXGyroOffsetTC(int8_t offset);
int8_t mpu6050_getYGyroOffsetTC(void);
void mpu6050_setYGyroOffsetTC(int8_t offset);
int8_t mpu6050_getZGyroOffsetTC(void);
void mpu6050_setZGyroOffsetTC(int8_t offset);

int16_t mpu6050_getXAccelOffset(void);
void mpu6050_setXAccelOffset(int16_t offset);
int16_t mpu6050_getYAccelOffset(void);
void mpu6050_setYAccelOffset(int16_t offset);
int16_t mpu6050_getZAccelOffset(void);
void mpu6050_setZAccelOffset(int16_t offset);
int16_t mpu6050_getXGyroOffset(void);
void mpu6050_setXGyroOffset(int16_t offset);
int16_t mpu6050_getYGyroOffset(void);
void mpu6050_setYGyroOffset(int16_t offset);
int16_t mpu6050_getZGyroOffset(void);
void mpu6050_setZGyroOffset(int16_t offset);

void mpu6050_setSleepDisabled(void);
void mpu6050_setSleepEnabled(void);

void mpu6050_getQuaternion(const uint8_t* packet, quaternion *quat);
uint8_t mpu6050_getGravity(axis *gravity, quaternion *quat);
void mpu6050_getYawPitchRoll(quaternion *quat, angles *angle, axis *gravity);
uint8_t mpu6050_getEuler(float *data, quaternion *quat);
void mpu6050_setClockSource(uint8_t source);
void mpu6050_reset(void);

void mpu6050_setI2CMasterModeEnabled(bool enabled);

void mpu6050_setI2CBypassEnabled(bool enabled);
void mpu6050_setSlaveAddress(uint8_t num, uint8_t address);

void mpu6050_setSlaveControl(uint8_t num, uint8_t enabled, bool word_byte_swap, bool write_mode, bool group_offset, uint8_t length);
uint8_t mpu6050_getSlaveAddress(uint8_t num);
void mpu6050_setSlaveAddress(uint8_t num, uint8_t address);
uint8_t mpu6050_getSlaveRegister(uint8_t num);
void mpu6050_setSlaveRegister(uint8_t num, uint8_t reg);
bool mpu6050_getSlaveEnabled(uint8_t num);
void mpu6050_setSlaveEnabled(uint8_t num, bool enabled);
bool mpu6050_getSlaveWordByteSwap(uint8_t num);
void mpu6050_setSlaveWordByteSwap(uint8_t num, bool enabled);
bool mpu6050_getSlaveWriteMode(uint8_t num);
void mpu6050_setSlaveWriteMode(uint8_t num, bool mode);
bool mpu6050_getSlaveWordGroupOffset(uint8_t num);
void mpu6050_setSlaveWordGroupOffset(uint8_t num, bool enabled);
uint8_t mpu6050_getSlaveDataLength(uint8_t num);
void mpu6050_setSlaveDataLength(uint8_t num, uint8_t length);
void mpu6050_getMagDMP(const uint8_t* packet, axis *mag);
long map(long x, long in_min, long in_max, long out_min, long out_max);

int imu_init(void);
