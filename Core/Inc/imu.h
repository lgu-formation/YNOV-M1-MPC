/*
 * imu.h
 *
 * IMU driver interfaces
 *
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Sample structure with accelerometer and gyroscope data */
typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
} ImuSample;

/* IMU status enum */
typedef enum {
	IMU_OK = 0,
	IMU_INVALID_ARGUMENT,
	IMU_COMMUNICATION_ERROR,
	IMU_BAD_ID
} ImuStatus;

/* IMU Initialisation function */
ImuStatus Imu_Init(I2C_HandleTypeDef *i2c);

/* */
ImuStatus Imu_EnableDataReady(bool enable);

/* */
ImuStatus Imu_ReadInterruptStatus(uint8_t *status);

/* Sample read function */
ImuStatus Imu_ReadSample(ImuSample *sample);

/* Value decoding function */
int16_t Imu_DecodeI16BE(uint8_t high, uint8_t low);

#endif /* INC_IMU_H_ */
