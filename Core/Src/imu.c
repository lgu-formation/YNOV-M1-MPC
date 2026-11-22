/*
 * imu.c
 *
 * IMU driver implementation
 *
 */


#include "imu.h"

/* IMU address */
#define MPU_ADDR7				(0x68U)
#define MPU_ADDR_HAL			(MPU_ADDR7 << 1U)

/* IMU registers */
#define MPU_REG_SMPLRT_DIV		(0x19U)
#define MPU_REG_CONFIG			(0x1AU)
#define MPU_REG_GYRO_CONFIG		(0x1BU)
#define MPU_REG_ACCEL_CONFIG	(0x1CU)
#define MPU_REG_INT_PIN_CFG		(0x37U)
#define MPU_REG_INT_ENABLE		(0x38U)
#define MPU_REG_INT_STATUS		(0x3AU)
#define MPU_REG_ACCEL_XOUT_H	(0x3BU)
#define MPU_REG_PWR_MGMT_1		(0x6BU)
#define MPU_REG_WHO_AM_I		(0x75U)
#define MPU_EXPECTED_ID			(0x70U)
#define MPU_TIMEOUT_MS			(20U)

static I2C_HandleTypeDef *s_i2c = NULL;

static ImuStatus write_register(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef result;

	uint8_t data = value;
	result = HAL_I2C_Mem_Write( s_i2c,
								(uint16_t)MPU_ADDR_HAL,
								(uint16_t)reg,
								I2C_MEMADD_SIZE_8BIT,
								&data,
								1U,
								MPU_TIMEOUT_MS);

	return(result == HAL_OK) ? IMU_OK : IMU_COMMUNICATION_ERROR;
}

static ImuStatus read_register(uint8_t reg, uint8_t *value)
{
	HAL_StatusTypeDef result;

	if ((s_i2c == NULL) || (value == NULL))
	{
		return IMU_INVALID_ARGUMENT;
	}

	result = HAL_I2C_Mem_Read(  s_i2c,
								(uint16_t)MPU_ADDR_HAL,
								(uint16_t)reg,
								I2C_MEMADD_SIZE_8BIT,
								value,
								1U,
								MPU_TIMEOUT_MS);

	return (result == HAL_OK) ? IMU_OK : IMU_COMMUNICATION_ERROR;
}


int16_t Imu_DecodeI16BE(uint8_t high, uint8_t low)
{

	const uint32_t raw = ((uint32_t)high << 8U) | (uint32_t)low;
	int32_t signed_value = (int32_t)raw;

	if (raw >= 32768U)
	{
		signed_value -= 65536;
	}

	return (int16_t)signed_value;
}

ImuStatus Imu_Init(I2C_HandleTypeDef *i2c)
{
	ImuStatus result;

	uint8_t id = 0U;
	if (i2c == NULL)
	{
		return IMU_INVALID_ARGUMENT;
	}
	s_i2c = i2c;
	result = read_register(MPU_REG_WHO_AM_I, &id);
	if (result != IMU_OK)
	{
		return result;
	}
	if (id != MPU_EXPECTED_ID)
	{
		return IMU_BAD_ID;
	}

	/* PLL X-gyro clock. Wait for stable oscillator after waking. */
	/*
	* 0x01U : wake with PLL on X gyro (100ms to stabilize).
	*/
	result = write_register(MPU_REG_PWR_MGMT_1, 0x01U);
	if (result != IMU_OK)
	{
		return result;
	}
	HAL_Delay(100U);

	/* DLPF_CFG=3: gyro ~42 Hz, accel ~44 Hz; output base = 1 kHz. */
	/*
	* DLPF_CFG=3 : fréquence de base gyro 1 kHz, bande limitée.
	* Le diviseur 9 ci-dessous fournit une interruption a 100 Hz.
	*/
	result = write_register(MPU_REG_CONFIG, 0x03U);
	if (result != IMU_OK)
	{
		return result;
	}

	/* 1 kHz / (9+1) = 100 Hz. */
	result = write_register(MPU_REG_SMPLRT_DIV, 0x09U);
	if (result != IMU_OK)
	{
		return result;
	}

	/* gyro +/- 250 deg/s: 131 LSB/(deg/s) */
	result = write_register(MPU_REG_GYRO_CONFIG, 0x00U);
	if (result != IMU_OK)
	{
		return result;
	}

	/* accel +/- 2g: 16384 LSB/g */
	result = write_register(MPU_REG_ACCEL_CONFIG, 0x00U);
	if (result != IMU_OK)
	{
		return result;
	}

	/* Active-high push-pull; latch until INT_STATUS is read. */
	/*
	* 0x20U active LATCH_INT_EN : INT conserve son niveau jusqu'à
	* la lecture de INT_STATUS ; PC2 doit être configure en EXTI.
	*/
	result = write_register(MPU_REG_INT_PIN_CFG, 0x20U);
	if (result != IMU_OK)
	{
		return result;
	}

	return write_register(MPU_REG_INT_ENABLE, 0x00U);
}

ImuStatus Imu_EnableDataReady(bool enable)
{
	uint8_t check = 0U;
	ImuStatus result;

	if (s_i2c == NULL)
	{
		return IMU_INVALID_ARGUMENT;
	}

	result = write_register(MPU_REG_INT_ENABLE, enable ? 0x01U : 0x00U);

	if (result != IMU_OK)
		return result;
	result = read_register(MPU_REG_INT_ENABLE, &check);

	return write_register(MPU_REG_INT_ENABLE, enable ? 0x01U : 0x00U);
}

ImuStatus Imu_ReadInterruptStatus(uint8_t *status)
{
	return read_register(MPU_REG_INT_STATUS, status);
}


ImuStatus Imu_ReadSample(ImuSample *sample)
{
	/*
	* 14 octets contigus : 6 accel, 2 temperature, 6 gyro.
	* uint8_t garantit des valeurs de bus 0..255 ; buffer initialisé.
	*/
	uint8_t b[14] = {0U};
	HAL_StatusTypeDef result;

	ImuSample decoded = {0};
	if ((sample == NULL) || (s_i2c == NULL))
	{
		return IMU_INVALID_ARGUMENT;
	}

	result = HAL_I2C_Mem_Read(  s_i2c,
								(uint16_t)MPU_ADDR_HAL,
								MPU_REG_ACCEL_XOUT_H,
								I2C_MEMADD_SIZE_8BIT,
								b,
								(uint16_t)sizeof(b),
								MPU_TIMEOUT_MS);

	if (result != HAL_OK)
	{
		/* Leave the caller's previous sample unchanged on failure. */
		return IMU_COMMUNICATION_ERROR;
	}


	decoded.ax = Imu_DecodeI16BE(b[0], b[1]);
	decoded.ay = Imu_DecodeI16BE(b[2], b[3]);
	decoded.az = Imu_DecodeI16BE(b[4], b[5]);

	decoded.gx = Imu_DecodeI16BE(b[8], b[9]);
	decoded.gy = Imu_DecodeI16BE(b[10], b[11]);
	decoded.gz = Imu_DecodeI16BE(b[12], b[13]);

	*sample = decoded;

	return IMU_OK;
}




