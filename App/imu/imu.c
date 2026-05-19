#include "imu.h"
#include "config.h"
#include "../utils/time.h"
#include "../utils/math_utils.h"

#include "stm32f4xx_hal.h"
#include "i2c.h"   /* declares extern I2C_HandleTypeDef IMU_I2C_HANDLE */

// MPU6050 register map (subset)
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

#define MPU_WHOAMI_VAL       0x68

#define I2C_TIMEOUT_MS       5U

/* Scale factors derived from full-scale settings in config.h.
 * MPU6050 raw is int16: ±32768 maps to ±FS. */

#define G_TO_MS2  9.80665f
static const float ACCEL_SCALE = (IMU_ACCEL_FS_G * G_TO_MS2) / 32768.0f;
static const float GYRO_SCALE  = (IMU_GYRO_FS_DPS * (M_PI / 180.0f)) / 32768.0f;

//static imu_roll_value, imu_pitch_value;

static HAL_StatusTypeDef imu_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&IMU_I2C_HANDLE,
							 MPU6050_I2C_ADDR,
							 reg,
							 I2C_MEMADD_SIZE_8BIT,
						 	 &value,
						 	 1,
							 I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef imu_read_regs(uint8_t reg,
                                       uint8_t *buf,
                                       uint16_t len)
{
    return HAL_I2C_Mem_Read(&IMU_I2C_HANDLE,
							MPU6050_I2C_ADDR,
							reg,
							I2C_MEMADD_SIZE_8BIT,
							buf,
							len,
							I2C_TIMEOUT_MS);
}

int imu_init(void)
{
    uint8_t whoami = 0;
    HAL_Delay(100);

    if (imu_read_regs(MPU_REG_WHO_AM_I, &whoami, 1) != HAL_OK)
        return -1;

    if (whoami != MPU_WHOAMI_VAL)
        return -2;

    /* Wake up device */
    if (imu_write_reg(MPU_REG_PWR_MGMT_1, 0x00) != HAL_OK)
        return -3;

    HAL_Delay(10);

    /* Sample rate divider */
    if (imu_write_reg(MPU_REG_SMPLRT_DIV, 0x07) != HAL_OK)
        return -4;

    /* DLPF config */
    if (imu_write_reg(MPU_REG_CONFIG, 0x03) != HAL_OK)
        return -5;

    /* Gyro FS = ±1000 dps */
    if (imu_write_reg(MPU_REG_GYRO_CONFIG, 0x10) != HAL_OK)
        return -6;

    /* Accel FS = ±8g */
    if (imu_write_reg(MPU_REG_ACCEL_CONFIG, 0x10) != HAL_OK)
        return -7;

    return 0;
}


int imu_read(imu_data_t *out)
{
	uint8_t raw[14];

	int16_t ax_raw;
	int16_t ay_raw;
	int16_t az_raw;

	int16_t gx_raw;
	int16_t gy_raw;
	int16_t gz_raw;

	if (out == NULL) {
		return -1;
	}

	if (imu_read_regs(MPU_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK) {
		return -2;
	}

	ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
	ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
	az_raw = (int16_t)((raw[4] << 8) | raw[5]);

	gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
	gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
	gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

	out->ax = (float)ax_raw * ACCEL_SCALE;
	out->ay = (float)ay_raw * ACCEL_SCALE;
	out->az = (float)az_raw * ACCEL_SCALE;

	out->gx = (float)gx_raw * GYRO_SCALE;
	out->gy = (float)gy_raw * GYRO_SCALE;
	out->gz = (float)gz_raw * GYRO_SCALE;

	out->timestamp_us = micros();

	return 0;
}
