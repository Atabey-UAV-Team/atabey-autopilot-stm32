#include "imu.h"
#include "../config.h"
#include "../utils/time.h"
#include "../utils/math_utils.h"

#include "stm32f4xx_hal.h"
#include "i2c.h"   /* CubeMX-generated; declares extern I2C_HandleTypeDef IMU_I2C_HANDLE */

/* MPU6050 register map (subset) */
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75
#define MPU_WHOAMI_VAL       0x68

#define I2C_TIMEOUT_MS       5

/* Scale factors derived from full-scale settings in config.h.
 * MPU6050 raw is int16: ±32768 maps to ±FS. */
#define G_TO_MS2  9.80665f
static const float ACCEL_SCALE = (IMU_ACCEL_FS_G * G_TO_MS2) / 32768.0f;
static const float GYRO_SCALE  = (IMU_GYRO_FS_DPS * (M_PI / 180.0f)) / 32768.0f;

static HAL_StatusTypeDef wr(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&IMU_I2C_HANDLE, MPU6050_I2C_ADDR,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT_MS);
}

int imu_init(void)
{
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, MPU6050_I2C_ADDR,
                         MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         &who, 1, I2C_TIMEOUT_MS) != HAL_OK) return -1;
    if (who != MPU_WHOAMI_VAL) return -2;

    /* Wake (clear sleep), select PLL with X-gyro as clock source. */
    if (wr(MPU_REG_PWR_MGMT_1,   0x01) != HAL_OK) return -3;
    /* DLPF ~44 Hz accel / 42 Hz gyro -> 1 kHz internal sample rate. */
    if (wr(MPU_REG_CONFIG,       0x03) != HAL_OK) return -3;
    /* Sample rate = 1 kHz / (1 + SMPLRT_DIV). 0 -> 1 kHz. */
    if (wr(MPU_REG_SMPLRT_DIV,   0x00) != HAL_OK) return -3;
    /* Gyro FS: 0=±250, 1=±500, 2=±1000, 3=±2000 dps. Must match config. */
    if (wr(MPU_REG_GYRO_CONFIG,  0x10) != HAL_OK) return -3;   /* ±1000 dps */
    /* Accel FS: 0=±2g, 1=±4g, 2=±8g, 3=±16g. */
    if (wr(MPU_REG_ACCEL_CONFIG, 0x10) != HAL_OK) return -3;   /* ±8g     */
    return 0;
}

int imu_read(imu_data_t *out)
{
    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, MPU6050_I2C_ADDR,
                         MPU_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                         buf, sizeof(buf), I2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature, ignored */
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    out->ax = (float)ax * ACCEL_SCALE;
    out->ay = (float)ay * ACCEL_SCALE;
    out->az = (float)az * ACCEL_SCALE;
    out->gx = (float)gx * GYRO_SCALE;
    out->gy = (float)gy * GYRO_SCALE;
    out->gz = (float)gz * GYRO_SCALE;
    out->timestamp_us = micros();
    return 0;
}
