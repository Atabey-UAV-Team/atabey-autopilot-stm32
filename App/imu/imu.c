#include "imu.h"
#include "../config.h"
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

#define I2C_TIMEOUT_MS       5

/* Scale factors derived from full-scale settings in config.h.
 * MPU6050 raw is int16: ±32768 maps to ±FS. */

#define G_TO_MS2  9.80665f
static const float ACCEL_SCALE = (IMU_ACCEL_FS_G * G_TO_MS2) / 32768.0f;
static const float GYRO_SCALE  = (IMU_GYRO_FS_DPS * (M_PI / 180.0f)) / 32768.0f;

int imu_init(void)
{

}

int imu_read(imu_data_t *out)
{

}
