#include "ahrs.h"
#include "config.h"

#include <math.h>
#include <stddef.h>

static attitude_t s_attitude;

int ahrs_init(void)
{
    s_attitude.roll  = 0.0f;
    s_attitude.pitch = 0.0f;
    s_attitude.yaw   = 0.0f;

    return 0;
}

void ahrs_update(const imu_data_t *imu,
                 float dt)
{
    float accel_roll;
    float accel_pitch;

    if ((imu == NULL) || (dt <= 0.0f)) {
        return;
    }

    // Accelerometer angle estimate
    accel_roll = atan2f(imu->ay, imu->az);
    accel_pitch = atan2f(-imu->ax,
               	   	   	 sqrtf((imu->ay * imu->ay) +
                         (imu->az * imu->az)));

    // Gyro integration
    s_attitude.roll += imu->gx * dt;
    s_attitude.pitch += imu->gy * dt;
    s_attitude.yaw += imu->gz * dt;

    // Complementary filter fusion
    s_attitude.roll = (AHRS_ALPHA * s_attitude.roll) + ((1.0f - AHRS_ALPHA) * accel_roll);
    s_attitude.pitch = (AHRS_ALPHA * s_attitude.pitch) + ((1.0f - AHRS_ALPHA) * accel_pitch);
}

void ahrs_get_attitude(attitude_t *att)
{
    if (att == NULL) {
        return;
    }

    *att = s_attitude;
}
