#ifndef APP_AHRS_AHRS_H
#define APP_AHRS_AHRS_H

#include <stdint.h>
#include "../imu/imu.h"

typedef struct {
    float roll;          /* rad, body X */
    float pitch;         /* rad, body Y */
    float yaw;           /* rad, body Z (gyro-integrated; no mag yet) */
    float roll_rate;     /* rad/s, gyro - bias */
    float pitch_rate;
    float yaw_rate;
} ahrs_state_t;

void ahrs_init(void);

/* Updates filter using IMU sample. dt is computed from imu->timestamp_us
 * vs the previous call. First call seeds state from accelerometer. */
void ahrs_update(const imu_data_t *imu, ahrs_state_t *out);

#endif /* APP_AHRS_AHRS_H */
