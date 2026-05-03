#ifndef APP_IMU_IMU_H
#define APP_IMU_IMU_H

#include <stdint.h>

typedef struct {
    float    ax, ay, az; /* m/s^2 */
    float    gx, gy, gz; /* rad/s */
    uint32_t timestamp_us;
} imu_data_t;

int  imu_init(void);
int  imu_read(imu_data_t *out);

#endif
