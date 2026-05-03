#ifndef APP_IMU_IMU_H
#define APP_IMU_IMU_H

#include <stdint.h>

typedef struct {
    float    ax, ay, az;        /* m/s^2 */
    float    gx, gy, gz;        /* rad/s */
    uint32_t timestamp_us;      /* DWT µs at end of read */
} imu_data_t;

int  imu_init(void);            /* 0 = ok, <0 = error */
int  imu_read(imu_data_t *out); /* 0 = ok, <0 = bus error */

#endif /* APP_IMU_IMU_H */
