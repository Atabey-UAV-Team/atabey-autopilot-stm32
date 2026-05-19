#ifndef APP_AHRS_AHRS_H
#define APP_AHRS_AHRS_H

#include "../imu/imu.h"

typedef struct {
    float roll;   /* rad */
    float pitch;  /* rad */
    float yaw;    /* rad */
} attitude_t;

int  ahrs_init(void);

void ahrs_update(const imu_data_t *imu, float dt);
void ahrs_get_attitude(attitude_t *att);

#endif /* APP_AHRS_AHRS_H */
