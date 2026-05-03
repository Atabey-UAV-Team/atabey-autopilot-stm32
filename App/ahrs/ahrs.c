#include "ahrs.h"
#include "../config.h"
#include "../utils/math_utils.h"

#include <math.h>

/*
 * 2-state Kalman per axis (roll, pitch): state = [angle, gyro_bias].
 *   Predict:  angle += (gyro - bias) * dt
 *   Measure:  angle from accelerometer
 * Yaw is integrated from gyro only (no magnetometer in this build).
 *
 * dt is taken from IMU timestamps so jitter in the loop is absorbed.
 */

typedef struct {
    float angle;
    float bias;
    float P[2][2];
} kalman1_t;

static kalman1_t k_roll;
static kalman1_t k_pitch;

static uint32_t  s_last_us;
static int       s_have_prev;

static void k_init(kalman1_t *k)
{
    k->angle = 0.0f;
    k->bias  = 0.0f;
    k->P[0][0] = 0.0f; k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f; k->P[1][1] = 0.0f;
}

static float k_step(kalman1_t *k, float new_angle, float new_rate, float dt)
{
    /* Predict */
    float rate = new_rate - k->bias;
    k->angle += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + AHRS_Q_ANGLE);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += AHRS_Q_BIAS * dt;

    /* Update */
    float S = k->P[0][0] + AHRS_R_MEAS;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    float y = new_angle - k->angle;
    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00 = k->P[0][0];
    float P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}

void ahrs_init(void)
{
    k_init(&k_roll);
    k_init(&k_pitch);
    s_have_prev = 0;
    s_last_us   = 0;
}

void ahrs_update(const imu_data_t *imu, ahrs_state_t *out)
{
    /* Accel-derived angles (radians). Aerospace convention:
     *   roll  = atan2(ay, az)
     *   pitch = atan2(-ax, sqrt(ay^2 + az^2))
     */
    float roll_acc  = atan2f(imu->ay, imu->az);
    float pitch_acc = atan2f(-imu->ax,
                             sqrtf(imu->ay * imu->ay + imu->az * imu->az));

    if (!s_have_prev) {
        k_roll.angle  = roll_acc;
        k_pitch.angle = pitch_acc;
        s_last_us     = imu->timestamp_us;
        s_have_prev   = 1;

        out->roll       = roll_acc;
        out->pitch      = pitch_acc;
        out->yaw        = 0.0f;
        out->roll_rate  = imu->gx;
        out->pitch_rate = imu->gy;
        out->yaw_rate   = imu->gz;
        return;
    }

    /* dt from timestamps; uint32 µs subtraction handles wrap. */
    uint32_t dt_us = imu->timestamp_us - s_last_us;
    s_last_us = imu->timestamp_us;

    /* Clamp dt to a sane range to survive scheduler hiccups. */
    if (dt_us == 0)        dt_us = 1;
    if (dt_us > 50000U)    dt_us = 50000U;   /* >20 Hz floor */
    float dt = (float)dt_us * 1e-6f;

    float roll  = k_step(&k_roll,  roll_acc,  imu->gx, dt);
    float pitch = k_step(&k_pitch, pitch_acc, imu->gy, dt);

    /* Yaw: gyro integration only. Wrap to [-pi, pi]. */
    out->yaw += imu->gz * dt;
    if (out->yaw >  M_PI) out->yaw -= 2.0f * M_PI;
    if (out->yaw < -M_PI) out->yaw += 2.0f * M_PI;

    out->roll       = roll;
    out->pitch      = pitch;
    out->roll_rate  = imu->gx - k_roll.bias;
    out->pitch_rate = imu->gy - k_pitch.bias;
    out->yaw_rate   = imu->gz;
}
