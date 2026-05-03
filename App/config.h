#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================================================
 * Flight controller configuration
 * STM32F411 Blackpill, flying wing (2 elevons)
 * ============================================================ */

/* ---- Fast loop ---- */
#define LOOP_HZ              300U
#define LOOP_PERIOD_US       (1000000U / LOOP_HZ)

/* ---- IMU (MPU6050) ---- */
#define IMU_I2C_HANDLE       hi2c1
#define MPU6050_I2C_ADDR     (0x68U << 1)        /* HAL expects 8-bit addr */
#define IMU_GYRO_FS_DPS      1000.0f             /* must match register cfg */
#define IMU_ACCEL_FS_G       8.0f                /* must match register cfg */

/* ---- PWM (servos) ---- */
#define PWM_TIM_HANDLE       htim3
#define PWM_CH_LEFT          TIM_CHANNEL_1
#define PWM_CH_RIGHT         TIM_CHANNEL_2
#define PWM_TIMER_HZ         1000000U            /* 1 MHz tick -> CCR == µs */
#define SERVO_US_MIN         1000U
#define SERVO_US_MID         1500U
#define SERVO_US_MAX         2000U

/* ---- PID gains (start conservative; tune in flight) ---- */
#define PID_ROLL_KP          0.80f
#define PID_ROLL_KI          0.10f
#define PID_ROLL_KD          0.04f
#define PID_ROLL_I_LIMIT     0.30f

#define PID_PITCH_KP         0.90f
#define PID_PITCH_KI         0.12f
#define PID_PITCH_KD         0.05f
#define PID_PITCH_I_LIMIT    0.30f

/* ---- Output limits ---- */
#define ELEVON_LIMIT         0.85f               /* max normalized elevon */
#define CONTROL_OUT_LIMIT    1.00f               /* per-axis PID output cap */

/* ---- AHRS (Kalman) ---- */
#define AHRS_Q_ANGLE         0.001f
#define AHRS_Q_BIAS          0.003f
#define AHRS_R_MEAS          0.030f

#endif /* APP_CONFIG_H */
