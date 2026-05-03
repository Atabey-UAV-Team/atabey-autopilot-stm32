#include "flight_app.h"
#include "config.h"

#include "utils/time.h"
#include "scheduler/scheduler.h"
#include "imu/imu.h"
#include "ahrs/ahrs.h"
#include "control/control.h"
#include "pwm/pwm.h"

/*
 * The fast loop (imu_read -> ahrs_update -> control_update -> pwm_write)
 * runs as a single scheduled task at LOOP_HZ. It is intentionally NOT split
 * across multiple tasks: separating these stages would add latency and
 * jitter between sensing and actuation, which is what controllers hate
 * most.
 */

static ahrs_state_t s_ahrs;
static uint32_t     s_last_loop_us;
static int          s_imu_ok;

/* Setpoints — for now, level flight. A future RC/RX module will write here. */
static float s_roll_sp  = 0.0f;
static float s_pitch_sp = 0.0f;

static void fast_loop(void)
{
    imu_data_t imu;
    if (imu_read(&imu) != 0) {
        s_imu_ok = 0;
        pwm_failsafe();
        return;
    }
    s_imu_ok = 1;

    ahrs_update(&imu, &s_ahrs);

    /* dt for the controller, derived from loop wall-clock. */
    uint32_t now = micros();
    uint32_t dt_us = now - s_last_loop_us;
    s_last_loop_us = now;
    if (dt_us == 0)     dt_us = 1;
    if (dt_us > 50000U) dt_us = 50000U;
    float dt = (float)dt_us * 1e-6f;

    elevon_cmd_t cmd;
    control_update(&s_ahrs, s_roll_sp, s_pitch_sp, dt, &cmd);
    pwm_write(&cmd);
}

void flight_app_init(void)
{
    time_init();
    sched_init();

    pwm_init();          /* outputs neutral immediately */
    ahrs_init();
    control_init();

    s_imu_ok = (imu_init() == 0);

    s_last_loop_us = micros();
    sched_add_task("fast", LOOP_PERIOD_US, fast_loop);
}

void flight_app_run(void)
{
    for (;;) {
        sched_run();
    }
}
