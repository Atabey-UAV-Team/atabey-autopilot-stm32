#include "rc.h"
#include "pwm_input.h"
#include "config.h"

#include "stm32f4xx_hal.h"

#define RC_DEADBAND    0.05f
#define RC_TIMEOUT_MS  100U

volatile rc_raw_t s_raw;
volatile rc_cmd_t s_cmd;
static uint32_t s_last_update_ms = 0U;
static bool s_failsafe = true;

static float apply_deadband(float x)
{
    if (x > -RC_DEADBAND && x < RC_DEADBAND)
    {
        return 0.0f;
    }

    return x;
}

static float normalize_center(uint16_t x)
{
    float v =( (float)x - RC_US_MID) / (RC_US_MAX - RC_US_MID);

    if (v < -1.0f)
		v = -1.0f;

	if (v > 1.0f)
		v = 1.0f;

    return apply_deadband(v);
}

static float normalize_throttle(uint16_t x)
{
    float v = ( (float)x - SERVO_US_MIN) / (SERVO_US_MAX - SERVO_US_MIN);

    if (v < 0.0f)
        v = 0.0f;

    if (v > 1.0f)
        v = 1.0f;

    return v;
}

void rc_init(void)
{
    s_raw.roll_raw     = 1500U;
    s_raw.pitch_raw    = 1500U;
    s_raw.yaw_raw      = 1500U;
    s_raw.throttle_raw = 1000U;

    s_failsafe = true;
}

void rc_update(void)
{
    s_raw.roll_raw = pwm_read_us(0);
    s_raw.pitch_raw = pwm_read_us(1);
    s_raw.throttle_raw = pwm_read_us(2);
    s_raw.yaw_raw = pwm_read_us(3);

    uint32_t now = HAL_GetTick();

    if (s_raw.roll_raw < RC_VALID_MIN ||
        s_raw.roll_raw > RC_VALID_MAX ||

        s_raw.pitch_raw < RC_VALID_MIN ||
        s_raw.pitch_raw > RC_VALID_MAX ||

        s_raw.yaw_raw < RC_VALID_MIN ||
        s_raw.yaw_raw > RC_VALID_MAX ||

        s_raw.throttle_raw < RC_VALID_MIN ||
        s_raw.throttle_raw > RC_VALID_MAX)
    {
        s_failsafe = true;
        // return;
    }

    s_last_update_ms = now;
    s_failsafe = false;

    s_cmd.roll_cmd = normalize_center(s_raw.roll_raw);
    s_cmd.pitch_cmd = normalize_center(s_raw.pitch_raw);
    s_cmd.yaw_cmd = normalize_center(s_raw.yaw_raw);
    s_cmd.throttle_cmd = normalize_throttle(s_raw.throttle_raw);
}

bool rc_is_failsafe(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_last_update_ms) > RC_TIMEOUT_MS)
    {
        return true;
    }

    return s_failsafe;
}

const rc_raw_t* rc_get_raw(void)
{
    return &s_raw;
}

const rc_cmd_t* rc_get_cmd(void)
{
    return &s_cmd;
}
