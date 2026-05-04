#include "rc.h"
#include <stdint.h>
#include <stdbool.h>

#define RC_MIN        1000.0f
#define RC_MAX        2000.0f
#define RC_MID        1500.0f

#define RC_DEADBAND   0.05f
#define RC_TIMEOUT_MS 100

#define PWM_CHANNEL_COUNT 4

static volatile uint32_t rise_time[PWM_CHANNEL_COUNT];
static volatile uint16_t pulse_width[PWM_CHANNEL_COUNT];

static rc_raw_t rc_raw;
static rc_cmd_t rc_cmd;

static uint32_t last_update_ms = 0;
static bool failsafe = true;

void rc_hw_read(rc_raw_t* raw)
{
    raw->roll_raw     = pwm_read_us(0);
    raw->pitch_raw    = pwm_read_us(1);
    raw->throttle_raw = pwm_read_us(2);
    raw->yaw_raw      = pwm_read_us(3);
}

uint32_t rc_get_time_ms(void)
{
    return HAL_GetTick();
}

static float apply_deadband(float x)
{
    if (x > -RC_DEADBAND && x < RC_DEADBAND)
        return 0.0f;
    return x;
}

static float normalize_center(uint16_t x)
{
    float v = ((float)x - RC_MID) / (RC_MAX - RC_MID);
    return apply_deadband(v);
}

static float normalize_throttle(uint16_t x)
{
    float v = ((float)x - RC_MIN) / (RC_MAX - RC_MIN);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

void pwm_input_init(void)
{
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    uint8_t ch = 0;
    uint32_t channel = 0;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ch = 0;
        channel = TIM_CHANNEL_1;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        ch = 1;
        channel = TIM_CHANNEL_2;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        ch = 2;
        channel = TIM_CHANNEL_3;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        ch = 3;
        channel = TIM_CHANNEL_4;
    }

    uint32_t now = HAL_TIM_ReadCapturedValue(htim, channel);

    /* Pin state kontrolü */
    if (HAL_GPIO_ReadPin(GPIOA, (1 << ch))) {
        rise_time[ch] = now;

        __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
        uint32_t diff;

        if (now >= rise_time[ch])
            diff = now - rise_time[ch];
        else
            diff = (0xFFFF - rise_time[ch]) + now;

        pulse_width[ch] = (uint16_t)diff;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

uint16_t pwm_read_us(uint8_t channel)
{
    if (channel >= PWM_CHANNEL_COUNT)
        return 1500;

    return pulse_width[channel];
}

void rc_init(void)
{
    rc_raw.roll_raw = 1500;
    rc_raw.pitch_raw = 1500;
    rc_raw.yaw_raw = 1500;
    rc_raw.throttle_raw = 1000;

    failsafe = true;
}

void rc_update(void)
{
    rc_hw_read(&rc_raw);

    uint32_t now = rc_get_time_ms();

    if (rc_raw.roll_raw < 900 || rc_raw.roll_raw > 2100 ||
    	rc_raw.pitch_raw < 900 || rc_raw.pitch_raw > 2100 ||
    	rc_raw.yaw_raw < 900 || rc_raw.yaw_raw > 2100 ||
    	rc_raw.throttle_raw < 900 || rc_raw.throttle_raw > 2100)
    {
        failsafe = true;
        return;
    }

    last_update_ms = now;
    failsafe = false;

    rc_cmd.roll_cmd     = normalize_center(rc_raw.roll_raw);
    rc_cmd.pitch_cmd    = normalize_center(rc_raw.pitch_raw);
    rc_cmd.yaw_cmd      = normalize_center(rc_raw.yaw_raw);
    rc_cmd.throttle_cmd = normalize_throttle(rc_raw.throttle_raw);
}

bool rc_is_failsafe(void)
{
    uint32_t now = rc_get_time_ms();

    if ((now - last_update_ms) > RC_TIMEOUT_MS) {
        return true;
    }

    return failsafe;
}

const rc_raw_t* rc_get_raw(void)
{
    return &rc_raw;
}

const rc_cmd_t* rc_get_cmd(void)
{
    return &rc_cmd;
}
