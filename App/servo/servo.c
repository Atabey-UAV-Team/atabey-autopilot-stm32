#include "servo.h"
#include "config.h"

static uint16_t servo_limit(uint16_t value)
{
    if (value < SERVO_US_MIN)
    {
        return SERVO_US_MIN;
    }

    if (value > SERVO_US_MAX)
    {
        return SERVO_US_MAX;
    }

    return value;
}

void servo_init(servo_output_t *servo,
				TIM_HandleTypeDef *htim,
				uint32_t channel)
{
    servo->htim = htim;
    servo->channel = channel;

    HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

void servo_write_us(servo_output_t *servo,
				    uint16_t pulse_us)
{
    pulse_us = servo_limit(pulse_us);

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
}
