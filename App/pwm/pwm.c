#include "pwm.h"
#include "../config.h"
#include "../utils/math_utils.h"

#include "stm32f4xx_hal.h"
#include "tim.h"   /* CubeMX-generated; declares extern TIM_HandleTypeDef PWM_TIM_HANDLE */

/*
 * Hardware PWM driven by a TIM peripheral configured for 1 µs ticks
 * (PSC = TIM_clk/1e6 - 1) and ARR = 19999 (50 Hz frame).
 * CCR value is directly the pulse width in µs.
 */

static inline uint32_t norm_to_us(float n)
{
    n = constrainf(n, -1.0f, 1.0f);
    int32_t us = (int32_t)(SERVO_US_MID + n * (int32_t)(SERVO_US_MAX - SERVO_US_MID));
    return (uint32_t)constraini(us, (int32_t)SERVO_US_MIN, (int32_t)SERVO_US_MAX);
}

void pwm_init(void)
{
    HAL_TIM_PWM_Start(&PWM_TIM_HANDLE, PWM_CH_LEFT);
    HAL_TIM_PWM_Start(&PWM_TIM_HANDLE, PWM_CH_RIGHT);
    pwm_failsafe();
}

void pwm_write(const elevon_cmd_t *cmd)
{
    uint32_t us_l = norm_to_us(cmd->left);
    uint32_t us_r = norm_to_us(cmd->right);
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_LEFT,  us_l);
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_RIGHT, us_r);
}

void pwm_failsafe(void)
{
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_LEFT,  SERVO_US_MID);
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_RIGHT, SERVO_US_MID);
}
