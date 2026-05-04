#include "servo.h"
#include "../config.h"
#include "../utils/math_utils.h"

#include "stm32f4xx_hal.h"
#include "tim.h"   /* declares extern TIM_HandleTypeDef PWM_TIM_HANDLE */

const float us_per_deg = (SERVO_US_MAX - SERVO_US_MIN) / 180.0f;

static inline uint32_t angle_to_us(float deg)
{
    deg = constrainf(deg, -20.0f, 20.0f);
    float us = SERVO_US_MID + deg * us_per_deg;

    return (uint32_t)us;
}

/*
static inline uint32_t norm_to_us(float n)
{
    n = constrainf(n, -1.0f, 1.0f);
    int32_t us = (int32_t)(SERVO_US_MID + n * (int32_t)(SERVO_US_MAX - SERVO_US_MID));
    return (uint32_t)constraini(us, (int32_t)SERVO_US_MIN, (int32_t)SERVO_US_MAX);
}
*/

void pwm_init(void)
{
    HAL_TIM_PWM_Start(&PWM_TIM_HANDLE, PWM_CH_LEFT);
    HAL_TIM_PWM_Start(&PWM_TIM_HANDLE, PWM_CH_RIGHT);
    pwm_failsafe();
}

void pwm_write(const elevon_cmd_t *cmd)
{
    uint32_t us_left_elevon = angle_to_us(cmd->left);
    uint32_t us_right_elevon = angle_to_us(cmd->right);
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_LEFT,  us_lefts_elevon);
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_RIGHT, us_right_elevon);
}

void pwm_failsafe(void)
{
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_LEFT,  SERVO_US_MID);
    __HAL_TIM_SET_COMPARE(&PWM_TIM_HANDLE, PWM_CH_RIGHT, SERVO_US_MID);
}

void elevon_mixing(rc_cmd_t *cmd)
{

}
