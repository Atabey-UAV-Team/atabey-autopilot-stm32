#ifndef APP_RC_PWM_INPUT_H
#define APP_RC_PWM_INPUT_H

#include "config.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

void pwm_input_init(TIM_HandleTypeDef *htim);

void pwm_input_capture_callback(TIM_HandleTypeDef *htim);

uint16_t pwm_read_us(uint8_t channel);

#endif
