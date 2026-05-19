#ifndef APP_SERVO_SERVO_H
#define APP_SERVO_SERVO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} servo_output_t;

void servo_init(servo_output_t *servo,
			    TIM_HandleTypeDef *htim,
			    uint32_t channel);

void servo_write_us(servo_output_t *servo,
					uint16_t pulse_us);

#endif
