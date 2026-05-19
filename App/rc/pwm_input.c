#include "pwm_input.h"

static TIM_HandleTypeDef *s_tim = NULL;

static volatile uint32_t s_rise_time[RC_CHANNEL_COUNT];
static volatile uint16_t s_pulse_width[RC_CHANNEL_COUNT];

static GPIO_PinState pwm_read_pin(uint8_t ch)
{
    switch (ch)
    {
        case 0:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

        case 1:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

        case 2:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

        case 3:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

        default:
            return GPIO_PIN_RESET;
    }
}

static void pwm_process_channel(TIM_HandleTypeDef *htim,
								uint32_t tim_channel,
								uint8_t pwm_channel)
{
    uint32_t now = HAL_TIM_ReadCapturedValue(htim, tim_channel);

    GPIO_PinState pin = pwm_read_pin(pwm_channel);

    /*
        BOTH EDGE MODE

        HIGH -> rising sonrası
        LOW  -> falling sonrası
    */

    if (pin == GPIO_PIN_SET)
        s_rise_time[pwm_channel] = now;
    else
        s_pulse_width[pwm_channel] = (uint16_t)(now - s_rise_time[pwm_channel]);
}

void pwm_input_init(TIM_HandleTypeDef *htim)
{
    s_tim = htim;
}

void pwm_input_capture_callback(TIM_HandleTypeDef *htim)
{
	if ((s_tim == NULL) || (htim == NULL))
		return;

	if (htim->Instance != s_tim->Instance)
		return;

    switch (htim->Channel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            pwm_process_channel(htim,
								TIM_CHANNEL_1,
								0);
            break;

        case HAL_TIM_ACTIVE_CHANNEL_2:
            pwm_process_channel(htim,
								TIM_CHANNEL_2,
								1);
            break;

        case HAL_TIM_ACTIVE_CHANNEL_3:
            pwm_process_channel(htim,
								TIM_CHANNEL_3,
								2);
            break;

        case HAL_TIM_ACTIVE_CHANNEL_4:
            pwm_process_channel(htim,
								TIM_CHANNEL_4,
								3);
            break;

        default:
            break;
    }
}

uint16_t pwm_read_us(uint8_t channel)
{
    if (channel >= RC_CHANNEL_COUNT)
    {
        return 1500U;
    }

    return s_pulse_width[channel];
}
