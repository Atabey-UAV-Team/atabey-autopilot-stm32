#ifndef APP_PWM_PWM_H
#define APP_PWM_PWM_H

#include "../control/control.h"

void pwm_init(void);
void pwm_write(const elevon_cmd_t *cmd);
void pwm_failsafe(void);    /* drive both servos to neutral */

#endif /* APP_PWM_PWM_H */
