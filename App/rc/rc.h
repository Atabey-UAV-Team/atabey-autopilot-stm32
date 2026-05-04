#ifndef APP_RC_RC_H
#define APP_RC_RC_H

typedef struct {
	uint16_t roll_raw;
	uint16_t pitch_raw;
	uint16_t throttle_raw;
	uint16_t yaw_raw;
} rc_raw_t;

typedef struct {
    float roll_cmd;
    float pitch_cmd;
    float throttle_cmd;
    float yaw_cmd;
} rc_cmd_t;

void pwm_input_init(void);
uint16_t pwm_read_us(uint8_t channel);

void rc_init(void);
void rc_update(void);

const rc_raw_t* rc_get_raw(void);
const rc_cmd_t* rc_get_cmd(void);

bool rc_is_failsafe(void);

#endif
