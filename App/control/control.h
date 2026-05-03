#ifndef APP_CONTROL_CONTROL_H
#define APP_CONTROL_CONTROL_H

#include "../ahrs/ahrs.h"

typedef struct {
    float left;     /* normalized [-1, 1] */
    float right;    /* normalized [-1, 1] */
} elevon_cmd_t;

void control_init(void);
/* Single-step control update.
 *   ahrs:    current attitude estimate
 *   roll_sp, pitch_sp: setpoints in radians
 *   dt:      seconds since previous update
 *   out:     mixed elevon command */
void control_update(const ahrs_state_t *ahrs,
                    float roll_sp,
					float pitch_sp,
                    float dt,
					elevon_cmd_t *out);

#endif
