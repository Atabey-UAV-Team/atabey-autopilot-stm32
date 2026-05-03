#include "control.h"
#include "../config.h"
#include "../utils/math_utils.h"

typedef struct {
    float kp, ki, kd;
    float i_term;
    float prev_meas;
    float i_limit;
    int   primed;        /* derivative needs one prior sample */
} pid_t;

static pid_t roll_pid;
static pid_t pitch_pid;

static void pid_set(pid_t *p, float kp, float ki, float kd, float i_limit)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->i_term = 0.0f;
    p->prev_meas = 0.0f;
    p->i_limit = i_limit;
    p->primed = 0;
}

/* Derivative-on-measurement to avoid setpoint-step kicks. */
static float pid_step(pid_t *p, float setpoint, float meas, float dt)
{
    float err = setpoint - meas;

    p->i_term += p->ki * err * dt;
    p->i_term  = constrainf(p->i_term, -p->i_limit, p->i_limit);

    float d = 0.0f;
    if (p->primed && dt > 0.0f) {
        d = -p->kd * (meas - p->prev_meas) / dt;
    }
    p->prev_meas = meas;
    p->primed = 1;

    float u = p->kp * err + p->i_term + d;
    return constrainf(u, -CONTROL_OUT_LIMIT, CONTROL_OUT_LIMIT);
}

void control_init(void)
{
    pid_set(&roll_pid,  PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  PID_ROLL_I_LIMIT);
    pid_set(&pitch_pid, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_I_LIMIT);
}

void control_update(const ahrs_state_t *ahrs,
                    float roll_sp, float pitch_sp,
                    float dt, elevon_cmd_t *out)
{
    float u_roll  = pid_step(&roll_pid,  roll_sp,  ahrs->roll,  dt);
    float u_pitch = pid_step(&pitch_pid, pitch_sp, ahrs->pitch, dt);

    /* Elevon mixing for a flying wing.
     * Right wing-down roll is positive; nose-up pitch is positive.
     *   left  = pitch + roll
     *   right = pitch - roll  */
    float left  = u_pitch + u_roll;
    float right = u_pitch - u_roll;

    out->left  = constrainf(left,  -ELEVON_LIMIT, ELEVON_LIMIT);
    out->right = constrainf(right, -ELEVON_LIMIT, ELEVON_LIMIT);
}
