#include "control.h"
#include "config.h"
#include "../utils/math_utils.h"

#include <math.h>

static float saturate(float x, float min_val, float max_val)
{
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

void init_controller(AttitudeController *ctrl)
{
    ctrl->Kp_theta = PID_ROLL_KP;
    ctrl->Ki_theta = PID_ROLL_KI;

    ctrl->Kp_phi = PID_PITCH_KP;
    ctrl->Ki_phi = PID_PITCH_KI;

    ctrl->Kq_eta = KQ_ETA;
    ctrl->Kp_phi_xi = KP_PHI_XI;
    ctrl->Ktheta_eta = KTHETA_ETA;

    ctrl->int_theta = 0.0f;
    ctrl->int_phi = 0.0f;

    ctrl->elevator_min = -0.5f;
    ctrl->elevator_max =  0.5f;

    ctrl->aileron_min = -0.5f;
    ctrl->aileron_max =  0.5f;

    ctrl->rudder_trim = 0.0f;
}

ControlOutput attitude_controller_update(
    AttitudeController *ctrl,
    float roll,
    float pitch,
    float roll_d,
    float pitch_d,
    float throttle_d,
    float dt
)
{
    ControlOutput out;

    float theta_error = pitch_d - pitch;

    float elevator_unsat =
        ctrl->Kp_theta * theta_error +
        ctrl->Ki_theta * ctrl->int_theta;

    float elevator_sat = saturate(
        elevator_unsat,
        ctrl->elevator_min,
        ctrl->elevator_max
    );

    if ((elevator_unsat == elevator_sat) ||
        (elevator_unsat > ctrl->elevator_max && theta_error < 0.0f) ||
        (elevator_unsat < ctrl->elevator_min && theta_error > 0.0f))
    {
        ctrl->int_theta += theta_error * dt;
    }

    elevator_unsat =
        ctrl->Kp_theta * theta_error +
        ctrl->Ki_theta * ctrl->int_theta;

    out.elevator = saturate(
        elevator_unsat,
        ctrl->elevator_min,
        ctrl->elevator_max
    );

    float phi_error = roll_d - roll;

    float aileron_unsat =
        ctrl->Kp_phi * phi_error +
        ctrl->Ki_phi * ctrl->int_phi;

    float aileron_sat = saturate(
        aileron_unsat,
        ctrl->aileron_min,
        ctrl->aileron_max
    );

    if ((aileron_unsat == aileron_sat) ||
        (aileron_unsat > ctrl->aileron_max && phi_error < 0.0f) ||
        (aileron_unsat < ctrl->aileron_min && phi_error > 0.0f))
    {
        ctrl->int_phi += phi_error * dt;
    }

    aileron_unsat =
        ctrl->Kp_phi * phi_error +
        ctrl->Ki_phi * ctrl->int_phi;

    out.aileron = saturate(
        aileron_unsat,
        ctrl->aileron_min,
        ctrl->aileron_max
    );

    out.rudder = ctrl->rudder_trim;
    out.throttle = throttle_d;

    return out;
}
