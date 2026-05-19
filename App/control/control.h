#ifndef CONTROL_H
#define CONTROL_H

typedef struct
{
    float elevator;
    float aileron;
    float rudder;
    float throttle;
} ControlOutput;

typedef struct
{
    float Kp_theta;
    float Ki_theta;

    float Kp_phi;
    float Ki_phi;

    float Kq_eta;
    float Kp_phi_xi;
    float Ktheta_eta;

    float int_theta;
    float int_phi;

    float elevator_min;
    float elevator_max;

    float aileron_min;
    float aileron_max;

    float rudder_trim;

} AttitudeController;

void init_controller(AttitudeController *ctrl);

ControlOutput attitude_controller_update(
    AttitudeController *ctrl,
    float roll,
    float pitch,
    float roll_d,
    float pitch_d,
    float throttle_d,
    float dt
);

#endif
