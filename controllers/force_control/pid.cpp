#include "pid.hpp"

PID_controller::PID_controller(double kp_, double ki_, double kd_, double T_, double err_sum_limit)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    dT = T_;
    err = 0;
    prev_err = 0;
    err_sum = 0;
    err_dev = 0;
    err_sum_lim = err_sum_limit;
}

double PID_controller::tracking(double setpoint, double state)
{
    // printf("= = = In Tracking = = =\n");
    // printf("setpoint, state: [%lf, %lf]\n", setpoint, state);

    err = setpoint - state;
    err_sum += dT * err;
    err_sum = saturation(err_sum, err_sum_lim, -err_sum_lim);

    err_dev = (err - prev_err) / dT;
    // err_dev = dev_lpf.update(err_dev);
    prev_err = err;

    // printf("kp, err, ki, err_sum, kd, err_dev: %lf, %lf, %lf, %lf, %lf, %lf\n", kp, err, ki, err_sum, kd, err_dev);

    double output = 0;
    if (dev_init)
    {
        output = kp * err + ki * err_sum + kd * err_dev;
    }
    else
    {
        output = kp * err + ki * err_sum;
    }
    dev_init = 1;

    return output;
}

double PID_controller::saturation(double input, double upper_bound, double lower_bound)
{
    double out = input;
    if (input > upper_bound)
        out = upper_bound;
    else if (input < lower_bound)
        out = lower_bound;
    return out;
}
