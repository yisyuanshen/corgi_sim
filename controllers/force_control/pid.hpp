#ifndef _PID_H
#define _PID_H

#include <math.h>
#include "lpf.hpp"
#include <iostream>

class PID_controller
{
  public:
    PID_controller(double kp, double ki, double kd, double T_, double err_sum_limit);
    PID_controller()
    {
    }

    double tracking(double setpoint, double state);
    double saturation(double a, double upper_bound, double lower_bound);

    double dT;
    double kp;
    double ki;
    double kd;

    double err;
    double prev_err;
    double err_sum;
    double err_dev;
    double err_sum_lim;

    int dev_init = 0;
};

#endif  // DEBUG
