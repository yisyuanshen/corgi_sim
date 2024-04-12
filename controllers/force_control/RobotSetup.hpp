#ifndef ROBOTSETUP_HPP
#define ROBOTSETUP_HPP

#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/DistanceSensor.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <cmath>

#include "ForceTracking.hpp"

#define PI 3.1415926

using namespace webots;


class LegModule{
    public:
        Motor *right_motor;
        Motor *left_motor;
        PositionSensor *right_encoder;
        PositionSensor *left_encoder;
        DistanceSensor *dist_sensor;
        TouchSensor *force_sensor;
        Node *force_sensor_node;
        ForceTracker force_tracker;

        double right_motor_position = 0.0;
        double right_motor_last_position = 0.0;
        double right_motor_velocity = 0.0;
        double right_motor_torque = 0.0;
        double left_motor_position = 0.0;
        double left_motor_last_position = 0.0;
        double left_motor_velocity = 0.0;
        double left_motor_torque = 0.0;

        Eigen::Vector3d force;

        void setLegPosition(double right_phi_cmd, double left_phi_cmd);
        void update_leg_param();
};


class Corgi{
    public:
        Node *robot_node;
        Accelerometer *imu;
        InertialUnit *gyro;
        Gyro *ang_vel;
        LegModule *mod_A, *mod_B, *mod_C, *mod_D;
        
        std::vector<LegModule*> leg_mods;

        Corgi()
        : mod_A(new LegModule), 
          mod_B(new LegModule), 
          mod_C(new LegModule), 
          mod_D(new LegModule), 
          leg_mods{mod_A, mod_B, mod_C, mod_D} {}


        void robot_initialize(Supervisor *supervisor);
        void update_robot_param();
};
#endif