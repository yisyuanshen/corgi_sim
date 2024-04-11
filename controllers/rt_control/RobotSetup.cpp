#include "RobotSetup.hpp"

void Corgi::robot_initialize(Supervisor *supervisor){
    this->robot_node = supervisor->getFromDef("CORGI");
    this->robot_node->enableContactPointsTracking(1);
    this->imu = supervisor->getAccelerometer("imu");
    this->imu->enable(1);
    this->gyro = supervisor->getInertialUnit("gyro");
    this->gyro->enable(1);
    this->ang_vel = supervisor->getGyro("ang_vel");
    this->ang_vel->enable(1);
    
    this->mod_A->right_motor = supervisor->getMotor("lf_right_motor");
    this->mod_A->left_motor = supervisor->getMotor("lf_left_motor");
    this->mod_A->dist_sensor = supervisor->getDistanceSensor("dst_lf");
    this->mod_A->force_sensor = supervisor->getTouchSensor("force_lf");

    this->mod_B->right_motor = supervisor->getMotor("rf_left_motor");
    this->mod_B->left_motor = supervisor->getMotor("rf_right_motor");
    this->mod_B->dist_sensor = supervisor->getDistanceSensor("dst_rf");
    this->mod_B->force_sensor = supervisor->getTouchSensor("force_rf");
    
    this->mod_C->right_motor = supervisor->getMotor("rh_left_motor");
    this->mod_C->left_motor = supervisor->getMotor("rh_right_motor");
    this->mod_C->dist_sensor = supervisor->getDistanceSensor("dst_rh");
    this->mod_C->force_sensor = supervisor->getTouchSensor("force_rh");

    this->mod_D->right_motor = supervisor->getMotor("lh_right_motor");
    this->mod_D->left_motor = supervisor->getMotor("lh_left_motor");
    this->mod_D->dist_sensor = supervisor->getDistanceSensor("dst_lh");
    this->mod_D->force_sensor = supervisor->getTouchSensor("force_lh");

    for (auto& mod: this->leg_mods){
        mod->right_encoder = mod->right_motor->getPositionSensor();
        mod->right_encoder->enable(1);
        mod->left_encoder = mod->left_motor->getPositionSensor();
        mod->left_encoder->enable(1);

        mod->dist_sensor->enable(1);
        mod->force_sensor->enable(1);
    }
}

void LegModule::setLegPosition(double right_phi_cmd, double left_phi_cmd){

    std::vector<double> phi_diff = {fmod(right_phi_cmd - this->right_encoder->getValue(), 2*PI),
                                    fmod(left_phi_cmd - this->left_encoder->getValue(), 2*PI)};

    if (phi_diff[0] > PI){ phi_diff[0] -= 2*PI; }
    else if (phi_diff[0] < -PI){ phi_diff[0] += 2*PI; }

    if (phi_diff[1] > PI){ phi_diff[1] -= 2*PI; }
    else if (phi_diff[1] < -PI){ phi_diff[1] += 2*PI; }
    
    std::vector<double> phi_cmd = {this->right_encoder->getValue() + phi_diff[0],
                                   this->left_encoder->getValue() + phi_diff[1]};

    this->right_motor->setPosition(phi_cmd[0]);
    this->left_motor->setPosition(phi_cmd[1]);
}