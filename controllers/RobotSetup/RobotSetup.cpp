#include "RobotSetup.hpp"

void Corgi::robot_initialize(Supervisor *supervisor){
    set_leg_mass(supervisor, 1);

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
    // this->mod_A->force_sensor = supervisor->getTouchSensor("force_lf");
    // this->mod_A->force_sensor_node = supervisor->getFromDef("LF_force_sensor");

    this->mod_B->right_motor = supervisor->getMotor("rf_left_motor");
    this->mod_B->left_motor = supervisor->getMotor("rf_right_motor");
    this->mod_B->dist_sensor = supervisor->getDistanceSensor("dst_rf");
    // this->mod_B->force_sensor = supervisor->getTouchSensor("force_rf");
    // this->mod_B->force_sensor_node = supervisor->getFromDef("RF_force_sensor");

    this->mod_C->right_motor = supervisor->getMotor("rh_left_motor");
    this->mod_C->left_motor = supervisor->getMotor("rh_right_motor");
    this->mod_C->dist_sensor = supervisor->getDistanceSensor("dst_rh");
    // this->mod_C->force_sensor = supervisor->getTouchSensor("force_rh");
    // this->mod_C->force_sensor_node = supervisor->getFromDef("RH_force_sensor");

    this->mod_D->right_motor = supervisor->getMotor("lh_right_motor");
    this->mod_D->left_motor = supervisor->getMotor("lh_left_motor");
    this->mod_D->dist_sensor = supervisor->getDistanceSensor("dst_lh");
    // this->mod_D->force_sensor = supervisor->getTouchSensor("force_lh");
    // this->mod_D->force_sensor_node = supervisor->getFromDef("LH_force_sensor");

    for (auto& mod: this->leg_mods){
        mod->right_motor->enableTorqueFeedback(1);
        mod->left_motor->enableTorqueFeedback(1);

        double kp = 30;
        double ki = 0;
        double kd = 0;
        mod->right_motor->setControlPID(kp, ki, kd);
        mod->left_motor->setControlPID(kp, ki, kd);

        mod->right_encoder = mod->right_motor->getPositionSensor();
        mod->right_encoder->enable(1);
        mod->left_encoder = mod->left_motor->getPositionSensor();
        mod->left_encoder->enable(1);

        mod->dist_sensor->enable(1);
        // mod->force_sensor->enable(1);
    }
}


void Corgi::set_leg_mass(Supervisor *supervisor, int enable){
    if (!enable){
        supervisor->getFromDef("LF_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LF_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);

        supervisor->getFromDef("RF_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RF_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        
        supervisor->getFromDef("RH_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("RH_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);

        supervisor->getFromDef("LH_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
        supervisor->getFromDef("LH_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.001);
    }
    else{
        supervisor->getFromDef("LF_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("LF_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("LF_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("LF_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("LF_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("LF_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("LF_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("LF_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("LF_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.047);
        supervisor->getFromDef("LF_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.046);

        supervisor->getFromDef("RF_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("RF_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("RF_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("RF_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("RF_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("RF_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("RF_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("RF_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("RF_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.047);
        supervisor->getFromDef("RF_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.046);

        supervisor->getFromDef("RH_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("RH_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("RH_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("RH_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("RH_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("RH_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("RH_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("RH_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("RH_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.047);
        supervisor->getFromDef("RH_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.046);

        supervisor->getFromDef("LH_rl4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("LH_ll4")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0145704);
        supervisor->getFromDef("LH_rl5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("LH_ll5l6")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.0364259);
        supervisor->getFromDef("LH_rupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("LH_lupframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.112321);
        supervisor->getFromDef("LH_rdownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("LH_ldownframe")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.052707);
        supervisor->getFromDef("LH_rmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.047);
        supervisor->getFromDef("LH_lmotorbar")->getField("physics")->getSFNode()->getField("mass")->setSFFloat(0.046);
    }
}


void Corgi::update_robot_param(){
    this->pose_pos = this->robot_node->getCenterOfMass();
    this->pose_ori = this->gyro->getQuaternion();
    this->twist_lin = this->robot_node->getVelocity();
    this->twist_ang = this->ang_vel->getValues();
    this->acc_pos = this->imu->getValues();
}


void LegModule::update_leg_param(){
    this->right_motor_last_position = this->right_motor_position;
    this->left_motor_last_position = this->left_motor_position;
    this->right_motor_position = this->right_encoder->getValue();
    this->left_motor_position = this->left_encoder->getValue();
    this->right_motor_velocity = this->right_motor_position - this->right_motor_last_position;
    this->left_motor_velocity = this->left_motor_position - this->left_motor_last_position;
    this->right_motor_torque = this->right_motor->getTorqueFeedback();
    this->left_motor_torque = this->left_motor->getTorqueFeedback();

    // const double *rotation = this->force_sensor_node->getOrientation();
    // this->force = Eigen::Vector3d(this->force_sensor->getValues());

    // Eigen::Matrix3d rot_matrix;
    // rot_matrix << rotation[0], rotation[1], rotation[2],
    //               rotation[3], rotation[4], rotation[5],
    //               rotation[6], rotation[7], rotation[8];

    // this->force = (rot_matrix*this->force).transpose();
}


void LegModule::set_leg_position(double right_phi_cmd, double left_phi_cmd){

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
