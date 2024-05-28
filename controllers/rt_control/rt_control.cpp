#include "RobotSetup.hpp"
#include "tools.hpp"

#include "NodeHandler.h"
#include "motor.pb.h"
#include "sensor.pb.h"

#define TIME_STEP 1

using namespace webots;

std::mutex mutex_;
motor_msg::MotorStamped motor_data;
void motor_data_cb(motor_msg::MotorStamped msg)
{
    mutex_.lock();
    motor_data = msg;
    mutex_.unlock();
}

int main(int argc, char **argv) {

    printf("control by real time\n");
    setenv("CORE_LOCAL_IP", "127.0.0.1", 0);
    setenv("CORE_MASTER_ADDR", "127.0.0.1:10010", 0);

    core::NodeHandler nh;
    core::Ticker ticker;
    core::Subscriber<motor_msg::MotorStamped> &motor_sub = nh.subscribe<motor_msg::MotorStamped>("motor/command", 1000, motor_data_cb);

    // Setup the robot
    Supervisor *supervisor = new Supervisor();

    Corgi corgi;

    corgi.robot_initialize(supervisor);

    supervisor->step(1000);

    int loop_counter = 0;
    while (supervisor->step(TIME_STEP) != -1) {
        printf("= = = Loop Count %d = = =\n", loop_counter);

        core::spinOnce();
        mutex_.lock();

        if (motor_data.motors().size() == 8) {
            int mod_idx = 0;
            for (auto& mod: corgi.leg_mods){
                double right_phi_cmd = motor_data.motors(2*mod_idx).angle();
                double left_phi_cmd = motor_data.motors(2*mod_idx+1).angle();

                if (mod == corgi.mod_A || mod == corgi.mod_D){
                    swap(right_phi_cmd, left_phi_cmd);
                    right_phi_cmd *= -1;
                    left_phi_cmd *= -1;
                }

                // std::cout << right_phi_cmd << ", " << left_phi_cmd << std::endl;
                mod->setLegPosition(right_phi_cmd, left_phi_cmd);
                mod_idx += 1;
                
                mod->update_leg_param();
            }
        }

        mutex_.unlock();

        corgi.update_robot_param();

        for (auto& mod: corgi.leg_mods){
            double phi_r = mod->right_encoder->getValue();
            double phi_l = mod->left_encoder->getValue();
            double theta = (phi_r - phi_l) / 2 + 17.0 / 180 * PI;
            double beta = (phi_r + phi_l) / 2;

            printf("\nphi = (%lf, %lf), tb = (%lf, %lf), torque = (%lf, %lf), force = %lf\n",
                    phi_r, phi_l, theta, beta, mod->right_motor_torque, mod->left_motor_torque, mod->force[2]);
        }

        ticker.tick(loop_counter*1000);
        supervisor->step(1);

        loop_counter++;
    };

    return 0;
}
