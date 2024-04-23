#include "RobotSetup.hpp"
#include "tools.hpp"

#include "NodeHandler.h"
#include "motor.pb.h"
#include "sensor.pb.h"
#include "force.pb.h"

#define TIME_STEP 1

using namespace webots;

std::mutex mutex_;
motor_msg::MotorStamped motor_data;
force_msg::LegForceStamped force_data;

void motor_data_cb(motor_msg::MotorStamped msg)
{
    mutex_.lock();
    motor_data = msg;
    mutex_.unlock();
}

void force_data_cb(force_msg::LegForceStamped msg)
{
    mutex_.lock();
    force_data = msg;
    mutex_.unlock();
}

int main(int argc, char **argv) {
    // Declare the force message
    // const force_msg::LegForceStamped& force_cmd_msg;
    // force_msg::LegForceStamped& force_fb_msg;
    setenv("CORE_LOCAL_IP", "127.0.0.1", 0);
    setenv("CORE_MASTER_ADDR", "127.0.0.1:10010", 0);
    
    core::NodeHandler nh;
    core::Ticker ticker;
    core::Subscriber<motor_msg::MotorStamped> &motor_sub = nh.subscribe<motor_msg::MotorStamped>("motor/command", 1000, motor_data_cb);
    core::Subscriber<force_msg::LegForceStamped> &force_sub = nh.subscribe<force_msg::LegForceStamped>("force/force_command", 1000, force_data_cb);

    // Setup the robot
    Supervisor *supervisor = new Supervisor();

    Corgi corgi;

    corgi.robot_initialize(supervisor);

    kinematics_setup();

    // Initialize force tracker
    for (auto& mod: corgi.leg_mods){
        Eigen::Vector2d phi_(0.0, 0.0);
        Eigen::Vector2d tb_ = phi2tb(phi_);
        mod->force_tracker.initialize(tb_);
    }

    Eigen::MatrixXd force_cmd(4, 4);
    force_cmd << 0.0, -0.1, 8, 50, // x_d, y_d, f_x, f_y
                 0.0, -0.1, 8, 50,
                 0.0, -0.1, 8, 50,
                 0.0, -0.1, 8, 50;

    supervisor->step(1000);

    int loop_counter = 0;
    while (supervisor->step(TIME_STEP) != -1) {
        printf(" \n= = = Loop Count %d = = =\n", loop_counter);
        core::spinOnce();
        mutex_.lock();

        if (force_data.force().size() == 4){
            for (int i=0; i<4; i++){
                force_cmd(i, 0) = force_data.force(i).pose_x();
                force_cmd(i, 1) = force_data.force(i).pose_y();
                force_cmd(i, 2) = force_data.force(i).force_x();
                force_cmd(i, 3) = force_data.force(i).force_y();
            }
        }

        // publishMsg(motor_fb_msg);
        /*
        if (loop_counter < 3000)
            for (int i=0; i<4; i++){
                force_cmd(i, 0) = -0.06 * sin(0.01*loop_counter);
                force_cmd(i, 1) = -0.19 - 0.06 * cos(0.01*loop_counter);
            }
        if (loop_counter >= 3000)
            for (int i=0; i<4; i++){
                force_cmd(i, 0) = -0.2 * sin(0.005*loop_counter);
                force_cmd(i, 1) = -0.08 * sin(0.015*loop_counter) - 0.2 * cos(0.005*loop_counter);
            }
        */
        
        int mod_idx = 0;
        for (auto& mod: corgi.leg_mods){
            // if (force_cmd_msg.force().size() == 4) {
            if (force_cmd.rows() == 4) {
                // X_d, F_d
                double x_d = force_cmd(mod_idx, 0); // force_cmd_msg.force(mod_idx).pose_x();
                double y_d = force_cmd(mod_idx, 1); // force_cmd_msg.force(mod_idx).pose_y();
                double f_x = force_cmd(mod_idx, 2); // force_cmd_msg.force(mod_idx).force_x();
                double f_y = force_cmd(mod_idx, 3); // force_cmd_msg.force(mod_idx).force_y();

                Eigen::Vector2d X_d(x_d, y_d);
                Eigen::Vector2d F_d(f_x, f_y);

                // load param from command
                mod->force_tracker.M_d(0, 0) = 0.652; // force_cmd_msg.impedance(mod_idx).m_x();
                mod->force_tracker.M_d(1, 1) = 0.652; // force_cmd_msg.impedance(mod_idx).m_y();
                mod->force_tracker.K_0(0, 0) = 200000000; // force_cmd_msg.impedance(mod_idx).k0_x();
                mod->force_tracker.K_0(1, 1) = 200000000; // force_cmd_msg.impedance(mod_idx).k0_y();
                mod->force_tracker.D_d(0, 0) = 400; // force_cmd_msg.impedance(mod_idx).d_x();
                mod->force_tracker.D_d(1, 1) = 600; // force_cmd_msg.impedance(mod_idx).d_y();
                mod->force_tracker.force_tracker_x.kp = 1000; // force_cmd_msg.impedance(mod_idx).adaptive_kp_x();
                mod->force_tracker.force_tracker_x.ki = 0; // force_cmd_msg.impedance(mod_idx).adaptive_ki_x();
                mod->force_tracker.force_tracker_x.kd = 50; // force_cmd_msg.impedance(mod_idx).adaptive_kd_x();
                mod->force_tracker.force_tracker_x.dT = 0.001; // force_cmd_msg.impedance(mod_idx).adaptive_kd_y();
                mod->force_tracker.force_tracker_y.kp = 3000; // force_cmd_msg.impedance(mod_idx).adaptive_kp_y();
                mod->force_tracker.force_tracker_y.ki = 1800; // force_cmd_msg.impedance(mod_idx).adaptive_ki_y();
                mod->force_tracker.force_tracker_y.kd = 50; // force_cmd_msg.impedance(mod_idx).adaptive_kd_y();
                mod->force_tracker.force_tracker_y.dT = 0.001; // force_cmd_msg.impedance(mod_idx).adaptive_kd_y();

                /*
                double vel_filt_r = mod->force_tracker.vel_lpf_r.y_k;
                double vel_filt_l = mod->force_tracker.vel_lpf_l.y_k;
                double trq_filt_r = mod->force_tracker.trq_lpf_r.y_k;
                double trq_filt_l = mod->force_tracker.trq_lpf_l.y_k;
                */

                Eigen::Vector2d phi_fb(mod->right_motor_position, mod->left_motor_position);
                Eigen::Vector2d tb_fb = phi2tb(phi_fb);
                Eigen::Vector2d trq_fb(mod->right_motor_torque, mod->left_motor_torque);
                Eigen::Vector2d phi_vel(mod->right_motor_velocity, mod->left_motor_velocity);
                // trq_fb = trq_fb * 2.2; // KT compensation

                // printf("phi_fb = [%lf, %lf]\n", phi_fb[0], phi_fb[1]);
                // printf("tb_fb = [%lf, %lf]\n", tb_fb[0], tb_fb[1]);
                // printf("trq_fb = [%lf, %lf]\n", trq_fb[0], trq_fb[1]);
                // printf("phi_vel = [%lf, %lf]\n", phi_vel[0], phi_vel[1]);

                Eigen::Vector2d phi_cmd = mod->force_tracker.controlLoop(X_d, F_d, tb_fb, trq_fb, phi_vel);

                // printf("phi_cmd = [%lf, %lf]\n", phi_cmd[0], phi_cmd[1]);
                // printf("Force Measured = [%lf, %lf, %lf]\n", mod->force[0], mod->force[1], mod->force[2]);
                // printf("Z-axis Force Measured = %.2f N\n", mod->force[2]);
                printf("- - -\n");

                mod->setLegPosition(phi_cmd[0], phi_cmd[1]);

                mod->update_leg_param();

                /*
                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                mod.txdata_buffer_[0].torque_ = 0;
                mod.txdata_buffer_[1].torque_ = 0;
                mod.txdata_buffer_[0].KP_ = 90;
                mod.txdata_buffer_[0].KI_ = 0;
                mod.txdata_buffer_[0].KD_ = 1.75;
                mod.txdata_buffer_[1].KP_ = 90;
                mod.txdata_buffer_[1].KI_ = 0;
                mod.txdata_buffer_[1].KD_ = 1.75;
                */
               
                /*
                Eigen::Vector2d F_fb = mod->force_tracker.F_fb_q[0];
                // Publish Feedback force dataforceEstimation
                force_msg::LegForce f;
                Eigen::Vector2d xy_fb = fk(tb_fb);
                f.set_force_x(F_fb[0]);
                f.set_force_y(F_fb[1]);
                f.set_pose_x(xy_fb[0]);
                f.set_pose_x(xy_fb[1]);
                force_msg::Impedance imp;
                imp.set_m_x(mod->force_tracker.M_d(0, 0));
                imp.set_m_y(mod->force_tracker.M_d(1, 1));
                imp.set_k0_x(mod->force_tracker.K_ft(0, 0));
                imp.set_k0_y(mod->force_tracker.K_ft(1, 1));
                imp.set_d_x(mod->force_tracker.D_d(0, 0));
                imp.set_d_y(mod->force_tracker.D_d(1, 1));
                imp.set_adaptive_kp_x(mod->force_tracker.adaptive_kp[0]);
                imp.set_adaptive_kp_y(mod->force_tracker.adaptive_kp[1]);
                imp.set_adaptive_ki_x(mod->force_tracker.adaptive_ki[0]);
                imp.set_adaptive_ki_y(mod->force_tracker.adaptive_ki[1]);
                imp.set_adaptive_kd_x(mod->force_tracker.adaptive_kd[0]);
                imp.set_adaptive_kd_y(mod->force_tracker.adaptive_kd[1]);
                force_fb_msg.add_force()->CopyFrom(f);
                force_fb_msg.add_impedance()->CopyFrom(imp);
                */
            }
            else {
                /*
                force_msg::LegForce f;
                f.set_force_x(0);
                f.set_force_y(0);
                force_msg::Impedance imp;
                force_fb_msg.add_force()->CopyFrom(f);
                force_fb_msg.add_impedance()->CopyFrom(imp);
                */
            }

            mod_idx++;
        }
        
        mutex_.unlock();
        ticker.tick(loop_counter*1000);
        loop_counter++;
    };

    return 0;
}
