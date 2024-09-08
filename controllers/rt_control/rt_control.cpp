#include "RobotSetup.hpp"
#include "Tools.hpp"

#include "NodeHandler.h"
#include "motor.pb.h"
#include "sensor.pb.h"
#include "robot.pb.h"
#include "force.pb.h"

#define TIME_STEP 1

using namespace webots;

std::mutex mutex_;
motor_msg::MotorStamped motor_cmd_msg;
force_msg::LegForceStamped force_cmd_msg;

int motor_msg_updated;
int force_msg_updated;

void motor_cmd(motor_msg::MotorStamped msg)
{
    mutex_.lock();
    motor_cmd_msg = msg;
    motor_msg_updated = 1;
    mutex_.unlock();
}

void force_cmd(force_msg::LegForceStamped msg)
{
    mutex_.lock();
    force_cmd_msg = msg;
    force_msg_updated = 1;
    mutex_.unlock();
}


int main(int argc, char **argv) {
    printf("control by real time\n");
    setenv("CORE_LOCAL_IP", "127.0.0.1", 0);
    setenv("CORE_MASTER_ADDR", "127.0.0.1:10010", 0);

    core::NodeHandler nh;
    core::Ticker ticker;
    core::Subscriber<motor_msg::MotorStamped> &motor_sub = nh.subscribe<motor_msg::MotorStamped>("motor/command", 1000, motor_cmd);
    core::Subscriber<force_msg::LegForceStamped> &force_sub = nh.subscribe<force_msg::LegForceStamped>("force/command", 1000, force_cmd);
    core::Publisher<motor_msg::MotorStamped> &motor_pub = nh.advertise<motor_msg::MotorStamped>("motor/state");
    core::Publisher<force_msg::LegForceStamped> &force_pub = nh.advertise<force_msg::LegForceStamped>("force/state");
    core::Publisher<robot_msg::State> &robot_pub = nh.advertise<robot_msg::State>("robot/state");

    // Setup the robot
    Supervisor *supervisor = new Supervisor();

    Corgi corgi;

    corgi.robot_initialize(supervisor);

    // Setup output file
    std::string arg = argv[1];
    std::istringstream iss(arg);
    std::string output_filename, input_filename;

    if (!(iss >> output_filename)) {
        std::cerr << "Error parsing output filename" << std::endl;
        return 1;
    }

    if (! (output_filename.length() >= 4 && output_filename.substr(output_filename.length() - 4) == ".csv")) {
        output_filename += ".csv";
    }

    std::cout << "Output file: " << output_filename << std::endl;

    std::vector<std::vector<std::string>> output_data;

    output_data.push_back(get_output_header());
    write_csv(output_filename, output_data);

    // Start
    supervisor->step(1000);

    int loop_counter = 0;
    while (supervisor->step(TIME_STEP) != -1) {
        printf("= = = Loop Count %d = = =\n", loop_counter);

        core::spinOnce();
        mutex_.lock();

        if (motor_cmd_msg.motors().size() == 8) {
            int mod_idx = 0;
            for (auto& mod: corgi.leg_mods){
                double right_phi_cmd = motor_cmd_msg.motors(2*mod_idx).angle();
                double left_phi_cmd = motor_cmd_msg.motors(2*mod_idx+1).angle();

                if (mod == corgi.mod_A || mod == corgi.mod_D){
                    swap(right_phi_cmd, left_phi_cmd);
                    right_phi_cmd *= -1;
                    left_phi_cmd *= -1;
                }

                mod->set_leg_position(right_phi_cmd, left_phi_cmd);
                mod_idx += 1;
                
                mod->update_leg_param();
            }
        }

        mutex_.unlock();

        corgi.update_robot_param();

        output_data.push_back(get_output_data(supervisor, corgi));
        write_csv(output_filename, output_data);

        ticker.tick(loop_counter*1000);

        loop_counter++;
    };

    return 0;
}
