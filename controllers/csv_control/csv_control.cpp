#include "RobotSetup.hpp"
#include "tools.hpp"

#define TIME_STEP 1

using namespace webots;

int main(int argc, char **argv) {

    // read csv file
    std::string arg = argv[1];
    std::istringstream iss(arg);
    std::string output_filename, input_filename;
    vector<vector<double>> input_data;

    if (!(iss >> output_filename >> input_filename)) {
        std::cerr << "Error parsing input and output filenames" << std::endl;
        return 1;
    }

    if (! (output_filename.length() >= 4 && output_filename.substr(output_filename.length() - 4) == ".csv")) {
        output_filename += ".csv";
    }

    if (! (input_filename.length() >= 4 && input_filename.substr(input_filename.length() - 4) == ".csv")) {
        input_filename += ".csv";
    }

    std::cout << "Output file: " << output_filename << std::endl
              << "Input file: " << input_filename << std::endl;

    input_data = read_csv(input_filename);
    
    std::vector<std::vector<std::string>> output_data;
    std::vector<std::string> row;
    std::ostringstream oss;
    oss << "A_phi_r,A_phi_l,A_trq_r,A_trq_l,B_phi_r,B_phi_l,B_trq_r,B_trq_l,C_phi_r,C_phi_l,C_trq_r,C_trq_l,D_phi_r,D_phi_l,D_trq_r,D_trq_l";
    row.push_back(oss.str());
    output_data.push_back(row);

    // Setup the robot
    Supervisor *supervisor = new Supervisor();

    Corgi corgi;

    corgi.robot_initialize(supervisor);

    supervisor->step(1000);

    int loop_counter = 0;
    while (supervisor->step(TIME_STEP) != -1) {
        printf("= = = Loop Count %d = = =\n", loop_counter);
        std::vector<std::string> row;

        int csv_idx = 0;
        for (auto& mod: corgi.leg_mods){
            double right_phi_cmd = input_data[loop_counter][csv_idx];
            double left_phi_cmd = input_data[loop_counter][csv_idx+1];
            
            mod->setLegPosition(right_phi_cmd, left_phi_cmd);
            mod->update_leg_param();

            std::ostringstream oss;
            oss << mod->right_motor_position << "," << mod->left_motor_position << "," << mod->right_motor_torque << "," << mod->left_motor_torque;
            row.push_back(oss.str());

            csv_idx += 2;
        }

        output_data.push_back(row);

        loop_counter++;
        
        if (loop_counter == input_data.size()) break;
    };

    write_csv(output_filename, output_data);

    return 0;
}
