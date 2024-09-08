#include "RobotSetup.hpp"
#include "Tools.hpp"

#define TIME_STEP 1

using namespace webots;

int main(int argc, char **argv) {
    // process input and output files
    std::string arg = argv[1];
    std::istringstream iss(arg);
    std::string output_filename, input_filename;

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

    vector<vector<double>> input_data;

    input_data = read_csv(input_filename);

    std::vector<std::vector<std::string>> output_data;
    
    output_data.push_back(get_output_header());
    write_csv(output_filename, output_data);

    // setup the robot
    Supervisor *supervisor = new Supervisor();

    Corgi corgi;

    corgi.robot_initialize(supervisor);

    supervisor->step(1000);

    int loop_counter = 0;
    while (supervisor->step(TIME_STEP) != -1) {
        printf("= = = Loop Count %d = = =\n", loop_counter);

        int csv_idx = 0;
        for (auto& mod: corgi.leg_mods){
            double phi_r_cmd = input_data[loop_counter][csv_idx];
            double phi_l_cmd = input_data[loop_counter][csv_idx+1];
            
            mod->set_leg_position(phi_r_cmd, phi_l_cmd);
            mod->update_leg_param();

            csv_idx += 2;
        }

        corgi.update_robot_param();

        output_data.push_back(get_output_data(supervisor, corgi));
        write_csv(output_filename, output_data);

        loop_counter++;

        if (loop_counter == input_data.size()) break;
    };

    return 0;
}
