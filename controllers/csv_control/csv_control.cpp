#include "RobotSetup.hpp"
#include "tools.hpp"

#define TIME_STEP 1

using namespace webots;

int main(int argc, char **argv) {

    // read csv file
    std::string arg = argv[1];
    std::istringstream iss(arg);
    std::string output_filename, input_filename;
    vector<vector<double>> csv_data;

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

    csv_data = read_csv(input_filename);


    // Setup the robot
    Supervisor *supervisor = new Supervisor();

    Corgi corgi;

    corgi.robot_initialize(supervisor);

    supervisor->step(1000);

    int loop_counter = 0;
    while (supervisor->step(TIME_STEP) != -1) {
        printf("= = = Loop Count %d = = =\n", loop_counter);

        int csv_idx = 0;
        for (auto& mod: corgi.leg_mods){
            double right_phi_cmd = csv_data[loop_counter][csv_idx];
            double left_phi_cmd = csv_data[loop_counter][csv_idx];
            
            mod->setLegPosition(right_phi_cmd, left_phi_cmd);
            csv_idx += 2;
        }



        loop_counter++;
        
        if (loop_counter == csv_data.size()) break;
    };

    return 0;
}
