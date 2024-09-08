#include "RobotSetup.hpp"
#include "Tools.hpp"

vector<vector<double>> read_csv(string input_filename){
    vector<vector<double>> data;
    ifstream input_file(input_filename);
    string line;

    if (!input_file.is_open()) {
        cerr << "Could not open the input file: " << input_filename << endl;
        return data;
    }

    while (getline(input_file, line)) {
        istringstream iss(line);
        vector<double> row;
        string value;
    
        while (getline(iss, value, ',')) {
            try { row.push_back(stod(value)); } 
            catch (const invalid_argument& ia) { cerr << "Invalid argument: " << ia.what() << '\n'; }
        }
        
        if (row.size() >= 8) { data.push_back(row); } 
        else { cerr << "Row with incorrect number of elements encountered.\n"; }
    }

    input_file.close();

    return data;
}


void write_csv(const std::string& output_filename, const std::vector<std::vector<std::string>>& data) {
    std::ofstream output_file(output_filename);
    
    if (!output_file.is_open()) {
        std::cerr << "Could not open the output file: " << output_filename << std::endl;
        return;
    }

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            output_file << row[i];
            if (i != row.size() - 1) output_file << ",";
        }
        output_file << "\n";
    }

    output_file.close();
    std::cout << "Data successfully written to " << output_filename << std::endl;
}


std::vector<std::string> get_output_header(){
    std::vector<std::string> row;
    std::ostringstream oss;

    oss << "time,"
        << "A_phi_r,A_phi_l,A_trq_r,A_trq_l,A_dist,"
        << "B_phi_r,B_phi_l,B_trq_r,B_trq_l,B_dist,"
        << "C_phi_r,C_phi_l,C_trq_r,C_trq_l,C_dist,"
        << "D_phi_r,D_phi_l,D_trq_r,D_trq_l,D_dist,"
        << "pos_x,pos_y,pos_z,"
        << "ori_x,ori_y,ori_z,ori_w,"
        << "vel_x,vel_y,vel_z,"
        << "vel_rx,vel_ry,vel_rz,"
        << "acc_x,acc_y,acc_z";

    row.push_back(oss.str());

    return row;
}


std::vector<std::string> get_output_data(Supervisor *supervisor, Corgi corgi){
    std::vector<std::string> row;
    std::ostringstream oss;

    oss << supervisor->getTime() << ",";

    for (auto& mod: corgi.leg_mods){
        oss << mod->right_motor_position << "," << mod->left_motor_position << "," 
            << mod->right_motor_torque << "," << mod->left_motor_torque << "," 
            << mod->dist_sensor->getValue() * 0.62 / 1000. + 0.055 << ",";
    }

    oss << corgi.pose_pos[0] << "," << corgi.pose_pos[1] << "," << corgi.pose_pos[2] << ","
        << corgi.pose_ori[0] << "," << corgi.pose_ori[1] << "," << corgi.pose_ori[2] << "," << corgi.pose_ori[3] << ","
        << corgi.twist_lin[0] << "," << corgi.twist_lin[1] << "," << corgi.twist_lin[2] << ","
        << corgi.twist_ang[0] << "," << corgi.twist_ang[1] << "," << corgi.twist_ang[2] << ","
        << corgi.acc_pos[0] << "," << corgi.acc_pos[1] << "," << corgi.acc_pos[2]-9.81;

    row.push_back(oss.str());
    
    return row;
}