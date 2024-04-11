#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <numeric>
#include <map>
#include <stdexcept>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include "NodeHandler.h"
#include "motor.pb.h"
#include "sensor.pb.h"
#include <webots/DistanceSensor.hpp>
#include <webots/TouchSensor.hpp>
#include "Leg.hpp"
using namespace webots;
int group(double* pt, const double *center)
{
    double axises[4][3] = {{center[0] + 0.2, center[1] + 0.15, center[2] }, 
                            {center[0] + 0.2, center[1] - 0.15, center[2] }, 
                            {center[0] - 0.2, center[1] - 0.15, center[2] }, 
                            {center[0] - 0.2, center[1] + 0.15, center[2] }};
    double minf = 1e10;
    int group_index;
    for (int i = 0; i < 4; i++)
    {
        double dist = 0;
        for (int j = 0; j < 3; j++)
        {
            dist += (pt[j] - axises[i][j]) * (pt[j] - axises[i][j]);
        }
        if (dist < minf)
        {
            minf = dist;
            group_index = i;
        }
    }
    return group_index;
}

std::string readFileIntoString(const std::string& path) {
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        std::cerr << "Could not open the file - '"
             << path << "'" << std::endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    return ss.str();
}

void read_csv(std::string file, std::map<int, std::vector<double> > &csv_contents, int &data_len, int &data_size, int start_line) {
    std::string filename(file);
    std::string file_contents;
    char delimiter = ',';

    file_contents = readFileIntoString(filename);
    std::istringstream sstream(file_contents);
    std::string record;
    for (int i = 0; i < start_line; i ++)
    {
        std::getline(sstream, record);
    }
    int counter[2] = {0, 0};
    while (std::getline(sstream, record)) {
        counter[1] = 0;
        std::istringstream line(record);
        while (std::getline(line, record, delimiter)) {
            try
            {
                csv_contents[counter[1]].push_back(stod(record));
            }
            catch(std::invalid_argument& e)
            {
                csv_contents[counter[1]].push_back(0.0);
            }
            counter[1] += 1;
        }
        counter[0] += 1;
    }
    data_len = counter[0];
    data_size = counter[1];
}

double dst_function(double raw) {return raw * 0.62 / 1000. + 0.055;}

void filing(std::ofstream &file, Node *robot_node, Accelerometer *imu, InertialUnit *gyro, Gyro *ang_vel,
 DistanceSensor *DstLF, DistanceSensor *DstRF, DistanceSensor *DstRH, DistanceSensor *DstLH, 
 PositionSensor *lf_right_motor_encoder, PositionSensor *lf_left_motor_encoder, 
 PositionSensor *lh_right_motor_encoder, PositionSensor *lh_left_motor_encoder,
 PositionSensor *rf_right_motor_encoder, PositionSensor *rf_left_motor_encoder,
 PositionSensor *rh_right_motor_encoder, PositionSensor *rh_left_motor_encoder) {
    const double *velocities = new double[3]; velocities = robot_node->getVelocity();
    const double *position = new double[3]; position = robot_node->getCenterOfMass();
    const double *twist = new double[3]; twist = ang_vel->getValues();
    const double *accel = new double[3]; accel = imu->getValues();
    const double *quaternion = new double[4]; quaternion = gyro->getQuaternion();
    int contact_points = 0;
    ContactPoint *pts =  robot_node->getContactPoints(true, &contact_points);
    double leg_cpts[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double leg_cpts_num[4] = {0, 0, 0, 0};
    for (int i = 0; i < contact_points; i++) {
        int indice = group(pts[i].point, position);
        leg_cpts[indice][0] += pts[i].point[0];
        leg_cpts[indice][1] += pts[i].point[1];
        leg_cpts[indice][2] += pts[i].point[2];
        leg_cpts_num[indice] += 1;
    }
    for (int i = 0; i < 4; i++) {
        for (int k = 0; k < 3; k ++) {
            leg_cpts[i][k] /= leg_cpts_num[i];
        }
        // std::cout << "Contact : " << i << "\t" << leg_cpts[i][0] << "\t" << leg_cpts[i][1] << "\t" << leg_cpts[i][2] << "\n";
    }
    file << position[0] << "," << position[1] << "," << position[2] << ",";
    file << leg_cpts[0][0] << "," << leg_cpts[0][1] << "," << leg_cpts[0][2] << ",";
    file << leg_cpts[1][0] << "," << leg_cpts[1][1] << "," << leg_cpts[1][2] << ",";
    file << leg_cpts[2][0] << "," << leg_cpts[2][1] << "," << leg_cpts[2][2] << ",";
    file << leg_cpts[3][0] << "," << leg_cpts[3][1] << "," << leg_cpts[3][2] << ",";
    file << velocities[0] << "," << velocities[1] << "," << velocities[2] << "," ;
    file << quaternion[3] << "," << quaternion[0] << "," << quaternion[1] << "," << quaternion[2] << ",";
    file << twist[0] << "," << twist[1] << "," << twist[2] << "," ;
    file << accel[0] << "," << accel[1] << "," << accel[2] << "," ;
    file << dst_function(DstLF->getValue()) << "," << dst_function(DstRF->getValue()) << ",";
    file << dst_function(DstRH->getValue()) << "," << dst_function(DstLH->getValue()) << ",";
    file << lf_right_motor_encoder->getValue() << "," << lf_left_motor_encoder->getValue() << "," ;
    file << rf_right_motor_encoder->getValue() << "," << rf_left_motor_encoder->getValue() << "," ;
    file << rh_right_motor_encoder->getValue() << "," << rh_left_motor_encoder->getValue() << "," ;
    file << lh_right_motor_encoder->getValue() << "," << lh_left_motor_encoder->getValue() << "\n";

}