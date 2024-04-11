#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include "tools.h"

#define TIME_STEP 1
using namespace webots;

template<size_t n>
Eigen::Vector<double, n> random_vector() {
    std::random_device rd;
    std::mt19937 gen(rd());  //here you could also set a seed
    std::uniform_real_distribution<double> dis(-1, 1);
    Eigen::Vector<double, n> V = Eigen::Vector<double, n>().NullaryExpr([&](){return dis(gen);});
    return V;
}

std::mutex mutex_;
motor_msg::MotorStamped motor_data;
void motor_data_cb(motor_msg::MotorStamped msg)
{
    mutex_.lock();
    motor_data = msg;
    mutex_.unlock();
}

int main(int argc, char* argv[])
{
    Supervisor *supervisor = new Supervisor();
    std::string argumentString = argv[1];

    std::stringstream ss(argumentString);
    std::vector<std::string> tokens;
    std::string token;

    while (ss >> token) {
        tokens.push_back(token);
        std::cout << token << "\n";
    }
    argc = tokens.size();

    if (argc < 1) { return 0;
    }
    std::ofstream file;
    std::string saved_file_name = std::string(tokens[0]);
    file.open(saved_file_name + ".csv");
    file << "Cx" << "," << "Cy" << "," << "Cz" << "," << 
    "lf.x" << "," << "lf.y" << "," << "lf.z" << "," << 
    "rf.x" << "," << "rf.y" << "," << "rf.z" << "," << 
    "rh.x" << "," << "rh.y" << "," << "rh.z" << "," << 
    "lh.x" << "," << "lh.y" << "," << "lh.z" << "," << 
    "vel.x" << "," << "vel.y" << "," << "vel.z" << "," << 
    "q.w" << "," << "q.x" << "," << "q.y" << "," << "q.z" << "," << 
    "ang_vel.x" << "," << "ang_vel.y" << "," << "ang_vel.z" << "," << 
    "accel.x" << "," << "accel.y" << "," << "accel.z" << "," << 
    "lf.dst" << "," << "rf.dst" << "," << "rh.dst" << "," << "lh.dst" << "," <<
    "lf.r" << "," << "lf.l" << "," << 
    "rf.r" << "," << "rf.l" << "," << 
    "rh.r" << "," << "rh.l" << "," <<
    "lh.r" << "," << "lh.l" << "\n";

    if (argc < 2) {
    printf("control by real time\n");
    setenv("CORE_LOCAL_IP", "127.0.0.1", 0);
    setenv("CORE_MASTER_ADDR", "127.0.0.1:10010", 0);

    core::NodeHandler nh;
    core::Ticker ticker;
    core::Subscriber<motor_msg::MotorStamped> &motor_sub = nh.subscribe<motor_msg::MotorStamped>("motor/command", 1000, motor_data_cb);
    core::Publisher<sensor_msg::IMU> &imu_pub = nh.advertise<sensor_msg::IMU>("imu");
    core::Publisher<sensor_msg::Lidar> &lidar_pub = nh.advertise<sensor_msg::Lidar>("lidar");
    core::Publisher<motor_msg::MotorStamped> &motor_pub = nh.advertise<motor_msg::MotorStamped>("motor/state");

    Node *robot_node = supervisor->getFromDef("CORGI");

    Accelerometer *imu = supervisor->getAccelerometer("imu");
    imu->enable(TIME_STEP);
    InertialUnit *gyro = supervisor->getInertialUnit("gyro");
    gyro->enable(TIME_STEP);

    TouchSensor *force_lf = supervisor->getTouchSensor("force_lf");
    force_lf->enable(TIME_STEP);
    TouchSensor *force_rf = supervisor->getTouchSensor("force_rf");
    force_rf->enable(TIME_STEP);
    TouchSensor *force_rh = supervisor->getTouchSensor("force_rh");
    force_rh->enable(TIME_STEP);
    TouchSensor *force_lh = supervisor->getTouchSensor("force_lh");
    force_lh->enable(TIME_STEP);

    Node *force_LF_node = supervisor->getFromDef("LF_force_sensor");
    Node *force_RF_node = supervisor->getFromDef("RF_force_sensor");
    Node *force_RH_node = supervisor->getFromDef("RH_force_sensor");
    Node *force_LH_node = supervisor->getFromDef("LH_force_sensor");

    Gyro *ang_vel = supervisor->getGyro("ang_vel");
    ang_vel->enable(TIME_STEP);
    Motor *lf_right_motor = supervisor->getMotor("lf_right_motor");
    PositionSensor *lf_right_motor_encoder = lf_right_motor->getPositionSensor();
    lf_right_motor_encoder->enable(1);
    Motor *lf_left_motor = supervisor->getMotor("lf_left_motor");
    PositionSensor *lf_left_motor_encoder = lf_left_motor->getPositionSensor();
    lf_left_motor_encoder->enable(1);
    Motor *lh_right_motor = supervisor->getMotor("lh_right_motor");
    PositionSensor *lh_right_motor_encoder = lh_right_motor->getPositionSensor();
    lh_right_motor_encoder->enable(1);
    Motor *lh_left_motor = supervisor->getMotor("lh_left_motor");
    PositionSensor *lh_left_motor_encoder = lh_left_motor->getPositionSensor();
    lh_left_motor_encoder->enable(1);
    Motor *rf_right_motor = supervisor->getMotor("rf_right_motor");
    PositionSensor *rf_right_motor_encoder = rf_right_motor->getPositionSensor();
    rf_right_motor_encoder->enable(1);
    Motor *rf_left_motor = supervisor->getMotor("rf_left_motor");
    PositionSensor *rf_left_motor_encoder = rf_left_motor->getPositionSensor();
    rf_left_motor_encoder->enable(1);
    Motor *rh_right_motor = supervisor->getMotor("rh_right_motor");
    PositionSensor *rh_right_motor_encoder = rh_right_motor->getPositionSensor();
    rh_right_motor_encoder->enable(1);
    Motor *rh_left_motor = supervisor->getMotor("rh_left_motor");
    PositionSensor *rh_left_motor_encoder = rh_left_motor->getPositionSensor();
    rh_left_motor_encoder->enable(1);
    robot_node->enableContactPointsTracking(TIME_STEP);

    DistanceSensor *DstLF = supervisor->getDistanceSensor("dst_lf");
    DstLF->enable(TIME_STEP);
    DistanceSensor *DstRF = supervisor->getDistanceSensor("dst_rf");
    DstRF->enable(TIME_STEP);
    DistanceSensor *DstRH = supervisor->getDistanceSensor("dst_rh");
    DstRH->enable(TIME_STEP);
    DistanceSensor *DstLH = supervisor->getDistanceSensor("dst_lh");
    DstLH->enable(TIME_STEP);

    uint64_t counter = 0;
    double last_lf_right_encoder = 0, last_lf_left_encoder = 0,
           last_rf_right_encoder = 0, last_rf_left_encoder = 0,
           last_rh_right_encoder = 0, last_rh_left_encoder = 0,
           last_lh_right_encoder = 0, last_lh_left_encoder = 0;

    double noise_accel = 3e-2;
    double noise_quat = 1e-3;
    double noise_twist = 2e-2;
    double noise_encoder = 1e-2;
    double noise_encoder_twist = 1e-2;
    double noise_dist = 3e-3;

    while (supervisor->step(TIME_STEP) != -1) 
    {
        core::spinOnce();
        mutex_.lock();
        if (motor_data.motors().size() == 8) {
            lf_right_motor->setPosition(-motor_data.motors(1).angle());
            lf_left_motor->setPosition(-motor_data.motors(0).angle());
            rf_right_motor->setPosition(motor_data.motors(3).angle());
            rf_left_motor->setPosition(motor_data.motors(2).angle());
            rh_right_motor->setPosition(motor_data.motors(5).angle());
            rh_left_motor->setPosition(motor_data.motors(4).angle());
            lh_right_motor->setPosition(-motor_data.motors(7).angle());
            lh_left_motor->setPosition(-motor_data.motors(6).angle());
        }

        Eigen::Matrix3d world_to_body = (Eigen::Quaterniond(Eigen::Vector4d(gyro->getQuaternion())).toRotationMatrix()).transpose();
        Eigen::Vector3d force_lf_body_frame = world_to_body * (Eigen::Matrix3d(force_LF_node->getOrientation())).transpose() * Eigen::Vector3d(force_lf->getValues());
        Eigen::Vector3d force_rf_body_frame = world_to_body * (Eigen::Matrix3d(force_RF_node->getOrientation())).transpose() * Eigen::Vector3d(force_rf->getValues());
        Eigen::Vector3d force_rh_body_frame = world_to_body * (Eigen::Matrix3d(force_RH_node->getOrientation())).transpose() * Eigen::Vector3d(force_rh->getValues());
        Eigen::Vector3d force_lh_body_frame = world_to_body * (Eigen::Matrix3d(force_LH_node->getOrientation())).transpose() * Eigen::Vector3d(force_lh->getValues());

        sensor_msg::Lidar lidar_msg;
        Eigen::Vector4d current_noise_dist = noise_dist * random_vector<4>();
        lidar_msg.mutable_dist()->Add(dst_function(DstLF->getValue()) + current_noise_dist(0));
        lidar_msg.mutable_dist()->Add(dst_function(DstRF->getValue()) + current_noise_dist(1));
        lidar_msg.mutable_dist()->Add(dst_function(DstRH->getValue()) + current_noise_dist(2));
        lidar_msg.mutable_dist()->Add(dst_function(DstLH->getValue()) + current_noise_dist(3));
        lidar_msg.mutable_header()->set_seq(counter / 1000);

        sensor_msg::IMU imu_msg;
        Eigen::Vector3d current_noise_accel = noise_accel * random_vector<3>();
        Eigen::Vector3d current_noise_twist = noise_twist * random_vector<3>();
        Eigen::Vector4d current_noise_quat = noise_quat * random_vector<4>();
        Eigen::Vector3d imu_accel = Eigen::Vector3d(imu->getValues()) - (Eigen::Quaterniond(Eigen::Vector4d(gyro->getQuaternion())).toRotationMatrix()).transpose() * Eigen::Vector3d(0, 0, 9.81);
        imu_msg.mutable_acceleration()->set_x(imu_accel(0) + current_noise_accel(0));
        imu_msg.mutable_acceleration()->set_y(imu_accel(1) + current_noise_accel(1));
        imu_msg.mutable_acceleration()->set_z(imu_accel(2) + current_noise_accel(2));
        imu_msg.mutable_twist()->set_x(ang_vel->getValues()[0] + current_noise_twist(0));
        imu_msg.mutable_twist()->set_y(ang_vel->getValues()[1] + current_noise_twist(1));
        imu_msg.mutable_twist()->set_z(ang_vel->getValues()[2] + current_noise_twist(2));
        imu_msg.mutable_orientation()->set_x(gyro->getQuaternion()[0] + current_noise_quat(0));
        imu_msg.mutable_orientation()->set_y(gyro->getQuaternion()[1] + current_noise_quat(1));
        imu_msg.mutable_orientation()->set_z(gyro->getQuaternion()[2] + current_noise_quat(2));
        imu_msg.mutable_orientation()->set_w(gyro->getQuaternion()[3] + current_noise_quat(3));

        motor_msg::MotorStamped motor_msg;
        motor_msg::Motor motor_r;
        motor_msg::Motor motor_l;
        Eigen::Vector2d current_noise_encoder = noise_encoder * random_vector<2>();
        Eigen::Vector2d current_noise_encoder_twist = noise_encoder_twist * random_vector<2>();

        motor_r.set_angle(lf_right_motor_encoder->getValue() + current_noise_encoder(0)); // phi R
        motor_l.set_angle(lf_left_motor_encoder->getValue() + current_noise_encoder(1)); // phi L
        motor_r.set_twist((lf_right_motor_encoder->getValue() - last_lf_right_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(0)); // velocity R
        motor_l.set_twist((lf_left_motor_encoder->getValue() - last_lf_left_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(1)); // velocity L
        motor_l.set_torque(force_lf_body_frame.norm());
        motor_r.set_torque(force_lf_body_frame.norm());

        last_lf_right_encoder = lf_right_motor_encoder->getValue();
        last_lf_left_encoder = lf_left_motor_encoder->getValue();
        motor_msg.add_motors()->CopyFrom(motor_r);
        motor_msg.add_motors()->CopyFrom(motor_l);

        motor_r.set_angle(rf_right_motor_encoder->getValue() + current_noise_encoder(0)); // phi R
        motor_l.set_angle(rf_left_motor_encoder->getValue() + current_noise_encoder(1)); // phi L
        motor_r.set_twist((rf_right_motor_encoder->getValue() - last_rf_right_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(0)); // velocity R
        motor_l.set_twist((rf_left_motor_encoder->getValue() - last_rf_left_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(1)); // velocity L
        motor_l.set_torque(force_rf_body_frame.norm());
        motor_r.set_torque(force_rf_body_frame.norm());
        last_rf_right_encoder = rf_right_motor_encoder->getValue();
        last_rf_left_encoder = rf_left_motor_encoder->getValue();
        motor_msg.add_motors()->CopyFrom(motor_l);
        motor_msg.add_motors()->CopyFrom(motor_r);

        motor_r.set_angle(rh_right_motor_encoder->getValue() + current_noise_encoder(0)); // phi R
        motor_l.set_angle(rh_left_motor_encoder->getValue() + current_noise_encoder(1)); // phi L
        motor_r.set_twist((rh_right_motor_encoder->getValue() - last_rh_right_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(0)); // velocity R
        motor_l.set_twist((rh_left_motor_encoder->getValue() - last_rh_left_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(1)); // velocity L
        motor_l.set_torque(force_rh_body_frame.norm());
        motor_r.set_torque(force_rh_body_frame.norm());
        last_rh_right_encoder = rh_right_motor_encoder->getValue();
        last_rh_left_encoder = rh_left_motor_encoder->getValue();
        motor_msg.add_motors()->CopyFrom(motor_l);
        motor_msg.add_motors()->CopyFrom(motor_r);

        motor_r.set_angle(lh_right_motor_encoder->getValue() + current_noise_encoder(0)); // phi R
        motor_l.set_angle(lh_left_motor_encoder->getValue() + current_noise_encoder(1)); // phi L
        motor_r.set_twist((lh_right_motor_encoder->getValue() - last_lh_right_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(0)); // velocity R
        motor_l.set_twist((lh_left_motor_encoder->getValue() - last_lh_left_encoder) * 1000. / (double) TIME_STEP + current_noise_encoder_twist(1)); // velocity L
        motor_l.set_torque(force_lh_body_frame.norm());
        motor_r.set_torque(force_lh_body_frame.norm());
        last_lh_right_encoder = lh_right_motor_encoder->getValue();
        last_lh_left_encoder = lh_left_motor_encoder->getValue();
        motor_msg.add_motors()->CopyFrom(motor_r);
        motor_msg.add_motors()->CopyFrom(motor_l);

        mutex_.unlock();
        filing(file, robot_node, imu, gyro, ang_vel,
        DstLF, DstRF, DstRH, DstLH, 
        lf_right_motor_encoder, lf_left_motor_encoder, 
        lh_right_motor_encoder, lh_left_motor_encoder,
        rf_right_motor_encoder, rf_left_motor_encoder,
        rh_right_motor_encoder, rh_left_motor_encoder);
        ticker.tick(counter);
        imu_pub.publish(imu_msg);
        lidar_pub.publish(lidar_msg);
        motor_pub.publish(motor_msg);
        counter += TIME_STEP * 1000;
        usleep(1000);
    }
    }
    else {
    printf("read csv file mode\n");
    Node *robot_node = supervisor->getFromDef("CORGI");

    Accelerometer *imu = supervisor->getAccelerometer("imu");
    imu->enable(TIME_STEP);
    InertialUnit *gyro = supervisor->getInertialUnit("gyro");
    gyro->enable(TIME_STEP);
    Gyro *ang_vel = supervisor->getGyro("ang_vel");
    ang_vel->enable(TIME_STEP);

    Motor *lf_right_motor = supervisor->getMotor("lf_right_motor");
    PositionSensor *lf_right_motor_encoder = lf_right_motor->getPositionSensor();
    lf_right_motor_encoder->enable(1);
    Motor *lf_left_motor = supervisor->getMotor("lf_left_motor");
    PositionSensor *lf_left_motor_encoder = lf_left_motor->getPositionSensor();
    lf_left_motor_encoder->enable(1);
    Motor *lh_right_motor = supervisor->getMotor("lh_right_motor");
    PositionSensor *lh_right_motor_encoder = lh_right_motor->getPositionSensor();
    lh_right_motor_encoder->enable(1);
    Motor *lh_left_motor = supervisor->getMotor("lh_left_motor");
    PositionSensor *lh_left_motor_encoder = lh_left_motor->getPositionSensor();
    lh_left_motor_encoder->enable(1);
    Motor *rf_right_motor = supervisor->getMotor("rf_right_motor");
    PositionSensor *rf_right_motor_encoder = rf_right_motor->getPositionSensor();
    rf_right_motor_encoder->enable(1);
    Motor *rf_left_motor = supervisor->getMotor("rf_left_motor");
    PositionSensor *rf_left_motor_encoder = rf_left_motor->getPositionSensor();
    rf_left_motor_encoder->enable(1);
    Motor *rh_right_motor = supervisor->getMotor("rh_right_motor");
    PositionSensor *rh_right_motor_encoder = rh_right_motor->getPositionSensor();
    rh_right_motor_encoder->enable(1);
    Motor *rh_left_motor = supervisor->getMotor("rh_left_motor");
    PositionSensor *rh_left_motor_encoder = rh_left_motor->getPositionSensor();
    rh_left_motor_encoder->enable(1);
    robot_node->enableContactPointsTracking(TIME_STEP);

    DistanceSensor *DstLF = supervisor->getDistanceSensor("dst_lf");
    DstLF->enable(TIME_STEP);
    DistanceSensor *DstRF = supervisor->getDistanceSensor("dst_rf");
    DstRF->enable(TIME_STEP);
    DistanceSensor *DstRH = supervisor->getDistanceSensor("dst_rh");
    DstRH->enable(TIME_STEP);
    DistanceSensor *DstLH = supervisor->getDistanceSensor("dst_lh");
    DstLH->enable(TIME_STEP);

    TouchSensor *force_lf = supervisor->getTouchSensor("force_lf");
    force_lf->enable(TIME_STEP);
    TouchSensor *force_rf = supervisor->getTouchSensor("force_rf");
    force_rf->enable(TIME_STEP);
    TouchSensor *force_rh = supervisor->getTouchSensor("force_rh");
    force_rh->enable(TIME_STEP);
    TouchSensor *force_lh = supervisor->getTouchSensor("force_lh");
    force_lh->enable(TIME_STEP);

    Node *force_LF_node = supervisor->getFromDef("LF_force_sensor");
    Node *force_RF_node = supervisor->getFromDef("RF_force_sensor");
    Node *force_RH_node = supervisor->getFromDef("RH_force_sensor");
    Node *force_LH_node = supervisor->getFromDef("LH_force_sensor");

    printf("read file :\t%s\n", tokens[1].c_str());
    std::map<int, std::vector<double> > csv_file_content;
    int size, len; // size: column, len: row
    read_csv(std::string(tokens[1].c_str()), csv_file_content, len, size, 0);
    printf("read columns: %d\t rows: %d\n", size, len);
    int index = 0;
    while (supervisor->step(TIME_STEP) != -1) {
        Eigen::Matrix3d world_to_body = (Eigen::Quaterniond(Eigen::Vector4d(gyro->getQuaternion())).toRotationMatrix()).transpose();
        Eigen::Vector3d force_lf_body_frame = world_to_body * (Eigen::Matrix3d(force_LF_node->getOrientation())).transpose() * Eigen::Vector3d(force_lf->getValues());
        Eigen::Vector3d force_rf_body_frame = world_to_body * (Eigen::Matrix3d(force_RF_node->getOrientation())).transpose() * Eigen::Vector3d(force_rf->getValues());
        Eigen::Vector3d force_rh_body_frame = world_to_body * (Eigen::Matrix3d(force_RH_node->getOrientation())).transpose() * Eigen::Vector3d(force_rh->getValues());
        Eigen::Vector3d force_lh_body_frame = world_to_body * (Eigen::Matrix3d(force_LH_node->getOrientation())).transpose() * Eigen::Vector3d(force_lh->getValues());

        Eigen::Vector3d imu_accel = Eigen::Vector3d(imu->getValues());
        printf("force lf : x %f, y %f, z %f\n", force_lf_body_frame(0), force_lf_body_frame(1), force_lf_body_frame(2));
        printf("force rf : x %f, y %f, z %f\n", force_rf_body_frame(0), force_rf_body_frame(1), force_rf_body_frame(2));
        printf("force rh : x %f, y %f, z %f\n", force_rh_body_frame(0), force_rh_body_frame(1), force_rh_body_frame(2));
        printf("force lh : x %f, y %f, z %f\n", force_lh_body_frame(0), force_lh_body_frame(1), force_lh_body_frame(2));
        Eigen::Vector3d total_force = force_lf_body_frame + force_rf_body_frame + force_rh_body_frame + force_lh_body_frame;
        printf("force : x %f, y %f, z %f\n", total_force(0), total_force(1), total_force(2));
        printf("accel : x %f, y %f, z %f\n", imu_accel(0), imu_accel(1), imu_accel(2));

        lf_right_motor->setPosition(csv_file_content[0][index]);
        lf_left_motor->setPosition(csv_file_content[1][index]);
        rf_right_motor->setPosition(csv_file_content[3][index]);
        rf_left_motor->setPosition(csv_file_content[2][index]);
        rh_right_motor->setPosition(csv_file_content[5][index]);
        rh_left_motor->setPosition(csv_file_content[4][index]);
        lh_right_motor->setPosition(csv_file_content[6][index]);
        lh_left_motor->setPosition(csv_file_content[7][index]);
        filing(file, robot_node, imu, gyro, ang_vel,
        DstLF, DstRF, DstRH, DstLH, 
        lf_right_motor_encoder, lf_left_motor_encoder, 
        lh_right_motor_encoder, lh_left_motor_encoder,
        rf_right_motor_encoder, rf_left_motor_encoder,
        rh_right_motor_encoder, rh_left_motor_encoder);
        if (index >= len - 1) continue;
        index ++;
    }
    }
    return 0;
}