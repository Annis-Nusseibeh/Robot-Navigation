
/*
 * Project: Attitude_KF
 * Description: 4 state Kalman Filter estimating roll & pitch using BNO055 9-DOF IMU
 * Author: Annis Nusseibeh
 * Date: October 3, 2020
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Range.h"
#include "kalman_filters/ahrs.h"

#include <iostream>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

std::vector<sensor_msgs::Imu> imu_buffer;
std::vector<geometry_msgs::Vector3Stamped> cal_buffer;


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_buffer.push_back(*msg);
}

void cal_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    cal_buffer.push_back(*msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "attitude_kf_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/bno055/imu", 1000, imu_callback);
    ros::Subscriber cal_sub = nh.subscribe("/bno055/cal", 1000, cal_callback);
    
    ros::Publisher pose_pub = nh.advertise<kalman_filters::ahrs>("/ahrs/pose",1000);
    kalman_filters::ahrs pose_msg;

    ros::Rate loop_rate(300);

    bool calFlag = false;
    int N = 500;
    int init_offset_ct = N;

    double phi_offset = 0.0;
    double theta_offset = 0.0;
    double phi_dot, theta_dot;
    double phi_acc, theta_acc;
    double phi_hat = 0.0;
    double theta_hat = 0.0;

    Matrix4d A;
    Matrix<double, 4, 2> B, K;
    Matrix2d S;
    Matrix<double,2,4> C;
    C.row(0) << 1, 0, 0, 0;
    C.row(1) << 0, 0, 1, 0;
    Matrix4d P = Matrix4d::Identity() * 1, Q = Matrix4d::Identity();
    Matrix2d R = Matrix2d::Identity() * 0.003;

    double Q_a = 5;
    double Q_b = 0.003;

    Q << Q_a, 0, 0,   0,
        0,   Q_b, 0,   0,
        0,   0, Q_a, 0,
        0,   0, 0,   Q_b;

    std::cout << "P: \n" << P << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;

    Vector4d state_estimate = Vector4d::Ones().transpose();
    Vector2d gyro_input;
    Vector2d measurement;
    Vector2d y_tilde;

    double start_time = ros::Time::now().toSec();
    double dt;

    int print_counter = 0;
    int print_rate = 90;

    while(ros::ok()){

        if (!calFlag){
            // If the IMU is not calibrated, wait until all 3 values in the calibration message are 3 (well calibratied)
            if (!cal_buffer.empty()){
                geometry_msgs::Vector3Stamped cal_msg = cal_buffer.back();
                if (cal_msg.vector.x == 3 && cal_msg.vector.y == 3 && cal_msg.vector.z == 3){
                    calFlag = true;
                    std::cout << "IMU calibrated. Return to initial position (3 seconds)..." << std::endl;
                    sleep(4);
                }
                else{
                    if (print_counter == print_rate){
                        std::cout << "IMU not yet calibrated" << std::endl;
                        std::cout << "IMU calibration message in buffer" << std::endl;
                        std::cout << "Time: " << cal_msg.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << cal_msg.header.stamp.nsec << std::endl;
                        std::cout << "Status Gyro: " << cal_msg.vector.x << " Accel: " << cal_msg.vector.y << " Mag: " << cal_msg.vector.z << std::endl << std::endl;
                        print_counter = 0;
                    }else{
                        print_counter++;
                    }
                    cal_buffer.clear();
                }
            }
        }
        else if (init_offset_ct != 0){
            // Determine initial offsets
            if (!imu_buffer.empty()){
                // Grab imu message from buffer
                sensor_msgs::Imu imu_msg = imu_buffer.back();
                imu_buffer.clear();

                // phi_acc = atan2(imu_msg.linear_acceleration.y, sqrt(pow(imu_msg.linear_acceleration.x, 2.0) + pow(imu_msg.linear_acceleration.z, 2.0)));
                phi_acc = atan2(imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
                theta_acc = atan2(-imu_msg.linear_acceleration.x, sqrt(pow(imu_msg.linear_acceleration.y, 2.0) + pow(imu_msg.linear_acceleration.z, 2.0)));
                
                phi_offset += phi_acc;
                theta_offset += theta_acc;
                init_offset_ct--;

                if (print_counter == print_rate){
                    std::cout << "Determining initial angle offsets (" << phi_offset << ", " << theta_offset << ")..." << std::endl;
                    print_counter = 0;
                }
                else{
                    print_counter++;
                }
            }
        }
        else{
            // Once the IMU is calibrated once, don't need to check the calibration message anymore
            if (!imu_buffer.empty()){
                // Grab imu message from buffer
                sensor_msgs::Imu imu_msg = imu_buffer.back();
                imu_buffer.clear();

                // Get timing information
                dt = imu_msg.header.stamp.toSec() - start_time;
                start_time = imu_msg.header.stamp.toSec();

                // Display IMU message information
                if (print_counter == print_rate){
                    std::cout << "IMU message in buffer" << std::endl;
                    std::cout << "Time: " << imu_msg.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << imu_msg.header.stamp.nsec << std::endl;
                    std::cout << "Accel X: " << imu_msg.linear_acceleration.x << " Y: " << imu_msg.linear_acceleration.y << " Z: " << imu_msg.linear_acceleration.z << std::endl;
                    std::cout << "Gyro X: " << imu_msg.angular_velocity.x << " Y: " << imu_msg.angular_velocity.y << " Z: " << imu_msg.angular_velocity.z << std::endl << std::endl;
                }
                
                // Calculate Euler angle derivatives
                phi_dot = imu_msg.angular_velocity.x + sin(phi_hat) * tan(theta_hat) * imu_msg.angular_velocity.y + cos(phi_hat) * tan(theta_hat) * imu_msg.angular_velocity.z;
                theta_dot = cos(phi_hat) * imu_msg.angular_velocity.y - sin(phi_hat) * imu_msg.angular_velocity.z;

                // phi_acc = atan2(imu_msg.linear_acceleration.y, sqrt(pow(imu_msg.linear_acceleration.x, 2.0) + pow(imu_msg.linear_acceleration.z, 2.0)));
                phi_acc = atan2(imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
                theta_acc = atan2(-imu_msg.linear_acceleration.x, sqrt(pow(imu_msg.linear_acceleration.y, 2.0) + pow(imu_msg.linear_acceleration.z, 2.0)));
                
                // Subtract initial offset
                phi_acc -= phi_offset / double(N);
                theta_acc -= theta_offset / double(N);

                // Kalman filter
                A << 0, -dt, 0,   0,
                     0,   1, 0,   0,
                     0,   0, 1, -dt,
                     0,   0, 0,   1;

                B << dt, 0,
                     0,  0,
                     0, dt,
                     0,  0;

                gyro_input << phi_dot,
                              theta_dot;

                state_estimate << A * state_estimate + B * gyro_input;

                P << A * (P * A.transpose()) + Q;

                measurement << phi_acc,
                               theta_acc;

                y_tilde << measurement - C * state_estimate;

                S << R + C * (P * C.transpose());

                K << P * (C.transpose() * S.inverse());
                
                state_estimate << state_estimate + K * y_tilde;

                P << (Matrix4d::Identity() - K * C) * P;

                phi_hat = state_estimate(0);
                theta_hat = state_estimate(2);

                // Publish nav pose message
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.roll = phi_hat * 180.0 / M_PI;
                pose_msg.pitch = theta_hat * 180.0 / M_PI;
                pose_pub.publish(pose_msg);

                if (print_counter == print_rate){
                    printf("State estimates: Phi = %.2f Theta = %.2f\n\n",phi_hat * 180.0 / M_PI, theta_hat * 180.0 / M_PI);
                    print_counter = 0;
                }else{
                    print_counter++;
                }

            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
