
/*
 * Project: BNO055 IMU subscriber
 * Description: Node that subscribers to Gyro, Accel, and Mag messages from the BNO055 9-DOF IMU
 * Author: Annis Nusseibeh
 * Date: October 18, 2020
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Range.h"

#include <iostream>
#include <vector>

using namespace std;

std::vector<sensor_msgs::Imu> imu_buffer;
std::vector<sensor_msgs::MagneticField> mag_buffer;
std::vector<geometry_msgs::Vector3Stamped> cal_buffer;


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_buffer.push_back(*msg);
}

void mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
    mag_buffer.push_back(*msg);
}

void cal_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    cal_buffer.push_back(*msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bno055_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/bno055/imu", 1000, imu_callback);
    ros::Subscriber mag_sub = nh.subscribe("/bno055/mag", 1000, mag_callback);
    ros::Subscriber cal_sub = nh.subscribe("/bno055/cal", 1000, cal_callback);
    
    ros::Rate loop_rate(100);

    while(ros::ok()){
        // Once the IMU is calibrated once, don't need to check the calibration message anymore
        if (!imu_buffer.empty()){
            sensor_msgs::Imu imu_msg = imu_buffer.back();
            std::cout << "IMU message in buffer" << std::endl;
            std::cout << "Time: " << imu_msg.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << imu_msg.header.stamp.nsec << std::endl;
            std::cout << "Accel X: " << imu_msg.linear_acceleration.x << " Y: " << imu_msg.linear_acceleration.y << " Z: " << imu_msg.linear_acceleration.z << std::endl;
            std::cout << "Gyro X: " << imu_msg.angular_velocity.x << " Y: " << imu_msg.angular_velocity.y << " Z: " << imu_msg.angular_velocity.z << std::endl << std::endl;
            imu_buffer.clear();
        }

        if (!mag_buffer.empty()){
            sensor_msgs::MagneticField mag_msg = mag_buffer.back();
            std::cout << "Mag message in buffer" << std::endl;
            std::cout << "Time: " << mag_msg.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << mag_msg.header.stamp.nsec << std::endl;
            std::cout << "Field X: " << mag_msg.magnetic_field.x << " Y: " << mag_msg.magnetic_field.y << " Z: " << mag_msg.magnetic_field.z << std::endl << std::endl;
            mag_buffer.clear();
        }

        if(!cal_buffer.empty()){
            geometry_msgs::Vector3Stamped cal_msg = cal_buffer.back();
            std::cout << "Cal message in buffer" << std::endl;
            std::cout << "Time: " << cal_msg.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << cal_msg.header.stamp.nsec << std::endl;
            std::cout << "Status Gyro: " << cal_msg.vector.x << " Accel: " << cal_msg.vector.y << " Mag: " << cal_msg.vector.z << std::endl << std::endl;
            cal_buffer.clear();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
