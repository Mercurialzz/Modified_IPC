#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

// ROS Publishers & Subscribers
ros::Publisher trans_odom_pub;
ros::Publisher vision_pub;
ros::Subscriber odom_sub;
ros::Subscriber imu_sub;
ros::Timer timer_vision_pub;

// 全局变量
sensor_msgs::Imu imu;
geometry_msgs::PoseStamped vision_pose;
Eigen::Vector3d pub_pos;
Eigen::Quaterniond pub_q;

// IMU回调函数
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;
}

// // 定时发布vision pose的回调函数
// void timercb_pub_vision_pose(const ros::TimerEvent &e) {
//     vision_pose.header.stamp = ros::Time::now();
    
//     // 设置位置
//     vision_pose.pose.position.x = pub_pos.x();
//     vision_pose.pose.position.y = pub_pos.y();
//     vision_pose.pose.position.z = pub_pos.z();
    
//     // 设置姿态四元数
//     vision_pose.pose.orientation.x = pub_q.x();
//     vision_pose.pose.orientation.y = pub_q.y();
//     vision_pose.pose.orientation.z = pub_q.z();
//     vision_pose.pose.orientation.w = pub_q.w();
    
//     vision_pub.publish(vision_pose);
// }

// 里程计回调函数
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    // nav_msgs::Odometry trans_odom = *msg;
    
    // 提取里程计位姿信息
    Eigen::Quaterniond q_odom(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
                             
    Eigen::Vector3d pos(msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z);
    
    vision_pose.header.stamp = msg->header.stamp;
    
    // 设置位置
    vision_pose.pose.position.x = pos.x();
    vision_pose.pose.position.y = pos.y();
    vision_pose.pose.position.z = pos.z();

    // 设置姿态四元数
    vision_pose.pose.orientation.x = q_odom.x();
    vision_pose.pose.orientation.y = q_odom.y();
    vision_pose.pose.orientation.z = q_odom.z();
    vision_pose.pose.orientation.w = q_odom.w();

    vision_pub.publish(vision_pose);
    // 打印调试信息
    std::cout << "pos_odom: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
    std::cout << "q_odom: " << q_odom.x() << " " << q_odom.y() << " " << q_odom.z() << " " << q_odom.w() << std::endl;
}

//这里频率太低，可能会造成PX4那边ekf2融合出问题吗，因此还需要再PX4参数对vision的融合进行调参，gate参数
int main(int argc, char **argv) {
    ros::init(argc, argv, "trans_odom");
    ros::NodeHandle nh;

    // 初始化ROS通信
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    trans_odom_pub = nh.advertise<nav_msgs::Odometry>("/trans_odom", 10);
    // timer_vision_pub = nh.createTimer(ros::Duration(0.1), timercb_pub_vision_pose);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10, odom_cb);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, imu_cb);

    // ROS主循环
    while (ros::ok()) {
        ros::spinOnce();
    }
    
    return 0;
}
