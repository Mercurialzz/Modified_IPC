#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <uav_utils/utils.h>
#include "param.h"
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include "ipc/fake_rcConfig.h"
class RC_Data_t
{
public:
  double mode;
  double gear;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];

  mavros_msgs::RCIn msg;
  ros::Time rcv_stamp;

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool toggle_reboot;

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);
  bool is_received(const ros::Time &now_time);
};

class Dynamic_Data_t
{
public:

    bool is_command_mode;
    bool enter_command_mode;
    bool is_hover_mode;
    bool enter_hover_mode;
    bool toggle_reboot;

    bool mode;
    bool last_mode;
    bool cmd;
    bool last_cmd; // fake_rcConfig
    bool have_init_last_mode{false};
    bool have_init_last_cmd{false};

    ipc::fake_rcConfig msg;

    Dynamic_Data_t();
    void feed(ipc::fake_rcConfig &config);

};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  double yaw;
  nav_msgs::Odometry msg;
  ros::Time rcv_stamp;
  bool recv_new_msg;
  bool vel_in_body{true};

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t
{
public:
  mavros_msgs::State current_state;
  mavros_msgs::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp;

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

class Goal_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    geometry_msgs::PoseStamped msg;
    Eigen::Vector3d new_goal;
    Eigen::Vector3d last_goal;

    ros::Time rcv_stamp;
    bool recv_new_msg;

    Goal_Data_t();
    void feed(geometry_msgs::PoseStampedConstPtr pMsg);
};
class PointCloud_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    sensor_msgs::PointCloud2 msg;

    // std::vector<Eigen::Vector3d> local_pc_;

    std::deque<pcl::PointCloud<pcl::PointXYZ>> vec_cloud_;
    pcl::PointCloud<pcl::PointXYZ> static_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_;

    ros::Time rcv_stamp;
    bool recv_new_msg;

    PointCloud_Data_t();
    void feed(sensor_msgs::PointCloud2ConstPtr pMsg);
};

#endif
