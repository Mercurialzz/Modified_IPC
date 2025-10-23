#ifndef __PARAM_H
#define __PARAM_H

#include <ros/ros.h>
#include <Eigen/Dense>
class Parameter_t
{
public:

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};


	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	AutoTakeoffLand takeoff_land;



    bool simu_flag;
    bool perfect_simu_flag;
	double mass;
	double gra;
	double max_manual_vel;
	double low_voltage;
    double ctrl_freq_max;

    double ctrl_delay;
    double sfc_dis;

    double thrust_limit;
    bool hover_esti_flag;
    bool hover_perc;

    bool yaw_ctrl_flag;
    double yaw_gain;
    
    double goal_x;
    double goal_y;
    double goal_z;
    
	double resolution;
	Eigen::Vector3d map_size;
	double expand_dyn;
	double expand_fix;

	int ref_dis;
	double path_dis;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif