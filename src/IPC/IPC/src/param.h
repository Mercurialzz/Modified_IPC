#ifndef __PARAM_H
#define __PARAM_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <rog_map/rog_map_core/config.hpp>



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
		double goal;
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
    double hover_perc;

    bool yaw_ctrl_flag;
    double yaw_gain;
	double yaw_ki;
	double yaw_rate_limit;
	double yaw_i_limit;
    
    double goal_x;
    double goal_y;
    double goal_z;
    
	double goal_x_1;
    double goal_y_1;
    double goal_z_1;
    
	double goal_x_2;
    double goal_y_2;
    double goal_z_2;

	double goal_x_3;
    double goal_y_3;
    double goal_z_3;
	
    bool visualization_en;
    double corridor_bound_dis;
    double corridor_line_max_length;
    double safe_corridor_line_max_length;
	bool frontend_in_known_free;
    int iris_iter_num;
    int obs_skip_num;
    double replan_forward_dt;
    double planning_horizon;
    double sensing_horizon;
    double receding_dis;
    double robot_r;
	int ref_dis;
	double path_dis;

	rog_map::vec_E<rog_map::Vec3i> seed_line_neighbour;

	double resolution;
	Eigen::Vector3d map_size;
	double expand_dyn;
	double expand_fix;
	

	Eigen::Vector3d box_min;
	Eigen::Vector3d box_max;

	


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