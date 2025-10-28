#include "param.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{

    read_essential_param(nh, "simulation", simu_flag);
    read_essential_param(nh, "perfect_simu", perfect_simu_flag);
    read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "low_voltage", low_voltage);

	read_essential_param(nh, "ctrl_delay", ctrl_delay);
	read_essential_param(nh, "sfc/sfc_dis", sfc_dis);
	read_essential_param(nh, "sfc/box_min_x", box_min.x());
	read_essential_param(nh, "sfc/box_min_y", box_min.y());
	read_essential_param(nh, "sfc/box_min_z", box_min.z());
	read_essential_param(nh, "sfc/box_max_x", box_max.x());
	read_essential_param(nh, "sfc/box_max_y", box_max.y());
	read_essential_param(nh, "sfc/box_max_z", box_max.z());
	
	read_essential_param(nh, "thrust_limit", thrust_limit);
	read_essential_param(nh, "hover_esti", hover_esti_flag);
	read_essential_param(nh, "hover_perc", hover_perc);

	read_essential_param(nh, "yaw_ctrl_flag", yaw_ctrl_flag);
	read_essential_param(nh, "yaw_gain", yaw_gain);
	
	read_essential_param(nh, "goal_x", goal_x);
	read_essential_param(nh, "goal_y", goal_y);
	read_essential_param(nh, "goal_z", goal_z);

	read_essential_param(nh, "astar/resolution", resolution);
	read_essential_param(nh, "astar/map_x_size", map_size.x());
	read_essential_param(nh, "astar/map_y_size", map_size.y());
	read_essential_param(nh, "astar/map_z_size", map_size.z());
	read_essential_param(nh, "astar/expand_dyn", expand_dyn);
	read_essential_param(nh, "astar/expand_fix", expand_fix);

	read_essential_param(nh, "fsm/ref_dis", ref_dis);
	read_essential_param(nh, "fsm/path_dis", path_dis);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);
	read_essential_param(nh, "msg_timeout/goal", msg_timeout.goal);


	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);




	// if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	// {
	// 	takeoff_land.enable_auto_arm = false;
	// 	ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	// }
	// if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	// {
	// 	takeoff_land.no_RC = false;
	// 	ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	// }

};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
