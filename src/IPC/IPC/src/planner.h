#ifndef __PLANNER_H
#define __PLANNER_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <ros/package.h>

#include <fstream>
#include <mutex>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <Eigen/Dense>

#include "../include/astar.h"
#include "../include/mpc.h"
#include "../include/local_astar.h"
#include "sfc_core/corridor_generator.h"
#include <rog_map_ros/rog_map_ros1.hpp>

#include "input.h"
#include "param.h"
#include "sfc_core/corridor_vis.h"

// #include "ThrustCurve.h"

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};

struct AutoTakeoffLand_t
{
	bool landed{true};
	ros::Time toggle_takeoff_land_time;
	std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
	static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};
struct RcDy_Data_t
{
    bool is_hover_mode;
    bool is_command_mode;
    bool enter_hover_mode;
    bool enter_command_mode;
    bool toggle_reboot;
    void reset() {is_hover_mode = true;
                    enter_hover_mode = false;
                    is_command_mode = true;
                    enter_command_mode = false;
                    toggle_reboot = false;}
};
struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};


class PlannerClass
{
public:
	Parameter_t &param;

	RC_Data_t rc_data;
	Dynamic_Data_t dy_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;
    Goal_Data_t goal_data;
	PointCloud_Data_t point_cloud_data;

	ros::Publisher traj_start_trigger_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub; //debug
    ros::Publisher trajectory_pub;
    ros::Publisher vision_pub;
	//planner visualization
	ros::Publisher  gird_map_pub_, astar_pub_, cmd_pub_, sfc_pub_, mpc_path_pub_, goal_pub_;


    ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;

	Eigen::Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;

	enum State_t
	{
        WAIT_STATUS = 0,
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};

	PlannerClass(ros::NodeHandle &nh,Parameter_t &);
	void process();
	bool rc_is_received(const ros::Time &now_time);
	bool cmd_is_received(const ros::Time &now_time);
	bool odom_is_received(const ros::Time &now_time);
	bool imu_is_received(const ros::Time &now_time);
	bool bat_is_received(const ros::Time &now_time);
	bool goal_is_received();
	void reset_goal_flag();
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }
    bool judge_in_fence();
    bool judge_close_to_fence();

		//
	void LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
private:
	State_t state; // Should only be changed in PlannerClass::process() function!
	AutoTakeoffLand_t takeoff_land;
	RcDy_Data_t rc_dy_data;

	// std::shared_ptr<PlannerClass> planner_;
	bool has_map_flag_{false}, has_odom_flag_{false}, replan_flag_{false}, new_goal_flag_{false};
	bool simu_flag_, perfect_simu_flag_, hover_esti_flag_, yaw_ctrl_flag_;
	double resolution_;
	double ctrl_delay_;
    double thrust_limit_, hover_perc_;
	double sfc_dis_, path_dis_, expand_dyn_, expand_fix_;
	int ref_dis_;
	int mpc_ctrl_index_;
    std::vector<Eigen::Vector3d> remain_nodes_;

	std::ofstream write_time_;
    std::vector<double> log_times_;
	ros::Time last_mpc_time_;
	std::mutex  odom_mutex_, goal_mutex_, cloud_mutex_, local_pc_mutex_;

	Eigen::Vector3d goal_p_, map_upp_;
	Eigen::Vector3d box_min_, box_max_;
	Eigen::Vector3d rate_;
	double yaw_{0}, yaw_r_{0}, yaw_dot_r_{0}, yaw_gain_;
	double init_yaw_{0};
	double yaw_ki_{0.05};
	double yaw_rate_limit_{1.5};
	double yaw_i_limit_{0.5};
	double yaw_int_{0.0};

	int astar_index_{0};
    std::vector<Eigen::Vector3d> astar_path_;
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<Eigen::Vector3d> follow_path_;
    std::vector<Eigen::Vector3d> replan_path_;
    std::vector<Eigen::Vector3d> local_pc_;
    std::vector<Eigen::Vector3d> local_pc_buffer_[10];
    std::vector<Eigen::Vector3d> mpc_goals_;
	bool have_path_{false},last_have_path_{false};

	double thr2acc_;
    double thrust_;
    double P_{100.0};
    Eigen::Vector3d Gravity_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;

    std::deque<pcl::PointCloud<pcl::PointXYZ>> vec_cloud_;
    pcl::PointCloud<pcl::PointXYZ> static_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_;

    std::shared_ptr<rog_map::ROGMapROS> map_ptr_;
    std::shared_ptr<LoaclAstarClass> local_astar_;
    std::shared_ptr<MPCPlannerClass> mpc_;
    std::shared_ptr<CorridorGenerator> corridor_gen_;

	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();
	Desired_State_t get_goal_des(Eigen::Vector3d goal_p);

	// ---- auto takeoff/land ----
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const ros::Time now);
	Desired_State_t get_takeoff_land_des(const double speed);
	

	// ---- tools ----
	void WriteLogTime(void);
	void ComputeThrust(Eigen::Vector3d acc,const Eigen::Quaterniond& q);
	void ConvertCommand(Eigen::Vector3d acc, Eigen::Vector3d jerk);
	bool estimateThrustModel(const Eigen::Vector3d &est_a,const Eigen::Quaterniond &q);
	void resetThrustMapping(void)
    {
        thr2acc_ = 9.81 / hover_perc_;
        P_ = 100;
    }
	void PathReplan(bool extend,const Odom_Data_t& odom,const Desired_State_t& des);
    void GeneratePolyOnPath();
    void GenerateAPolytopeFromLine(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index);
	void GenerateAPolytopeFromPoint(Eigen::Vector3d pos, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index);
	void set_hov_with_odom();
	void set_hov_with_rc();
	void MpcCalculate(const Odom_Data_t& odom,const Imu_Data_t& imu, Controller_Output_t& u);

	void SetSFCAndGoal(const Odom_Data_t& odom, const Desired_State_t& des);
	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();
	void CmdMode(const Odom_Data_t& odom,const Desired_State_t& des);
	void PointCloudCorpAndSetMap(const Odom_Data_t& odom, PointCloud_Data_t& pc2);
	void MPCSetGoal(const Eigen::Vector3d& goal_pos,const Eigen::Vector3d& goal_vel,const Eigen::Vector3d& goal_acc,double yaw);
	void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);


	void AstarPublish(std::vector<Eigen::Vector3d>& nodes, uint8_t type, double scale);
	void CmdPublish(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r);
	void MPCPathPublish(std::vector<Eigen::Vector3d> &pt);
	void StateUpdate(void);
	void CorridorInit(Parameter_t &param);
	void visualization_sfc(const Polytope &sfc);
};

#endif