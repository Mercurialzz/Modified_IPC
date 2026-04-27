#ifndef __PLANNER_H
#define __PLANNER_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <ros/package.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <cstdint>
#include <condition_variable>

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

#include "mpc_control/mpc.h"
#include "path_search/rog_astar.h"
#include "sfc_core/corridor_generator.h"
#include <rog_map_ros/rog_map_ros1.hpp>
#include <vis_interface/vis_interface.hpp>
#include <utils/header/type_utils.hpp>

#include "input.h"
#include "param.h"


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
	ros::Publisher  gird_map_pub_, astar_pub_, cmd_pub_, mpc_path_pub_, goal_pub_;


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
	~PlannerClass();
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
	void LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
	State_t state; // Should only be changed in PlannerClass::process() function!
	AutoTakeoffLand_t takeoff_land;
	RcDy_Data_t rc_dy_data;

	struct GoalInfo {
		Vec3f goal_p{0, 0, 0};
		double goal_yaw{0};
		bool new_goal{true};
		bool goal_valid{true};
	} gi_;

	// std::shared_ptr<PlannerClass> planner_;
	bool has_map_flag_{false}, has_odom_flag_{false}, replan_flag_{false}, new_goal_flag_{false};
	bool return_flag_{false};
	bool simu_flag_, perfect_simu_flag_, hover_esti_flag_, yaw_ctrl_flag_;
	bool emergency_stop_flag_{false};
	bool path_blocked_flag_{false};
	bool corridor_generation_failed_{false};
	double ctrl_delay_;
    double thrust_limit_, hover_perc_;
	double path_dis_;
	int ref_dis_;
	double planning_horizon_;
	bool sim_mode_{false};
	int mpc_ctrl_index_;
    std::vector<Eigen::Vector3d> remain_nodes_;

	std::ofstream write_time_;
	std::ofstream write_data_;
    std::atomic<double> map_log_time_ms_{0.0};
    std::array<double, 4> log_times_{{0.0, 0.0, 0.0, 0.0}};
	Eigen::Vector3d mpc_pos_;
	ros::Time last_mpc_time_;
	std::mutex  odom_mutex_, goal_mutex_, cloud_mutex_, local_pc_mutex_, log_mutex_;

	Eigen::Vector3d goal_p_;
	Eigen::Vector3d rate_;
	double yaw_{0}, yaw_r_{0}, yaw_dot_r_{0}, yaw_gain_;
	double init_yaw_{0};
	double yaw_ki_{0.05};
	double yaw_rate_limit_{1.5};
	double yaw_i_limit_{0.5};
	double yaw_int_{0.0};
	int mpc_fail_count_{0};
	const int mpc_fail_limit_{3};

	int astar_index_{0};
    std::uint64_t path_version_{0};
    std::uint64_t active_path_version_{0};
    const int replan_transition_steps_{3};
    vec_Vec3f astar_path_;
    vec_Vec3f waypoints_;
    vec_Vec3f follow_path_;
    vec_Vec3f replan_path_;
    vec_Vec3f mpc_goals_;

	bool have_path_{false},last_have_path_{false};
	int change_goal{1};  // 目标点切换标志：1表示第一个目标，-1表示第二个目标
	bool goal_reached_{false};  // 目标点是否已到达
	double goal_reach_threshold_{0.5};  // 目标点到达判定阈值（单位：m）

	double thr2acc_;
    double thrust_;
    double P_{100.0};
    Eigen::Vector3d Gravity_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;

    std::deque<pcl::PointCloud<pcl::PointXYZ>> vec_cloud_;
    pcl::PointCloud<pcl::PointXYZ> static_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_;

    std::shared_ptr<rog_map::ROGMapROS> map_ptr_;
    std::shared_ptr<MPCPlannerClass> mpc_;
    std::shared_ptr<CorridorGenerator> corridor_gen_;
	std::shared_ptr<vis_interface::VisInterface> vis_ptr_;
	std::shared_ptr<path_search::Astar> astar_ptr_;

	// 【新增】多线程相关变量
    std::thread planning_thread_;        // 规划线程
    std::mutex path_mutex_;              // 路径读写锁（保护 follow_path_）
    std::mutex data_mutex_;              // 数据读写锁（保护 odom 和 goal）
    std::condition_variable plan_cv_;    // 条件变量，用于唤醒规划线程
    
    std::atomic<bool> trigger_replan_flag_{false}; // 触发标志
    std::atomic<bool> is_planning_{false};         // 正在规划标志
    std::atomic<bool> thread_running_{true};       // 线程运行保活标志

    // 用于线程间传递的快照数据
    Eigen::Vector3d thread_start_pt_;
    Eigen::Vector3d thread_goal_pt_;

	// 【新增】规划线程主函数
	void PlanningThreadFunc();
    bool CommitFallbackPathToSafePoint(const Eigen::Vector3d &current_pos, Eigen::Vector3d &safe_point);
    bool FindNearestSafeGoal(const Eigen::Vector3d &requested_goal, Eigen::Vector3d &safe_goal);

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
	void WriteLogData(void);
	void ComputeThrust(Eigen::Vector3d acc,const Eigen::Quaterniond& q);
	void ConvertCommand(Eigen::Vector3d acc, Eigen::Vector3d jerk);
	bool estimateThrustModel(const Eigen::Vector3d &est_a,const Eigen::Quaterniond &q);
	void resetThrustMapping(void)
    {
        thr2acc_ = 9.81 / hover_perc_;
        P_ = 100;
    }
	// void PathReplan(bool extend,const Odom_Data_t& odom,const Desired_State_t& des);
    void GeneratePolyOnPath();
	bool GenerateAPolytopeFromLine(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index);
	bool GenerateAPolytopeFromPoint(Eigen::Vector3d pos, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index);
	void set_hov_with_odom();
	void set_hov_with_rc();
	bool MpcCalculate(const Odom_Data_t& odom,const Imu_Data_t& imu, Controller_Output_t& u);

	bool SetSFCAndGoal(const Odom_Data_t& odom, const Desired_State_t& des);
	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();
	void CmdMode(const Odom_Data_t& odom,const Desired_State_t& des);
	void PointCloudCorpAndSetMap(const Odom_Data_t& odom, PointCloud_Data_t& pc2);
	void MPCSetGoal(const Eigen::Vector3d& goal_pos,const Eigen::Vector3d& goal_vel,const Eigen::Vector3d& goal_acc,double yaw);
	void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);


	void AstarPublish(vec_Vec3f& nodes, uint8_t type, double scale);
	void CmdPublish(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r);
	void MPCPathPublish(std::vector<Eigen::Vector3d> &pt);
	void StateUpdate(void);
	void CorridorInit(Parameter_t &param);
	bool PathSearch(const Vec3f &start_pt,const Vec3f &goal,vec_Vec3f &path);
	void PathReplan(const Eigen::Vector3d& start_pt,const Eigen::Vector3d& goal);
	void EvaluateReplan();
	bool consume_new_goal(Eigen::Vector3d &goal_out);
	Eigen::Vector3d get_goal_position();
	bool is_goal_reached();
	void set_log_time(size_t idx, double value_ms);

	};

#endif
