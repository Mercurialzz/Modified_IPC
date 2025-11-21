#include "planner.h"
#include <uav_utils/converters.h>
#include <Eigen/Core>
#include <sfc_core/ciri.h>

using namespace std;
using namespace uav_utils;

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

PlannerClass::PlannerClass(ros::NodeHandle &nh, Parameter_t &param_) : param(param_)/*, thrust_curve(thrust_curve_)*/
{
    //param update
    simu_flag_ = param.simu_flag;
    perfect_simu_flag_ = param.perfect_simu_flag;
    ctrl_delay_ = param.ctrl_delay;
    sfc_dis_ = param.sfc_dis;
    box_max_ = param.box_max;
    box_min_ = param.box_min;
    thrust_limit_ = param.thrust_limit;
    hover_esti_flag_ = param.hover_esti_flag;
    hover_perc_ = param.hover_perc;
    yaw_gain_ = param.yaw_gain;
    yaw_ctrl_flag_ = param.yaw_ctrl_flag;
    yaw_ki_ = param.yaw_ki;
    yaw_rate_limit_ = param.yaw_rate_limit;
    yaw_i_limit_ = param.yaw_i_limit;


    goal_p_ = Eigen::Vector3d(param.goal_x, param.goal_y, param.goal_z);

    resolution_ = param.resolution;
    expand_dyn_ = param.expand_dyn;
    expand_fix_ = param.expand_fix;
    Eigen::Vector3d map_low, map_upp;

    map_low << -param.map_size.x()/2.0, -param.map_size.y()/2.0, 0.0;
    map_upp << param.map_size.x()/2.0, param.map_size.y()/2.0, param.map_size.z()/2.0;
    map_upp_ = map_upp;

    path_dis_ = param.path_dis;
    ref_dis_ = param.ref_dis;

    Gravity_ << 0, 0, 9.81;
    if (simu_flag_) {
        thrust_ = 0.7;
    } else {
        thrust_ = hover_perc_;
    }    
    thr2acc_ = 9.81 / thrust_;

    
    mpc_   = std::make_shared<MPCPlannerClass>(nh);
    
    // 初始化 ROGMap - 从 ROS 参数服务器读取配置文件路径
    std::string config_path;
    if (!nh.getParam("config_path", config_path)) {
        // 如果参数不存在,使用默认路径
        ROS_ERROR("config_path parameter not found");
    } else {
        ROS_INFO("ROGMap config path from launch file: %s", config_path.c_str());
    }
    
    map_ptr_ = std::make_shared<rog_map::ROGMapROS>(nh, config_path);
    const auto &rog_map_cfg = map_ptr_->getMapConfig();
    ROS_INFO("ROGMap initialized successfully.");

    vis_ptr_ = std::make_shared<vis_interface::VisInterface>(nh);
    vis_ptr_->setResolution(rog_map_cfg.resolution);
    vis_ptr_->setVisualizationEn(param.visualization_en);

    // 初始化A*
    astar_ptr_ = std::make_shared<path_search::Astar>(nh, vis_ptr_, map_ptr_);
    const int neighbor_step = floor(param.robot_r / rog_map_cfg.resolution);
    astar_ptr_->setFineInfNeighbors(neighbor_step);
    //初始化 CorridorGenerator
    CorridorInit(param); 

    std::string file = ros::package::getPath("ipc") + "/config";
    write_time_.open((file+"/time_consuming.csv"), std::ios::out | std::ios::trunc);
    log_times_.resize(5, 1);
    write_time_ << "mapping" << ", " << "replan" << ", " << "sfc" << ", " << "mpc" << ", " << "df" << ", " << std::endl;

    state = MANUAL_CTRL;
    hover_pose.setZero();
    rc_dy_data.reset();
}

void PlannerClass::CorridorInit(Parameter_t &param)
{
    // 初始化 CorridorGenerator,传入 ROGMap 实例
    // 使用参数文件中的 corridor 配置
    const auto &rog_map_cfg = map_ptr_->getMapConfig();

    corridor_gen_ = std::make_shared<CorridorGenerator>(
        vis_ptr_,
        map_ptr_,  // 传入 ROGMap 实例
        param.corridor_bound_dis, 
        param.corridor_line_max_length, 
        rog_map_cfg.resolution,
        rog_map_cfg.virtual_ground_height, 
        rog_map_cfg.virtual_ceil_height, 
        param.robot_r, 
        param.obs_skip_num, 
        param.iris_iter_num);

    int step = ceil(param.robot_r / rog_map_cfg.resolution);
        for (int x = -step; x <= step; x++) {
            for (int y = -step; y <= step; y++) {
                for (int z = -step; z <= step; z++) {
                    if (x * x + y * y + z * z <= step * step) {
                        param.seed_line_neighbour.push_back({x, y, z});
                    }
                }
            }
        }
    std::sort(param.seed_line_neighbour.begin(), param.seed_line_neighbour.end(),
                [](const auto& a, const auto& b) {
                    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
                });

    corridor_gen_->SetLineNeighborList(param.seed_line_neighbour);
}

void PlannerClass::StateUpdate(void)
{
    // ROS_INFO("StateUpdate");
    //odom update
    static bool odom_int = false;
    if(odom_data.recv_new_msg)
    {
        if(!odom_int) {
            init_yaw_ = odom_data.yaw;
            ROS_INFO("Initial yaw set to %.2f rad", init_yaw_);
            odom_int = true;
        }
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_data.a = odom_data.q * Eigen::Vector3d(0,0,1) * (thrust_ * thr2acc_) - Gravity_;
        yaw_ = odom_data.yaw;
        odom_data.recv_new_msg = false;
    }
    //goal update
    if(goal_data.recv_new_msg)
    {
        static Eigen::Vector3d last_goal;
        if (last_goal != goal_data.new_goal)
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_p_ = goal_data.new_goal;
            
            if (goal_p_.z() > map_upp_.z() - 0.5) goal_p_.z() = map_upp_.z() - 0.5;
            if (goal_p_.z() < 0.5) goal_p_.z() = 0.5;
            new_goal_flag_ = true;
            goal_p_.z() = 2.5;
            ROS_INFO("[px4ctrl] New goal received: (%.2f, %.2f, %.2f)", goal_p_.x(), goal_p_.y(), goal_p_.z());
        }
        // new_goal_flag_ = false;
        last_goal = goal_data.new_goal;
        goal_data.recv_new_msg = false;
    }
}
/*
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

void PlannerClass::process()
{
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    Desired_State_t des(odom_data);
    bool rotor_low_speed_during_land = false;

    if(param.takeoff_land.no_RC) {
        rc_dy_data.enter_command_mode = dy_data.enter_command_mode;
        rc_dy_data.enter_hover_mode = dy_data.enter_hover_mode;
        rc_dy_data.is_command_mode = dy_data.is_command_mode;
        rc_dy_data.is_hover_mode = dy_data.is_hover_mode;
        rc_dy_data.toggle_reboot = dy_data.toggle_reboot;
    } else {
        rc_dy_data.enter_command_mode = rc_data.enter_command_mode;
        rc_dy_data.enter_hover_mode = rc_data.enter_hover_mode;
        rc_dy_data.is_command_mode = rc_data.is_command_mode;
        rc_dy_data.is_hover_mode = rc_data.is_hover_mode;
        rc_dy_data.toggle_reboot = rc_data.toggle_reboot;
    }
    StateUpdate();

    // STEP1: state machine runs
    switch (state)
    {
        case MANUAL_CTRL:
        {
            if (rc_dy_data.enter_hover_mode) // Try to jump to AUTO_HOVER
            {
                if (!odom_is_received(now_time))
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
                    break;
                }
                if (goal_is_received())
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending goals before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
                    reset_goal_flag();
                    break;
                }
                if (odom_data.v.norm() > 3.0)
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
                    break;
                }

                state = AUTO_HOVER;
                resetThrustMapping();
                set_hov_with_odom();
                toggle_offboard_mode(true);

                ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
            }
            else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
            {
                if (!odom_is_received(now_time))
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
                    break;
                }
                if (goal_is_received())
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending goals before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
                    reset_goal_flag();
                    break;
                }
                if (odom_data.v.norm() > 0.1)
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
                    break;
                }
                if (!get_landed())
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
                    break;
                }
                if (rc_is_received(now_time)) // Check this only if RC is connected.
                {
                    if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
                    {
                        ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
                        while (ros::ok())
                        {
                            ros::Duration(0.01).sleep();
                            ros::spinOnce();
                            if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
                            {
                                ROS_INFO("\033[32m[px4ctrl] OK, you can takeoff again.\033[32m");
                                break;
                            }
                        }
                        break;
                    }
                }

                state = AUTO_TAKEOFF;
                resetThrustMapping();
                set_start_pose_for_takeoff_land(odom_data);
                toggle_offboard_mode(true);				  // toggle on offboard before arm
                for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
                {
                    ros::Duration(0.01).sleep();
                    ros::spinOnce();
                }
                if (param.takeoff_land.enable_auto_arm)
                {
                    toggle_arm_disarm(true);
                }
                takeoff_land.toggle_takeoff_land_time = now_time;

                ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
            }
            else if(!param.takeoff_land.enable && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF)//use qgc takeoff
            {
                if (!odom_is_received(now_time))
                {
                    ROS_ERROR("[px4ctrl] Reject To Offboard. No odom!");
                    break;
                }
                if (goal_is_received())
                {
                    ROS_ERROR("[px4ctrl] Reject To Offboard. You are sending goals before toggling into Offboard, which is not allowed. Stop sending commands now!");
                    reset_goal_flag();
                    break;
                }
                if (odom_data.v.norm() > 0.5)
                {
                    ROS_ERROR("[px4ctrl] Reject To Offboard. Odom_Vel=%fm/s, non-static to Offboard is not allowed!", odom_data.v.norm());
                    break;
                }
                if (rc_is_received(now_time)) // Check this only if RC is connected.
                {
                    if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
                    {
                        ROS_ERROR("[px4ctrl] Reject To Offboard. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
                        while (ros::ok())
                        {
                            ros::Duration(0.01).sleep();
                            ros::spinOnce();
                            if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
                            {
                                ROS_INFO("\033[32m[px4ctrl] OK, you can to Offboard again.\033[32m");
                                break;
                            }
                        }
                        break;
                    }
                }
                state = AUTO_HOVER;
                set_hov_with_odom();
                resetThrustMapping();
                // set_start_pose_for_takeoff_land(odom_data);
                toggle_offboard_mode(true);				  // toggle on offboard before arm
                for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
                {
                    ros::Duration(0.01).sleep();
                    ros::spinOnce();
                }
                // if (param.takeoff_land.enable_auto_arm)
                // {
                //     toggle_arm_disarm(true);
                // }
                // takeoff_land.toggle_takeoff_land_time = now_time;

                ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER01\033[32m");
            }

            if (rc_dy_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
            {
                if (state_data.current_state.armed)
                {
                    ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
                    break;
                }
                reboot_FCU();
            }

            break;
        }

        case AUTO_HOVER:
        {
            if (!rc_dy_data.is_hover_mode || !odom_is_received(now_time))
            {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);

                ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
            }
            else if (rc_dy_data.is_command_mode && goal_is_received())
            {
                if (state_data.current_state.mode == "OFFBOARD")
                {
                    state = CMD_CTRL;
                    des = get_goal_des(goal_p_);
                    // des = get_cmd_des();
                    ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
                }
            }
            else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
            {

                state = AUTO_LAND;
                set_start_pose_for_takeoff_land(odom_data);

                ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
            }
            else
            {
                set_hov_with_rc();
                des = get_hover_des();
                // MPCSetGoal(des.p, des.v, des.a, des.yaw);//debug
                if ((rc_dy_data.enter_command_mode) ||
                    (takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
                {
                    takeoff_land.delay_trigger.first = false;
                    publish_trigger(odom_data.msg);
                    ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
                }

                // cout << "des.p=" << des.p.transpose() << endl;
            }

            break;
        }

        case CMD_CTRL:
        {
            if (!rc_dy_data.is_hover_mode || !odom_is_received(now_time))
            {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);

                ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
            }
            else if (!rc_dy_data.is_command_mode)
            {
                state = AUTO_HOVER;
                set_hov_with_odom();
                des = get_hover_des();
                // MPCSetGoal(des.p, des.v, des.a, des.yaw);//debug
                ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
            }
            else
            {
                des = get_goal_des(goal_p_);
                // des = get_cmd_des();
                // CmdMode();
            }

            if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
                          param.msg_timeout.cmd);
            }

            break;
        }

        case AUTO_TAKEOFF:
        {
            if (!rc_dy_data.is_hover_mode || !odom_is_received(now_time))
            {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);

                ROS_WARN("[px4ctrl] AUTO_TAKEOFF(L2) --> MANUAL_CTRL(L1)");
            }
            else if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
            {
                des = get_rotor_speed_up_des(now_time);
                // MPCSetGoal(des.p, des.v, des.a, des.yaw); //debug
            }
            else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
            {
                state = AUTO_HOVER;
                set_hov_with_odom();
                ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

                takeoff_land.delay_trigger.first = true;
                takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
            }
            else
            {
                des = get_takeoff_land_des(param.takeoff_land.speed);
                // MPCSetGoal(des.p, des.v, des.a, des.yaw); //debug
            }

            break;
        }

        case AUTO_LAND:
        {
            if (!rc_dy_data.is_hover_mode || !odom_is_received(now_time))
            {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);

                ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
            }
            else if (!rc_dy_data.is_command_mode)
            {
                state = AUTO_HOVER;
                set_hov_with_odom();
                des = get_hover_des();
                ROS_INFO("[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
            }
            else if (!get_landed())
            {
                des = get_takeoff_land_des(-param.takeoff_land.speed);
            }
            else
            {
                rotor_low_speed_during_land = true;

                static bool print_once_flag = true;
                if (print_once_flag)
                {
                    ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
                    print_once_flag = false;
                }

                if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
                {
                    static double last_trial_time = 0; // Avoid too frequent calls
                    if (now_time.toSec() - last_trial_time > 1.0)
                    {
                        if (toggle_arm_disarm(false)) // disarm
                        {
                            print_once_flag = true;
                            state = MANUAL_CTRL;
                            toggle_offboard_mode(false); // toggle off offboard after disarm
                            ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
                        }

                        last_trial_time = now_time.toSec();
                    }
                }
            }

            break;
        }

        default:
            break;
    }

    // STEP3: solve and update new control commands
    if (rotor_low_speed_during_land) // used at the start of auto takeoff
    {
        motors_idling(imu_data, u);
    }
    else if(state != MANUAL_CTRL)
    {
        // controller update
        if(state == AUTO_HOVER || state == AUTO_TAKEOFF)
        {
            MPCSetGoal(des.p, des.v, des.a, des.yaw);
        }
        else if(state == CMD_CTRL)
        {
            //MPCSetGoal(des.p, des.v, des.a, des.yaw);
            CmdMode(odom_data,des);
        }
        ROS_INFO_THROTTLE(1,"[px4ctrl] MPC Goal Pos: %.2f, %.2f, %.2f",des.p.x(),des.p.y(),des.p.z());
        MpcCalculate(odom_data, imu_data, u);
    }

    // Eigen::Matrix<double, Eigen::Dynamic, 4> planes;
    // // GenerateAPolytopeFromPoint(odom_data.p,planes, 0);
    // Eigen::Vector3d next_pt = Eigen::Vector3d(odom_data.p.x() + 0.5f, odom_data.p.y() , odom_data.p.z());
    // GenerateAPolytopeFromLine(odom_data.p,next_pt,planes, 0);

    ROS_INFO_THROTTLE(1,"odom_vel norm: %.2f",odom_data.v.norm());
    // STEP4: publish control commands to mavros
    publish_bodyrate_ctrl(u, now_time);

    // STEP5: Detect if the drone has landed
    land_detector(state, des, odom_data);
    // cout << takeoff_land.landed << " ";
    // fflush(stdout);
    WriteLogTime();

    // STEP6: Clear flags beyound their lifetime
    dy_data.enter_hover_mode = false;
    dy_data.enter_command_mode = false;
    dy_data.toggle_reboot = false;
    rc_data.enter_hover_mode = false;
    rc_data.enter_command_mode = false;
    rc_data.toggle_reboot = false;
    takeoff_land_data.triggered = false;
}

void PlannerClass::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
    u.q = imu.q;
    u.bodyrates = Eigen::Vector3d::Zero();
    u.thrust = 0.04;
}

void PlannerClass::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
    static State_t last_state = State_t::MANUAL_CTRL;
    if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
    {
        takeoff_land.landed = false; // Always holds
    }
    last_state = state;

    if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
    {
        takeoff_land.landed = true;
        return; // No need of other decisions
    }

    // land_detector parameters
    constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
    constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
    constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

    static ros::Time time_C12_reached; // time_Constraints12_reached
    static bool is_last_C12_satisfy;
    if (takeoff_land.landed)
    {
        time_C12_reached = ros::Time::now();
        is_last_C12_satisfy = false;
    }
    else
    {
        bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
        if (C12_satisfy && !is_last_C12_satisfy)
        {
            time_C12_reached = ros::Time::now();
        }
        else if (C12_satisfy && is_last_C12_satisfy)
        {
            if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) //Constraint 3 reached
            {
                takeoff_land.landed = true;
            }
        }

        is_last_C12_satisfy = C12_satisfy;
    }
}

Desired_State_t PlannerClass::get_hover_des()
{
    Desired_State_t des;
    des.p = hover_pose.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = hover_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t PlannerClass::get_cmd_des()
{
    Desired_State_t des;
    des.p = cmd_data.p;
    des.v = cmd_data.v;
    des.a = cmd_data.a;
    des.j = cmd_data.j;
    des.yaw = cmd_data.yaw;
    des.yaw_rate = cmd_data.yaw_rate;

    return des;
}

Desired_State_t PlannerClass::get_goal_des(Eigen::Vector3d goal_p)
{
    Desired_State_t des;
    des.p = goal_p;
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = hover_pose(3);
    des.yaw_rate = 0;

    return des;
}

Desired_State_t PlannerClass::get_rotor_speed_up_des(const ros::Time now)
{
    double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
    double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
    if (des_a_z > 0.1)
    {
        ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
        des_a_z = 0.0;
    }

    Desired_State_t des;

    des.p = takeoff_land.start_pose.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d(0, 0, des_a_z);
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t PlannerClass::get_takeoff_land_des(const double speed)
{
    ros::Time now = ros::Time::now();
    double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
    // takeoff_land.last_set_cmd_time = now;

     //takeoff_land.start_pose(2) += speed * delta_t;

    Desired_State_t des;
    des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
    des.v = Eigen::Vector3d(0, 0, speed);
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land.start_pose(3);
    des.yaw_rate = 0.0;


    return des;
}

void PlannerClass::set_hov_with_odom()
{
    hover_pose.head<3>() = odom_data.p;
    hover_pose(3) = uav_utils::get_yaw_from_quaternion(odom_data.q);

    last_set_hover_pose_time = ros::Time::now();
}

void PlannerClass::set_hov_with_rc()
{
    ros::Time now = ros::Time::now();
    double delta_t = (now - last_set_hover_pose_time).toSec();
    last_set_hover_pose_time = now;

    hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
    hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
    hover_pose(2) += rc_data.ch[2] * 1.0 * delta_t * (param.rc_reverse.throttle ? 1 : -1);
    hover_pose(3) += rc_data.ch[3] * 1.0 * delta_t * (param.rc_reverse.yaw ? 1 : -1);

    if (hover_pose(2) < -0.3)
        hover_pose(2) = -0.3;

    // if (param.print_dbg)
    // {
    // 	static unsigned int count = 0;
    // 	if (count++ % 100 == 0)
    // 	{
    // 		cout << "hover_pose=" << hover_pose.transpose() << endl;
    // 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
    // 	}
    // }
}

void PlannerClass::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
    takeoff_land.start_pose.head<3>() = odom_data.p;
    takeoff_land.start_pose(3) = uav_utils::get_yaw_from_quaternion(odom_data.q);

    takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool PlannerClass::rc_is_received(const ros::Time &now_time)
{
    return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PlannerClass::cmd_is_received(const ros::Time &now_time)
{
    return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}
bool PlannerClass::goal_is_received()
{
    return new_goal_flag_;
}
void PlannerClass::reset_goal_flag()
{
    new_goal_flag_ = false;
}
bool PlannerClass::odom_is_received(const ros::Time &now_time)
{
    return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PlannerClass::imu_is_received(const ros::Time &now_time)
{
    return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PlannerClass::bat_is_received(const ros::Time &now_time)
{
    return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PlannerClass::recv_new_odom()
{
    if (odom_data.recv_new_msg)
    {
        odom_data.recv_new_msg = false;
        return true;
    }

    return false;
}

void PlannerClass::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    msg.body_rate.x = u.bodyrates.x();
    msg.body_rate.y = u.bodyrates.y();
    msg.body_rate.z = u.bodyrates.z();

    msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(msg);
}

void PlannerClass::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose = odom_msg.pose.pose;

    traj_start_trigger_pub.publish(msg);
}

bool PlannerClass::toggle_offboard_mode(bool on_off)
{
    mavros_msgs::SetMode offb_set_mode;

    if (on_off)
    {
        state_data.state_before_offboard = state_data.current_state;
        if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
            state_data.state_before_offboard.mode = "MANUAL";

        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
        {
            ROS_ERROR("Enter OFFBOARD rejected by PX4!");
            return false;
        }
    }
    else
    {
        offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
        {
            ROS_ERROR("Exit OFFBOARD rejected by PX4!");
            return false;
        }
    }

    return true;

    // if (param.print_dbg)
    // 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool PlannerClass::toggle_arm_disarm(bool arm)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
    {
        if (arm)
            ROS_ERROR("ARM rejected by PX4!");
        else
            ROS_ERROR("DISARM rejected by PX4!");

        return false;
    }

    return true;
}

void PlannerClass::reboot_FCU()
{
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;	  // Reboot autopilot
    reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;

    reboot_FCU_srv.call(reboot_srv);

    ROS_INFO("Reboot FCU");

    // if (param.print_dbg)
    // 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}

void PlannerClass::AstarPublish(vec_Vec3f& nodes, uint8_t type, double scale) {
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    if (type == 0) {
        node_vis.ns = "astar_path";
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    } else if (type == 1) {
        node_vis.ns = "floyd_path";
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    } else if (type == 2) {
        node_vis.ns = "short_path";
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 1.0;
    } else if (type == 3) {
        node_vis.ns = "set_points";
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;
    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    
    node_vis.scale.x = scale;
    node_vis.scale.y = scale;
    node_vis.scale.z = scale;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        node_vis.points.push_back(pt);
    }
    astar_pub_.publish(node_vis);
}
void PlannerClass::CmdPublish(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r) {
    quadrotor_msgs::PositionCommand msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();
    msg.position.x      = p_r.x();
    msg.position.y      = p_r.y();
    msg.position.z      = p_r.z();
    msg.velocity.x      = v_r.x();
    msg.velocity.y      = v_r.y();
    msg.velocity.z      = v_r.z();
    msg.acceleration.x  = a_r.x();
    msg.acceleration.y  = a_r.y();
    msg.acceleration.z  = a_r.z();
    msg.jerk.x          = j_r.x();
    msg.jerk.y          = j_r.y();
    msg.jerk.z          = j_r.z();
    if (yaw_ctrl_flag_) {
        double yaw_error = yaw_r_ - yaw_;
        if (yaw_error >  M_PI) yaw_error -= M_PI * 2;
        if (yaw_error < -M_PI) yaw_error += M_PI * 2;
        msg.yaw     = yaw_ + yaw_error * 0.1;
        msg.yaw_dot = 0;
    } else {
        msg.yaw     = 0;
        msg.yaw_dot = 0;
    }
    cmd_pub_.publish(msg);
}
void PlannerClass::MPCPathPublish(std::vector<Eigen::Vector3d> &pt) {
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    for (int i = 0; i < pt.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt[i].x();
        pose.pose.position.y = pt[i].y();
        pose.pose.position.z = pt[i].z();
        msg.poses.push_back(pose);
        // std::cout << "mpc path " << i << ": " << pt[i].transpose() << std::endl;
    }
    mpc_path_pub_.publish(msg);
}
void PlannerClass::WriteLogTime(void) {
    for (int i = 0; i < log_times_.size(); i++) {
        write_time_ << log_times_[i] << ", ";
        log_times_[i] = 0.0;
    }
    write_time_ << std::endl;
}

void PlannerClass::ComputeThrust(Eigen::Vector3d acc,const Eigen::Quaterniond& q) {
    const Eigen::Vector3d zB =  q * Eigen::Vector3d::UnitZ();
    double des_acc_norm = acc.dot(zB);
    thrust_ = des_acc_norm / thr2acc_;
}


void PlannerClass::ConvertCommand(Eigen::Vector3d acc, Eigen::Vector3d jerk) {
    Eigen::Vector3d xB, yB, zB, xC;
    if (yaw_ctrl_flag_) {
        // double yaw_error = yaw_r_ - yaw_;
        // if (yaw_error >  M_PI) yaw_error -= M_PI * 2;
        // if (yaw_error < -M_PI) yaw_error += M_PI * 2;
        // yaw_dot_r_ = yaw_error * yaw_gain_;
         // --- PI 控制 yaw_dot_r_ ---
        double yaw_error = yaw_r_ - yaw_;
        if (yaw_error >  M_PI) yaw_error -= 2*M_PI;
        if (yaw_error < -M_PI) yaw_error += 2*M_PI;

        static ros::Time last_t(0);
        ros::Time now = ros::Time::now();
        double dt = 0.0;
        if (last_t.toSec() > 0.0) dt = (now - last_t).toSec();
        last_t = now;

        // 积分更新与限幅（抗风up）
        yaw_int_ += yaw_error * dt;
        if (yaw_int_ >  yaw_i_limit_) yaw_int_ =  yaw_i_limit_;
        if (yaw_int_ < -yaw_i_limit_) yaw_int_ = -yaw_i_limit_;

        // PI 输出与速率饱和
        double yaw_rate_cmd = yaw_gain_ * yaw_error + yaw_ki_ * yaw_int_;
        if (yaw_rate_cmd >  yaw_rate_limit_) yaw_rate_cmd =  yaw_rate_limit_;
        if (yaw_rate_cmd < -yaw_rate_limit_) yaw_rate_cmd = -yaw_rate_limit_;

        yaw_dot_r_ = yaw_rate_cmd;       
    } else {
        // 无航向跟随：回到初始朝向（保持原逻辑）
        double yaw_error = init_yaw_ - yaw_;
        if (yaw_error >  M_PI) yaw_error -= 2*M_PI;
        if (yaw_error < -M_PI) yaw_error += 2*M_PI;
        yaw_dot_r_ = yaw_error * yaw_gain_;
        //限幅
        if (yaw_dot_r_ >  yaw_rate_limit_) yaw_dot_r_ =  yaw_rate_limit_;
        if (yaw_dot_r_ < -yaw_rate_limit_) yaw_dot_r_ = -yaw_rate_limit_;
    }
    xC << std::cos(yaw_), std::sin(yaw_), 0;

    zB = acc.normalized();
    yB = (zB.cross(xC)).normalized();
    xB = yB.cross(zB);
    Eigen::Matrix3d R;
    R << xB, yB, zB;
    // u_q_ = R;

    Eigen::Vector3d hw = (jerk - (zB.dot(jerk) * zB)) / acc.norm();
    rate_.x() = -hw.dot(yB);
    rate_.y() = hw.dot(xB);
    rate_.z() = yaw_dot_r_ * zB.dot(Eigen::Vector3d(0, 0, 1));
}

bool PlannerClass::estimateThrustModel(const Eigen::Vector3d &est_a,const Eigen::Quaterniond &q)
{
    // if (hover_esti_flag_ == false) {
    //     thr2acc_ = 9.81 / hover_perc_;
    //     return true;
    // }
    // if (mode_ != Command) {//debug
    //     P_ = 100.0;
    //     thr2acc_ = 9.81 / hover_perc_;
    //     return true;
    // }
    ros::Time t_now = ros::Time::now();

    Eigen::Matrix3d Rotate = q.toRotationMatrix().inverse();
    Eigen::Vector3d acc_body = Rotate * est_a;
    if (timed_thrust_.size() == 0) return false;
    std::pair<ros::Time, double> t_t = timed_thrust_.front();

    while (timed_thrust_.size() >= 1) {
        double delta_t = (t_now - t_t.first).toSec();
        if (delta_t > 1.0) {
            timed_thrust_.pop();
            continue;
        } 
        if (delta_t < 0.035) {
            return false;
        }

        /* Recursive least squares algorithm with vanishing memory */
        double thr = t_t.second;
        timed_thrust_.pop();
        /* Model: acc_body(2) = thr2acc * thr */
        double R = 0.3; // using Kalman filter
        double K = P_ / (P_ + R);
        thr2acc_ = thr2acc_ + K * (acc_body(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_;
        double hover_percentage = 9.81 / thr2acc_;
        if (hover_percentage > 0.8 || hover_percentage < 0.1) {
            // ROS_INFO_THROTTLE(1, "Estimated hover_percentage >0.8 or <0.1! Perhaps the accel vibration is too high!");
            thr2acc_ = hover_percentage > 0.8 ? 9.81 / 0.8 : thr2acc_;
            thr2acc_ = hover_percentage < 0.1 ? 9.81 / 0.1 : thr2acc_;
        }
        // ROS_WARN("[PX4CTRL] hover_percentage = %f", hover_percentage);
        return true;
    }
    return false;
}

void PlannerClass::MpcCalculate(const Odom_Data_t& odom,const Imu_Data_t& imu, Controller_Output_t& u)
{
    // calculate model predict control algorithm
    ros::Time mpc_start = ros::Time::now();
    mpc_->SetStatus(odom.p, odom.v, odom.a);
    bool success_flag = mpc_->Run();
    log_times_[3] = (ros::Time::now() - mpc_start).toSec() * 1000.0;

    Eigen::Vector3d u_optimal, p_optimal, v_optimal, a_optimal, u_predict;
    Eigen::MatrixXd A1, B1;
    Eigen::VectorXd x_optimal = mpc_->X_0_;
    if (success_flag) {
        ROS_INFO_THROTTLE(1,"MPC SUCCESS");
        last_mpc_time_ = ros::Time::now();
        for (int i = 0; i <= ctrl_delay_/mpc_->MPC_STEP; i++) {
            mpc_->GetOptimCmd(u_optimal, i);
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
            x_optimal = A1 * x_optimal + B1 * u_optimal;
        }
        mpc_ctrl_index_ = ctrl_delay_/mpc_->MPC_STEP;

        p_optimal << x_optimal(0,0), x_optimal(1,0), x_optimal(2,0);
        v_optimal << x_optimal(3,0), x_optimal(4,0), x_optimal(5,0);
        a_optimal << x_optimal(6,0), x_optimal(7,0), x_optimal(8,0);
        if (!perfect_simu_flag_) CmdPublish(odom.p, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);

        std::vector<Eigen::Vector3d> path;
        x_optimal = mpc_->X_0_;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->GetOptimCmd(u_predict, i);
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
            x_optimal = A1 * x_optimal + B1 * u_predict;
            path.push_back(Eigen::Vector3d(x_optimal(0,0), x_optimal(1,0), x_optimal(2,0)));
        }
        MPCPathPublish(path);
    } else {
        ROS_WARN("MPC CANT SLOVE!!");
        double delta_t = (ros::Time::now()-last_mpc_time_).toSec();
        if (delta_t >= mpc_->MPC_STEP) {
            mpc_ctrl_index_ += delta_t / mpc_->MPC_STEP;
            last_mpc_time_ = ros::Time::now();
        }
        mpc_->GetOptimCmd(u_optimal, mpc_ctrl_index_);
        // std::cout << "index: " << mpc_ctrl_index_ << " u_optimal:" << u_optimal.transpose() << std::endl;
        // std::cout << "x_0: " << mpc_->X_0_.transpose() << std::endl << std::endl;
        mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
        x_optimal = A1 * mpc_->X_0_ + B1 * u_optimal;
        p_optimal << x_optimal(0,0), x_optimal(1,0), x_optimal(2,0);
        v_optimal << x_optimal(3,0), x_optimal(4,0), x_optimal(5,0);
        a_optimal << x_optimal(6,0), x_optimal(7,0), x_optimal(8,0);
        if (!perfect_simu_flag_) CmdPublish(odom.p, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);
    }
    
    // ROS_INFO_THROTTLE(1,"a_optimal ");
    ros::Time df_start = ros::Time::now();
    estimateThrustModel(imu.a, odom.q);
    a_optimal = a_optimal + Gravity_; //为微分平坦转换公式做准备
    ComputeThrust(a_optimal,odom.q);
    ConvertCommand(a_optimal, u_optimal);
    //BodyrateCtrlPub(rate_, thrust_, ros::Time::now());

    u.bodyrates = rate_;
    u.thrust = thrust_;

    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thrust_));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    log_times_[4] = (ros::Time::now() - df_start).toSec() * 1000.0;
}

/*
 * 功能: 基于局部A*进行路径重规划，生成用于跟踪的稠密路径 follow_path_
 * 触发: 1) 新目标点到来（extend=true 表示扩大局部地图中心范围）
 *       2) 当前路径被新障碍阻断（extend=false 的局部重规划）
 * 输入:
 *   - extend: 是否扩展局部地图中心（通常新目标时为 true）
 *   - odom:   当前里程计（起点、裁剪范围中心等来源）
 *   - des:    目标期望状态（终点 des.p）
 * 关键内部参数:
 *   - map_upp_:   局部地图半尺寸(正半轴)，用于限制A*搜索窗口与目标截断
 *   - resolution_: A* 栅格分辨率，用于边界截断时留余量
 *   - expand_dyn_: 动态障碍物膨胀半径，构建占据栅格(局部碰撞地图)时使用
 *   - expand_fix_: 启动/终点采样修正半径基准（遇占据时逐步扩大）
 *   - path_dis_:   路径插值间距(米)，控制 follow_path_ 的稠密程度
 * 输出/副作用:
 *   - astar_path_:   原始A*路径
 *   - waypoints_:    Floyd 平滑/简化后的关键点
 *   - follow_path_:  最终用于跟踪的稠密路径（发布可视化与最后点作为goal_pub_）
 *   - goal_pub_:     发布最终跟踪的末端点（PoseStamped）
 */
void PlannerClass::PathReplan(const Eigen::Vector3d& start_pt,const Eigen::Vector3d& goal)
{
    // 执行A*搜索
    Vec3f start_pos = Vec3f(start_pt.x(), start_pt.y(), start_pt.z());
    Vec3f goal_pos  = Vec3f(goal.x(),  goal.y(),  goal.z());

    astar_path_.clear();
    waypoints_.clear();
    follow_path_.clear();

    bool search_flag = PathSearch(start_pos, goal_pos, astar_path_);
    if (search_flag) {
        //原始 A* 路径可视化（黑色小点）
        AstarPublish(astar_path_, 0, 0.1);
        //Floyd 简化
        astar_ptr_->FloydHandle(astar_path_, waypoints_);  // Floyd 平滑/去冗余为关键路径点
        // Floyd 结果可视化（红色稍大点）
        AstarPublish(waypoints_, 1, 0.1);
        // 3) 使用 path_dis_ 进行稠密插值
        if (!waypoints_.empty()) {
            follow_path_.push_back(waypoints_.front());
            for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
                Vec3f p0 = waypoints_[i];
                Vec3f p1 = waypoints_[i + 1];
                Vec3f seg = p1 - p0;
                double seg_len = seg.norm();
                if (seg_len < 1e-6) continue;
                int inter_num = static_cast<int>(std::floor(seg_len / path_dis_));
                // 只插入中间点，避免重复首尾
                for (int k = 1; k < inter_num; ++k) {
                    double ratio = double(k) / double(inter_num);
                    follow_path_.push_back(p0 + seg * ratio);
                }
                follow_path_.push_back(p1);
            }
        }

        follow_path_.push_back(waypoints_.back());        // 确保包含终点
        // 稠密路径可视化（蓝色，点尺度使用 path_dis_）
        AstarPublish(follow_path_, 2, path_dis_);
    } else {
        ROS_INFO("\033[41;37m No path! Stay at current point! \033[0m");
        // 搜索失败：保持当前位置为唯一跟踪点（保证下游模块有目标）
        follow_path_.push_back(start_pos);
    }

    // 发布当前的最终跟踪目标为 Path 的末尾点（供其他模块使用/可视化）
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = follow_path_.back().x();
    msg.pose.position.y = follow_path_.back().y();
    msg.pose.position.z = follow_path_.back().z();    
    goal_pub_.publish(msg);

    //ros::Time now = ros::Time::now();
    // astar_ptr_->Reset(); // 重置A*内部缓存（占据、开闭集等），避免下次重用旧状态
    // std::cout << "astar reset time is: " << (ros::Time::now() - now).toSec()*1000 << " ms. " << std::endl;
}

bool PlannerClass::PathSearch(const Vec3f &start_pt,const Vec3f &goal,vec_Vec3f &path)
{
    using namespace path_search; 
    

    // ---------- Step 1: 起点类型检测 ----------
    //确保起终点在地图内并且不在障碍物内
    //起点检查
    rog_map::GridType start_type = map_ptr_->getGridType(start_pt);
    if (start_type == rog_map::GridType::OCCUPIED || // 起点在障碍内部
        start_type == rog_map::GridType::OUT_OF_MAP) { // 或者越界
        ROS_WARN("-- [SUPER] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
        return false; // 直接返回, 上层需处理此不一致状态
    }
    // 临时容器: 逃逸路径(起点若需移动到最近自由点时的路径段)
    vec_E<Vec3f> start_point_escape_path;

    // flag_es: 在概率地图(PROB_MAP)上执行逃逸搜索, 根据参数将 UNKNOWN 视作 OCCUPIED 或 FREE
    int flag_es = ON_PROB_MAP | (param.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
    vec_Vec3f out_path; // 暂存逃逸搜索输出
    RET_CODE ret_es = astar_ptr_->escapePathSearch(start_pt, flag_es, out_path);
    if (ret_es != NO_NEED) { // 有需要进行逃逸处理
        if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) { // 逃逸失败
            ROS_ERROR(" -- [SUPER] Escape path search failed with [%s], force return.", 
                    RET_CODE_STR[ret_es].c_str());
            return false;
        } else {
            // 逃逸成功, 保存结果(此处的 out_path 已按时间/几何顺序排列)
            start_point_escape_path = out_path;
        }
    }

    // 若逃逸路径非空, 将起点平移至逃逸路径末端; 否则保持原始 start_pt
    Vec3f shifted_start_pt = start_point_escape_path.empty() ? start_pt : start_point_escape_path.back();

    // ---------- Step 2: 在膨胀地图(INF_MAP)进行主路径搜索 ----------
    // flag 组合: 使用膨胀地图 + 未知区域处理策略 + 不使用膨胀点邻居(初次搜索更严格减少搜索空间)
    int flag = ON_INF_MAP | (param.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | DONT_USE_INF_NEIGHBOR;

    // 主路径搜索: 起点 -> 目标
    RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(shifted_start_pt,
                                                            goal,
                                                            flag,
                                                            path);
    if (ret_code == INIT_ERROR) { // A* 初始化失败, 标记目标无效并返回
        gi_.goal_valid = false;
        return false;
    }
    // ---------- Step 3: 回退策略 ----------
    // 若在膨胀地图上无法找到路径, 切换到概率地图并开放膨胀点邻居(扩大搜索空间)再次尝试
    if (ret_code == NO_PATH) {
        flag = ON_PROB_MAP | (param.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | USE_INF_NEIGHBOR;
        //使用ROS_INFO打印带颜色的日志
        ROS_INFO("\033[31;1m -- [Astar] Path search failed on inf map, try again on prob map.\033[0m");
        ret_code = astar_ptr_->pointToPointPathSearch(shifted_start_pt, goal, flag, path);
        if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
            ROS_INFO("\033[32;1m -- [Astar] Path search on prob map success.\033[0m");
        } else {
            ROS_ERROR("\033[31;1m -- [Astar] Path search failed on prob map still failed.\033[0m");
        }
    }
    // ---------- Step 4: 成功码验证 ----------
    // 允许的成功状态: REACH_HORIZON (达到搜索边界) 或 REACH_GOAL (达到目标)
    if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
        ROS_ERROR(" -- [SUPER] Path search failed with [%s], force return.", RET_CODE_STR[ret_code].c_str());
        return false;
    }
    // ---------- Step 5: 拼接逃逸路径与主路径 ----------
    if (!start_point_escape_path.empty()) {
        // 将逃逸路径插入 path 开头(保持先后顺序)
        path.insert(path.begin(), start_point_escape_path.begin(), start_point_escape_path.end());
    }

    // 若主路径为空(极端情况, 例如仅返回逃逸路径或搜索失败未及早返回), 进行保护性检查
    if (path.empty()) {
        ROS_WARN(" -- [SUPER] Path search failed with empty segments, force return.");
        return false;
    }

    // 保证原始 start_pt 是第一个元素, 方便上层模块进行相对计算(如速度/时间戳关联)
    path.insert(path.begin(), start_pt);

    // 到达目标时附加 goal 点, 保持路径语义完整
    if (ret_code == REACH_GOAL) {
        path.push_back(goal);
    }
    return true;
}

void PlannerClass::GenerateAPolytopeFromLine(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    // 使用 CorridorGenerator 的 GeneratePolytopeFromLine 方法
    super_utils::Line seed_line = std::make_pair(
        super_utils::Vec3f(p1.x(), p1.y(), p1.z()),
        super_utils::Vec3f(p2.x(), p2.y(), p2.z())  
    );
    
    geometry_utils::Polytope polytope;
    bool success = corridor_gen_->GeneratePolytopeFromLine(seed_line, polytope);
    
    if (success) {
        if(index == 0) vis_ptr_->vizCurSfc(polytope);
        planes = polytope.GetPlanes();
    } else {
        // 失败时返回轴对齐边界盒（与原逻辑一致）
        ROS_WARN_THROTTLE(1.0, "[Planner] GeneratePolytopeFromLine failed, returning axis-aligned bounding box");
    }
}

void PlannerClass::GenerateAPolytopeFromPoint(Eigen::Vector3d pos, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    // 使用 CorridorGenerator 的 GeneratePolytopeFromPoint 方法
    // 使用GeneratePolytopeFromPoint
    super_utils::Vec3f point = super_utils::Vec3f(pos.x(), pos.y(), pos.z());
    
    geometry_utils::Polytope polytope;
    bool success = corridor_gen_->GeneratePolytopeFromPoint(point, polytope);
    
    if (success) {
        ROS_INFO_THROTTLE(1.0, "[Planner] GeneratePolytopeFromPoint succeeded.");
        if(index == 0) vis_ptr_->vizCurSfc(polytope);
        planes = polytope.GetPlanes();
    } else {
        // 失败时返回轴对齐边界盒（与原逻辑一致）
        ROS_WARN_THROTTLE(1.0, "[Planner] GeneratePolytopeFromLine failed, returning axis-aligned bounding box");
    }
}
void PlannerClass::CmdMode(const Odom_Data_t& odom,const Desired_State_t& des)
{
        ros::Time t_start = ros::Time::now();  // 记录总执行开始时间
        EvaluateReplan();
        //路径规划触发逻辑
        if (new_goal_flag_) {
            // 新目标点：执行完整路径规划（扩展模式）
            new_goal_flag_ = false;
            replan_flag_ = false;
            PathReplan(odom.p,des.p);
        }
        if (new_goal_flag_ == false && replan_flag_) {
            // 障碍物触发：执行局部重规划（非扩展模式）
            replan_flag_ = false;
            PathReplan(odom.p,des.p);
        }
        log_times_[1] = (ros::Time::now() - t_start).toSec() * 1000.0;  // 记录规划耗时

        SetSFCAndGoal(odom,des);
}

void PlannerClass::SetSFCAndGoal(const Odom_Data_t& odom, const Desired_State_t& des)
{
    // === 安全飞行走廊(SFC)生成和MPC目标设置 ===
    ros::Time sfc_start = ros::Time::now();
    if (follow_path_.size() > 0) { // 存在有效路径
        have_path_ = true;
        last_have_path_ = true;
        // 寻找路径上距离当前位置最近的点
        double min_dis = 10000.0;
        for (int i = 0; i < follow_path_.size(); i++) { 
            double dis = (odom.p - follow_path_[i]).norm();
            if (dis < min_dis) {
                min_dis = dis;
                astar_index_ = i;  // 记录最近点索引
            }
        }

        int goal_in_sfc = astar_index_;
        if (follow_path_.size() - astar_index_ <= ref_dis_) { 
            // 接近路径终点：在当前位置生成SFC
            goal_in_sfc = follow_path_.size();
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes;
            GenerateAPolytopeFromPoint(odom.p, planes, 0);
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetFSC(planes, i);  // 为整个MPC预测地平线设置相同SFC
            }
        } else { 
            // 正常路径跟踪：寻找最大最长的SFC
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes, last_planes;
            GenerateAPolytopeFromLine(follow_path_[astar_index_], follow_path_[astar_index_], planes, 0); 
            last_planes = planes;
            int init_num = 0;
            
            // 检查当前位置是否在初始SFC内
            if (mpc_->IsInFSC(odom.p, planes) == false) { 
                init_num = (odom.p - follow_path_[astar_index_]).norm() / path_dis_ / ref_dis_;
                ROS_INFO("\033[35m UAV is out sfc! init num is %d \033[0m", init_num);
            }
            
            // 寻找第一个SFC能覆盖的最远路径点
            int first_id = astar_index_, mpc_goal_index = 0;
            for (int i = first_id+1; i < follow_path_.size(); i++) { 
                if (mpc_->IsInFSC(follow_path_[i], planes)) {
                    first_id = i;
                    goal_in_sfc = i;
                } else {
                    // 回退一点确保安全
                    first_id -= 0.2 / path_dis_;
                    if (first_id < astar_index_) first_id = astar_index_;
                    break;
                }
            }
            
            // 计算第一个SFC在MPC地平线中的终止索引
            mpc_goal_index = (first_id - astar_index_) / ref_dis_ + init_num + 1;
            assert(first_id >= astar_index_);
            
            // 为MPC前段步骤设置第一个SFC
            for (int i = init_num; i <= mpc_goal_index && i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetFSC(planes, i);
            }
            
            // 尝试生成更长的SFC覆盖MPC剩余地平线
            int end_index = first_id + (mpc_->MPC_HORIZON-mpc_goal_index) * ref_dis_;
            if (end_index >= follow_path_.size()) end_index = follow_path_.size() - 1;
            // 使用 rog_map 可用接口进行安全性检测（点可达 + 线段无碰）
            auto point_passable = [&](const Vec3f &p) -> bool {
                auto gt = map_ptr_->getGridType(p);
                if (gt == rog_map::GridType::OCCUPIED || gt == rog_map::GridType::OUT_OF_MAP)
                    return false;
                if (param.frontend_in_known_free && gt == rog_map::GridType::UNKNOWN)
                    return false; // 在“仅已知区域”策略下，未知视作不可达
                return true;
            };

            for (int i = end_index; i >= first_id && mpc_goal_index < mpc_->MPC_HORIZON - 1; i--) {
                // 检查路径点安全性和连通性
                if (!point_passable(follow_path_[i])) continue;
                if (!astar_ptr_->CheckLineObstacleFree(follow_path_[first_id], follow_path_[i])) continue;
                
                i = i - 0.2 / path_dis_;  // 回退确保安全
                if (i <= first_id) break;
                
                // 生成长距离SFC
                Eigen::Matrix<double, Eigen::Dynamic, 4> long_planes;
                GenerateAPolytopeFromLine(follow_path_[first_id], follow_path_[i], long_planes, 1);
                if (long_planes.rows() > 0) {
                    // 为MPC后段步骤设置长SFC
                    for (int j = mpc_goal_index+1; j < mpc_->MPC_HORIZON; j++) {
                        mpc_->SetFSC(long_planes, j);
                    }
                    mpc_goal_index = mpc_->MPC_HORIZON;
                    
                    // 更新长SFC覆盖的目标范围
                    for (int k = first_id; k < follow_path_.size(); k++) {
                        if (mpc_->IsInFSC(follow_path_[k], long_planes)) goal_in_sfc = k;
                        else break;
                    }
                    break;
                }
            }
            
            // 为剩余MPC步骤设置原始SFC
            for (int i = mpc_goal_index+1; i < mpc_->MPC_HORIZON; i++) {
                if (last_planes.rows() > 0) mpc_->SetFSC(last_planes, i);
            }
        }
        // === MPC参考轨迹设置 ===
        mpc_goals_.clear();
        Eigen::Vector3d last_p_ref;

        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            // 计算MPC每步的参考位置索引
            int index = astar_index_ + i * ref_dis_;
            if (index >= goal_in_sfc) index = goal_in_sfc;      // 不超过SFC覆盖范围
            if (index >= follow_path_.size()) index = follow_path_.size() - 1;  // 不超过路径长度
            
            //计算参考速度
            Eigen::Vector3d v_r(0, 0, 0);
            if (i == 0) v_r = (follow_path_[index] - odom.p) / mpc_->MPC_STEP;         // 第一步：当前到目标
            else if (i == mpc_->MPC_HORIZON) v_r.setZero();                           // 最后一步：速度为零
            else v_r = (follow_path_[index] - last_p_ref) / mpc_->MPC_STEP;           // 中间步：点间速度
            last_p_ref = follow_path_[index];

            mpc_->SetGoal(follow_path_[index], v_r, Eigen::Vector3d::Zero(), i);
            // std::cout << "mpc goal " << i << ": " << follow_path_[index].transpose() << " v: " << v_r.transpose() << std::endl;
            mpc_goals_.push_back(follow_path_[index]);
            
        }
        //发布目标点可视化
        AstarPublish(mpc_goals_,3,0.1);

        // // === 偏航角控制 ===
        // if (astar_index_ < follow_path_.size() - 0.3 / path_dis_) {
        //     // 偏航角指向路径终点方向
        //     yaw_r_ = std::atan2(follow_path_.back().y()-odom.p.y(), follow_path_.back().x()-odom.p.x());
        // }
        // === 偏航角控制 ===
        // 航向指向路径切线方向：从当前位置在路径上取一个前瞻点，计算二维方向
        if (follow_path_.size() >= 2) {
            // 前瞻距离 L（米），可按需要调大/调小：大则更“看远”，小则更贴合当前弯道
            const double L = std::max(0.2, 3.0 * path_dis_);   // 例如 0.5 m 或 3*path_dis_
            const int lookahead_steps = std::max(1, int(L / path_dis_));

            const int i0 = std::min(astar_index_, int(follow_path_.size()) - 2);
            const int i1 = std::min(i0 + lookahead_steps, int(follow_path_.size()) - 1);

            const Eigen::Vector2d dir(
                follow_path_[i1].x() - follow_path_[i0].x(),
                follow_path_[i1].y() - follow_path_[i0].y()
            );

            if (dir.norm() > 1e-3) {
                yaw_r_ = std::atan2(dir.y(), dir.x());
            }
            // 若方向极短则保持当前 yaw_r_ 不变，避免抖动
        }
    } else { 
        have_path_ = false;
        static double yaw_now;
        if(have_path_ != last_have_path_) {
            ROS_WARN("\033[41;37m No valid path! Stay at current point! \033[0m");
            yaw_now = yaw_;
            last_have_path_ = have_path_;
        }
        // 无有效路径：保持在当前位置
        yaw_r_ = yaw_now; //debug
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->SetGoal(des.p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
        }
    }
    log_times_[2] = (ros::Time::now() - sfc_start).toSec() * 1000.0;  // 记录SFC生成耗时
}
void PlannerClass::EvaluateReplan()
{
    // evaluate whether need replan
    vec_Vec3f remain_path;
    int start_idx = std::min<int>(std::max<int>(astar_index_, 0), int(follow_path_.size()));
    remain_path.insert(remain_path.begin(), follow_path_.begin() + start_idx, follow_path_.end());
    if (!remain_path.empty() && astar_ptr_->CheckPathFree(remain_path) == false) replan_flag_ = true;
}

void PlannerClass::MPCSetGoal(const Eigen::Vector3d& goal_pos,const Eigen::Vector3d& goal_vel,const Eigen::Vector3d& goal_acc,double yaw)
{
    for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
        mpc_->SetGoal(goal_pos, goal_vel, goal_acc, i);
    }
    yaw_r_ = yaw;
}