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
    thrust_limit_ = param.thrust_limit;
    hover_esti_flag_ = param.hover_esti_flag;
    hover_perc_ = param.hover_perc;
    yaw_gain_ = param.yaw_gain;
    yaw_ctrl_flag_ = param.yaw_ctrl_flag;
    yaw_ki_ = param.yaw_ki;
    yaw_rate_limit_ = param.yaw_rate_limit;
    yaw_i_limit_ = param.yaw_i_limit;

    odom_data.vel_in_body = param.odom_vel_in_body;

    goal_p_ = Eigen::Vector3d(param.goal_x, param.goal_y, param.goal_z);

    path_dis_ = param.path_dis;
    ref_dis_ = param.ref_dis;
    planning_horizon_ = param.planning_horizon;
    sim_mode_ = param.simu_flag;

    Gravity_ << 0, 0, 9.81;

    thrust_ = hover_perc_;

    thr2acc_ = 9.81 / thrust_;


    std::string file = ros::package::getPath("ipc") + "/config";
    write_time_.open((file+"/time_consuming.csv"), std::ios::out | std::ios::trunc);
    write_data_.open((file+"/log_data.csv"), std::ios::out | std::ios::trunc);
    write_time_ << "mapping(ms)" << ", " << "replan(ms)" << ", " << "sfc(ms)" << ", " << "mpc(ms)" << ", " << "df(ms)" << ", " <<std::endl;

    write_data_ << "time(ms)" << ", " << "vel_x" << ", " << "vel_y" << ", " << "vel_z" << ", " <<"vel_norm" << ", "
                << "mpc_x" << ", " << "mpc_y" << ", " << "mpc_z" << ", "
                << "odom_x" << ", " << "odom_y" << ", " << "odom_z"
                << std::endl;
    
    mpc_   = std::make_shared<MPCPlannerClass>(nh);
    
    // 初始化 ROGMap - 从 ROS 参数服务器读取配置文件路径
    std::string config_path;
    if (!nh.getParam("config_path", config_path)) {
        // 如果参数不存在,使用默认路径
        ROS_ERROR("config_path parameter not found");
    } else {
        ROS_INFO("ROGMap config path from launch file: %s", config_path.c_str());
    }

    map_ptr_ = std::make_shared<rog_map::ROGMapROS>(nh, config_path, map_log_time_ms_);
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

// 【新增】启动规划线程
    thread_running_ = true;
    planning_thread_ = std::thread(&PlannerClass::PlanningThreadFunc, this);
    
    ROS_INFO("Planning thread started.");

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
            new_goal_flag_ = true;

            if (!param.use_waypoint_sequence)
            {
                goal_p_ = goal_data.new_goal;
                goal_p_.z() = std::max(param.goal_z, goal_p_.z()); // 确保目标点的 z 不低于 param.goal_z
                goal_reached_ = false;
                ROS_INFO("[px4ctrl] Manual goal mode. New clicked goal: (%.2f, %.2f, %.2f)",
                         goal_p_.x(), goal_p_.y(), goal_p_.z());
            }
            else
            {
                // 只有到达了目标点，才能切换到下一个目标
                if(goal_reached_)
                {
                    change_goal ++;  // 切换目标标志
                    if(change_goal == 5) change_goal = 1;
                    goal_reached_ = false;  // 重置到达标志
                    ROS_INFO("[px4ctrl] Goal reached! Switching target mode...");
                }

                // 根据 change_goal 设置目标点。外部点击只作为“切换到下一个预设点”的触发信号。
                if(change_goal == 1)
                {
                    goal_p_.x() = param.goal_x_1;
                    goal_p_.y() = param.goal_y_1;
                    goal_p_.z() = param.goal_z_1;
                    ROS_INFO("[px4ctrl] Waypoint sequence mode - Mode 1: (%.2f, %.2f, %.2f)",
                             goal_p_.x(), goal_p_.y(), goal_p_.z());
                }
                else if(change_goal == 2)
                {
                    goal_p_.x() = param.goal_x_2;
                    goal_p_.y() = param.goal_y_2;
                    goal_p_.z() = param.goal_z_2;
                    ROS_INFO("[px4ctrl] Waypoint sequence mode - Mode 2: (%.2f, %.2f, %.2f)",
                             goal_p_.x(), goal_p_.y(), goal_p_.z());
                }
                else if(change_goal == 3)
                {
                    goal_p_.x() = param.goal_x_3;
                    goal_p_.y() = param.goal_y_3;
                    goal_p_.z() = param.goal_z_3;
                    ROS_INFO("[px4ctrl] Waypoint sequence mode - Mode 3: (%.2f, %.2f, %.2f)",
                             goal_p_.x(), goal_p_.y(), goal_p_.z());
                }
                else if(change_goal == 4)
                {
                    goal_p_.x() = 0.0;
                    goal_p_.y() = 0.0;
                    goal_p_.z() = param.goal_z;
                    ROS_INFO("[px4ctrl] Waypoint sequence mode - Mode 4: (%.2f, %.2f, %.2f)",
                             goal_p_.x(), goal_p_.y(), goal_p_.z());
                }
            }
        }
        last_goal = goal_data.new_goal;
        goal_data.recv_new_msg = false;
    }
    
    // 检测无人机是否到达目标点
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if(!goal_reached_)
        {
            double distance_to_goal = (odom_data.p - goal_p_).norm();
            if(distance_to_goal < goal_reach_threshold_)
            {
                goal_reached_ = true;
                ROS_INFO("[px4ctrl] Goal reached! Distance: %.2f m. Ready to receive next goal.", distance_to_goal);
            }
        }
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

    if (sim_mode_) {
        rc_dy_data.enter_hover_mode = (state == MANUAL_CTRL) && odom_is_received(now_time);
        // In simulation command mode is always enabled; do not treat every
        // pending goal as a repeated "enter command mode" edge.
        rc_dy_data.enter_command_mode = false;
        rc_dy_data.is_hover_mode = odom_is_received(now_time);
        rc_dy_data.is_command_mode = true;
        rc_dy_data.toggle_reboot = false;
        state_data.current_state.connected = true;
        state_data.current_state.mode = "OFFBOARD";
    }
    else if(param.takeoff_land.no_RC) {
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
                if (!sim_mode_ && goal_is_received())
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
                if (!sim_mode_) {
                    toggle_offboard_mode(true);
                }

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
                        ROS_ERROR("[px4ctrl] RC throttle channel ch[0] is not zero: %.3f",
                                  rc_data.ch[0]);
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
                if (!sim_mode_) {
                    toggle_offboard_mode(false);
                }

                ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
            }
            else if (rc_dy_data.is_command_mode && goal_is_received())
            {
                if (sim_mode_ || state_data.current_state.mode == "OFFBOARD")
                {
                    if (emergency_stop_flag_) {
                        // A fresh goal should allow the planner to retry from
                        // stable hover instead of getting stuck in emergency hover.
                        emergency_stop_flag_ = false;
                        path_blocked_flag_ = false;
                        corridor_generation_failed_ = false;
                        mpc_fail_count_ = 0;
                        ROS_WARN("[Planner] New goal received during emergency hover. Retry planning.");
                    }
                    state = CMD_CTRL;
                    des = get_goal_des(get_goal_position());
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
                if (!sim_mode_) {
                    toggle_offboard_mode(false);
                }

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
                des = get_goal_des(get_goal_position());
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
    if (emergency_stop_flag_ && state == CMD_CTRL)
    {
        state = AUTO_HOVER;
        reset_goal_flag();
        set_hov_with_odom();
        des = get_hover_des();
        ROS_ERROR_THROTTLE(1.0, "[Planner] Emergency protection active. Switch CMD_CTRL -> AUTO_HOVER.");
    }

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
            if (emergency_stop_flag_) {
                state = AUTO_HOVER;
                reset_goal_flag();
                set_hov_with_odom();
                des = get_hover_des();
                MPCSetGoal(des.p, des.v, des.a, des.yaw);
                ROS_ERROR_THROTTLE(1.0, "[Planner] Emergency protection active after CMD update. Force hover reference.");
            }
        }
        MpcCalculate(odom_data,imu_data,u);
    }

        // if (new_goal_flag_) {
        //     // 新目标点：执行完整路径规划（扩展模式）
        //     new_goal_flag_ = false;
        //     replan_flag_ = false;
        //     PathReplan(odom_data.p,goal_p_);
        // }
        // if (new_goal_flag_ == false && replan_flag_) {
        //     // 障碍物触发：执行局部重规划（非扩展模式）
        //     replan_flag_ = false;
        //     PathReplan(odom_data.p,goal_p_);
        // }


    // Eigen::Matrix<double, Eigen::Dynamic, 4> planes;
    // // GenerateAPolytopeFromPoint(odom_data.p,planes, 0);
    // Eigen::Vector3d next_pt = Eigen::Vector3d(odom_data.p.x() + 0.5f, odom_data.p.y() , odom_data.p.z());
    // GenerateAPolytopeFromLine(odom_data.p,next_pt,planes, 0);

    // ROS_INFO_THROTTLE(1,"odom_vel norm: %.2f",odom_data.v.norm());
    // STEP4: publish control commands to mavros
    if (!sim_mode_) {
        publish_bodyrate_ctrl(u, now_time);
    }

    // STEP5: Detect if the drone has landed
    land_detector(state, des, odom_data);
    // cout << takeoff_land.landed << " ";
    // fflush(stdout);
    WriteLogTime();
    WriteLogData();

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
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return new_goal_flag_;
}
void PlannerClass::reset_goal_flag()
{
    std::lock_guard<std::mutex> lock(goal_mutex_);
    new_goal_flag_ = false;
}
bool PlannerClass::consume_new_goal(Eigen::Vector3d &goal_out)
{
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!new_goal_flag_) {
        return false;
    }
    new_goal_flag_ = false;
    goal_out = goal_p_;
    return true;
}

Eigen::Vector3d PlannerClass::get_goal_position()
{
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return goal_p_;
}

bool PlannerClass::is_goal_reached()
{
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return goal_reached_;
}

void PlannerClass::set_log_time(size_t idx, double value_ms)
{
    if (idx >= log_times_.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_times_[idx] = value_ms;
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
    const double map_log_ms = map_log_time_ms_.exchange(0.0);
    std::array<double, 4> log_snapshot;
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_snapshot = log_times_;
        log_times_.fill(0.0);
    }
    write_time_ << map_log_ms << ", ";
    for (double val : log_snapshot) {
        write_time_ << val << ", ";
    }
    write_time_ << std::endl;
}

void PlannerClass::WriteLogData(void) {

    write_data_ << ros::Time::now().toSec() << ", ";
    write_data_ << odom_data.v.x() << ", " << odom_data.v.y() << ", " << odom_data.v.z() << ", "<<odom_data.v.norm() << ", ";
    write_data_ << mpc_pos_.x() << ", " << mpc_pos_.y() << ", " << mpc_pos_.z() << ", ";
    write_data_ << odom_data.p.x() << ", " << odom_data.p.y() << ", " << odom_data.p.z()<< ", ";
    write_data_ << std::endl;
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

bool PlannerClass::MpcCalculate(const Odom_Data_t& odom,const Imu_Data_t& imu, Controller_Output_t& u)
{
    // calculate model predict control algorithm
    ros::Time mpc_start = ros::Time::now();
    mpc_->SetStatus(odom.p, odom.v, odom.a);
    bool success_flag = mpc_->Run();
    set_log_time(2, (ros::Time::now() - mpc_start).toSec() * 1000.0);

    Eigen::Vector3d u_optimal, p_optimal, v_optimal, a_optimal, u_predict;
    Eigen::MatrixXd A1, B1;
    Eigen::VectorXd x_optimal = mpc_->X_0_;
    if (success_flag) {
        ROS_INFO_THROTTLE(1,"MPC SUCCESS");
        const bool emergency_was_active = emergency_stop_flag_;
        mpc_fail_count_ = 0;
        if (!path_blocked_flag_ && !corridor_generation_failed_) {
            emergency_stop_flag_ = false;
            if (emergency_was_active) {
                ROS_WARN("[Planner] Emergency cleared. AUTO_HOVER is stable and ready for a new goal.");
            }
        }
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

        mpc_pos_ = p_optimal;
        if (!perfect_simu_flag_) CmdPublish(odom.p, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);
        mpc_->UpdateOutputHistory(u_optimal);

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
        mpc_fail_count_++;
        if (mpc_fail_count_ >= mpc_fail_limit_) {
            emergency_stop_flag_ = true;
            ROS_ERROR_THROTTLE(1.0, "[Planner] MPC failed %d times continuously. Trigger emergency hover.", mpc_fail_count_);
        }
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
        mpc_->UpdateOutputHistory(u_optimal);
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
    set_log_time(3, (ros::Time::now() - df_start).toSec() * 1000.0);
    return success_flag;
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
    Vec3f goal_pos  = Vec3f(goal.x(), goal.y(), goal.z());

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
        AstarPublish(waypoints_, 1, 0.2);
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
    msg.pose.position.x = goal.x();
    msg.pose.position.y = goal.y();
    msg.pose.position.z = goal.z();   
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

bool PlannerClass::GenerateAPolytopeFromLine(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    // 使用 CorridorGenerator 的 GeneratePolytopeFromLine 方法
    super_utils::Line seed_line = std::make_pair(
        super_utils::Vec3f(p1.x(), p1.y(), p1.z()),
        super_utils::Vec3f(p2.x(), p2.y(), p2.z())  
    );
    
    geometry_utils::Polytope polytope;
    bool success = corridor_gen_->GeneratePolytopeFromLine(seed_line, polytope);
    
    if (success) {
        if(index == 0) 
        {
            vis_ptr_->vizCurSfc(polytope);
            
        }
        planes = polytope.GetPlanes();
        return planes.rows() > 0;
    } else {
        // 失败时返回轴对齐边界盒（与原逻辑一致）
        ROS_WARN_THROTTLE(1.0, "[Planner] GeneratePolytopeFromLine failed, returning axis-aligned bounding box");
        planes.resize(0, 4);
        return false;
    }
}

bool PlannerClass::GenerateAPolytopeFromPoint(Eigen::Vector3d pos, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    // 使用 CorridorGenerator 的 GeneratePolytopeFromPoint 方法
    // 使用GeneratePolytopeFromPoint
    super_utils::Vec3f point = super_utils::Vec3f(pos.x(), pos.y(), pos.z());
    
    geometry_utils::Polytope polytope;
    bool success = corridor_gen_->GeneratePolytopeFromPoint(point, polytope);
    
    if (success) {
        // ROS_INFO_THROTTLE(1.0, "[Planner] GeneratePolytopeFromPoint succeeded.");
        if(index == 0) vis_ptr_->vizCurSfc(polytope);
        planes = polytope.GetPlanes();
        return planes.rows() > 0;
    } else {
        // 失败时返回轴对齐边界盒（与原逻辑一致）
        ROS_WARN_THROTTLE(1.0, "[Planner] GeneratePolytopeFromLine failed, returning axis-aligned bounding box");
        planes.resize(0, 4);
        return false;
    }
}
void PlannerClass::CmdMode(const Odom_Data_t& odom,const Desired_State_t& des)
{
        ros::Time t_start = ros::Time::now();  // 记录总执行开始时间
        bool triggered_new_plan = false;
        // 1. 触发判断 (仅仅是设置标志位，耗时几乎为0)
        Eigen::Vector3d new_goal_pos;
        if (consume_new_goal(new_goal_pos)) {
            // 新目标到来，强制触发一次
            triggered_new_plan = true;
            std::lock_guard<std::mutex> lk(data_mutex_);
            thread_start_pt_ = odom.p;
            thread_goal_pt_ = new_goal_pos;
            trigger_replan_flag_ = true;
            plan_cv_.notify_one();
        } else {
            // 常规检查
            EvaluateReplan();
        }

        // 2. 执行 SFC 和 MPC (必须加锁保护 follow_path_)
        {
            // 加上大括号限制锁的范围，尽快释放
            std::lock_guard<std::mutex> lock(path_mutex_); 

            // The first cycle after a new goal arrives usually runs before the
            // async planner has produced the first valid path. Hold the current
            // pose for one control cycle instead of treating it as an emergency.
            if (follow_path_.empty() && (triggered_new_plan || is_planning_)) {
                yaw_r_ = yaw_;
                for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                    mpc_->SetGoal(odom.p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
                }
                return;
            }

            // 调用 SetSFCAndGoal 时，它内部会读取 follow_path_
            // 此时 follow_path_ 是线程安全的
            if (!SetSFCAndGoal(odom, des)) {
                emergency_stop_flag_ = true;
            }
        }
}

bool PlannerClass::SetSFCAndGoal(const Odom_Data_t& odom, const Desired_State_t& des)
{
    // === 安全飞行走廊(SFC)生成和MPC目标设置 ===
    // 本函数的职责：
    // 1) 基于当前跟踪路径 follow_path_ 与当前位置 odom.p，生成若干 SFC（Safe Flight Corridor）。
    //    SFC 用若干平面组成的凸多面体来约束 MPC 的未来位置，确保优化解在安全空间内。
    // 2) 依据 SFC 覆盖范围与路径采样，为 MPC 设置每个预测步的参考位置/速度。
    // 3) 同时设置偏航参考 yaw_r_ 用于下游姿态控制。
    // 关键变量解释：
    // - follow_path_: 由 A* + Floyd + 插值生成的稠密路径点（世界系）。
    // - astar_index_: 当前“路径最近点”的索引，用于确定跟踪起点。
    // - ref_dis_: MPC 步之间沿路径的索引步长（类似采样间距的步数单位）。
    // - goal_in_sfc: 在当前 SFC 约束下，参考点允许到达的路径上“最远的索引”。
    // - GenerateAPolytopeFromPoint/Line: 基于点/线生成走廊多面体平面集合。
    // - mpc_->SetFSC(planes, i): 将第 i 个预测步的约束设置为 planes 所定义的凸多面体。
    // 注意：SFC 太“紧”或“排斥当前状态/参考”会使求解器报告 Primal Infeasible。
    //       下面的流程通过“先点后线、逐步扩展、必要回退”的策略降低不一致风险。
    ros::Time sfc_start = ros::Time::now();
    corridor_generation_failed_ = false;
    if (follow_path_.size() > 0) { // 存在有效路径
        have_path_ = true;
        last_have_path_ = true;

        // Step-A: 寻找路径上距离当前位置最近的点，作为跟踪起点
        //         非新路径只允许索引向前推进，避免最近点搜索抖回旧路径段。
        double min_dis = 10000.0;
        for (int i = 0; i < static_cast<int>(follow_path_.size()); i++) {
            double dis = (odom.p - follow_path_[i]).norm();
            if (dis < min_dis) {
                min_dis = dis;
                astar_index_ = i;  // 记录最近点索引
            }
        }

        // Step-B: 计算走廊与覆盖范围（goal_in_sfc）
        //         根据当前是否接近末端、以及初始点生成的多面体，逐步扩展 SFC。
        //         goal_in_sfc 用于约束参考生成不超出安全覆盖。
        int corridor_seed_idx = astar_index_;
        while (corridor_seed_idx < static_cast<int>(follow_path_.size()) &&
               !astar_ptr_->CheckPointFree(follow_path_[corridor_seed_idx], true)) {
            corridor_seed_idx++;
        }

        if (corridor_seed_idx >= static_cast<int>(follow_path_.size())) {
            ROS_ERROR_THROTTLE(1.0, "[Planner] Path exists on prob map but no point escapes inflated obstacle. Hold current pose.");
            yaw_r_ = yaw_;
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetGoal(odom.p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
            set_log_time(1, (ros::Time::now() - sfc_start).toSec() * 1000.0);
            return false;
        }

        int goal_in_sfc = corridor_seed_idx;
        if (follow_path_.size() - astar_index_ <= ref_dis_) { 
            // Case-1: 接近路径终点 → 在当前位置生成“点型”SFC，约束整条 horizon。
            // 该策略可防止末端“跳出安全区域”，简化为单一走廊约束。
            goal_in_sfc = follow_path_.size();
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes;
            if (!GenerateAPolytopeFromPoint(odom.p, planes, 0) || planes.rows() == 0) {
                corridor_generation_failed_ = true;
            }
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetFSC(planes, i);  // 为整个MPC预测地平线设置相同SFC
            }
        } else { 
            // Case-2: 正常路径跟踪 → 先用“点型”SFC保护起始段，再尝试生成更长的“线型”SFC覆盖后段。
            // 设计动机：
            //   - 点型 SFC 易确保当前状态在走廊内，降低 infeasible 风险；
            //   - 在线型 SFC 扩展时，逐段验证连通性与安全，尽量扩大可行域。
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes, last_planes;
            // GenerateAPolytopeFromLine(follow_path_[corridor_seed_idx], follow_path_[corridor_seed_idx], planes, 0);
            if (!GenerateAPolytopeFromPoint(follow_path_[corridor_seed_idx], planes, 0) || planes.rows() == 0) {
                corridor_generation_failed_ = true;
            }
            last_planes = planes;
            int init_num = 0;
            
            // 检查当前位置是否在初始SFC内：若不在，则前段预测步需要“初始化跳过”几步，避免一开始就不可行。
            if (mpc_->IsInFSC(odom.p, planes) == false) { 
                init_num = std::ceil((odom.p - follow_path_[corridor_seed_idx]).norm() / path_dis_ / ref_dis_);
                ROS_INFO("\033[35m UAV is out sfc! init num is %d \033[0m", init_num);
            }
            
            // 找到第一个点型 SFC 能覆盖的最远路径点 first_id（作为“短走廊”的终点）
            int first_id = corridor_seed_idx, mpc_goal_index = 0;
            for (int i = first_id+1; i < follow_path_.size(); i++) { 
                if (mpc_->IsInFSC(follow_path_[i], planes)) {
                    first_id = i;
                    goal_in_sfc = i;
                } else {
                    // 回退一点确保安全：避免刚好落在边界外导致下一步不可行
                    first_id -= 0.2 / path_dis_;
                    if (first_id < corridor_seed_idx) first_id = corridor_seed_idx;
                    break;
                }
            }
            
            // 计算第一个 SFC 在 MPC 地平线中的终止索引（从 init_num 开始到 mpc_goal_index 为“短走廊”步）
            mpc_goal_index = (first_id - corridor_seed_idx) / ref_dis_ + init_num + 1;
            assert(first_id >= corridor_seed_idx);
            
            // 为 MPC 前段步骤设置点型 SFC：先保障“短走廊”段在安全内
            for (int i = init_num; i <= mpc_goal_index && i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetFSC(planes, i);
            }
            
            // 尝试生成更长的“线型”SFC覆盖 MPC 剩余地平线：
            // - 先选潜在的远端索引 end_index
            // - 用点可达检查 + 线段无碰检查过滤不安全的终点
            // - 生成线型走廊并覆盖后段；若失败则回退为点型走廊
            int end_index = first_id + (mpc_->MPC_HORIZON-mpc_goal_index) * ref_dis_;
            if (end_index >= follow_path_.size()) end_index = follow_path_.size() - 1;

            for (int i = end_index; i >= first_id && mpc_goal_index < mpc_->MPC_HORIZON - 1; i--) {
                // 检查路径点安全性和连通性：逐候选点向前回退，寻找可用的线型走廊终点
                if (!astar_ptr_->CheckPointFree(follow_path_[i])) continue;
                if (!astar_ptr_->CheckLineObstacleFree(follow_path_[first_id], follow_path_[i])) continue;
                
                i = i - 0.2 / path_dis_;  // 回退确保安全，避免刚性边界造成不可行
                if (i <= first_id) break;
                
                // 生成长距离线型 SFC：用首尾两点作为“种子线”生成更大可行域
                Eigen::Matrix<double, Eigen::Dynamic, 4> long_planes;
                ros::Time gen_start = ros::Time::now();
                GenerateAPolytopeFromLine(follow_path_[first_id], follow_path_[i], long_planes, 0);
                // log_times_[2] += (ros::Time::now() - gen_start).toSec() * 1000.0;
                //ROS_INFO( "[Planner] Long SFC generation time: %.2f ms", (ros::Time::now() - gen_start).toSec() * 1000.0);
                
                // ROS_INFO("Generated long SFC from index %d to %d", first_id, i);
                if (long_planes.rows() > 0) {
                    // 为 MPC 后段步骤设置长 SFC，并更新覆盖范围 goal_in_sfc（用于参考点不越界）
                    for (int j = mpc_goal_index+1; j < mpc_->MPC_HORIZON; j++) {
                        mpc_->SetFSC(long_planes, j);
                    }
                    mpc_goal_index = mpc_->MPC_HORIZON;
                    
                    // 更新长SFC覆盖的目标范围：在 long_planes 内的最后一个路径索引作为新的 goal_in_sfc
                    for (int k = first_id; k < follow_path_.size(); k++) {
                        if (mpc_->IsInFSC(follow_path_[k], long_planes)) goal_in_sfc = k;
                        else break;
                    }
                    break;
                }
            }
            
            // 若未成功生成更长线型 SFC，则为剩余步设置原始点型 SFC（last_planes）作为保底约束
            for (int i = mpc_goal_index+1; i < mpc_->MPC_HORIZON; i++) {
                if (last_planes.rows() > 0) mpc_->SetFSC(last_planes, i);
            }
        }
        if (corridor_generation_failed_) {
            ROS_ERROR_THROTTLE(1.0, "[Planner] Corridor generation failed. Hold current pose.");
        }
        // === MPC参考轨迹设置（回退为 IPC 初版逻辑）===
        // 直接沿当前 follow_path_ 采样，不做新旧轨迹过渡拼接。
        mpc_goals_.clear();
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            int index = astar_index_ + i * ref_dis_;
            if (index >= goal_in_sfc) index = goal_in_sfc;
            if (index >= static_cast<int>(follow_path_.size())) index = static_cast<int>(follow_path_.size()) - 1;
            mpc_goals_.push_back(follow_path_[index]);
        }
        if (corridor_generation_failed_) {
            yaw_r_ = yaw_;
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetGoal(odom.p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
            set_log_time(1, (ros::Time::now() - sfc_start).toSec() * 1000.0);
            return false;
        }

        Eigen::Vector3d last_p_ref;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            Eigen::Vector3d v_r(0, 0, 0);
            if (i == 0) v_r = (mpc_goals_[i] - odom.p) / mpc_->MPC_STEP;
            else if (i == mpc_->MPC_HORIZON - 1) v_r.setZero();
            else v_r = (mpc_goals_[i] - last_p_ref) / mpc_->MPC_STEP;
            last_p_ref = mpc_goals_[i];
            mpc_->SetGoal(mpc_goals_[i], v_r, Eigen::Vector3d::Zero(), i);
        }

        //发布目标点可视化
        AstarPublish(mpc_goals_,3,0.1);
        // 偏航控制回退为 IPC 初版：朝向路径终点
        if (astar_index_ < static_cast<int>(follow_path_.size()) - 0.3 / path_dis_) {
            yaw_r_ = std::atan2(follow_path_.back().y() - odom.p.y(),
                                follow_path_.back().x() - odom.p.x());
        }
        set_log_time(1, (ros::Time::now() - sfc_start).toSec() * 1000.0);
        return true;
    } else { 
        // 无有效路径：
        // - 维持当前位置的参考，避免 MPC 追踪到不可预期点；
        // - yaw_r_ 保持当前航向。
        have_path_ = false;
        static double yaw_now;
        static Eigen::Vector3d pos_now;
        if(have_path_ != last_have_path_) {
            ROS_WARN("\033[41;37m No valid path! Stay at current point! \033[0m");
            yaw_now = yaw_;
            pos_now = odom.p;
            last_have_path_ = have_path_;
        }
        // 无有效路径：保持在当前位置
        yaw_r_ = yaw_now; //debug
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->SetGoal(pos_now, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
        } //debug 这里有问题
    }
    set_log_time(1, (ros::Time::now() - sfc_start).toSec() * 1000.0);  // 记录SFC生成耗时
    return false;
}
void PlannerClass::EvaluateReplan()
{
    // 0. 基础状态检查
    if (is_planning_) return; // 正在规划中，勿扰
    if (!have_path_ || is_goal_reached()) return;

    const Eigen::Vector3d current_goal = get_goal_position();

    bool need_plan = false;

    // 1. 【关键】将 A* 的局部地图中心更新到无人机当前位置
    // 这意味着 A* 内部的 "insideLocalMap" 判断将基于无人机当前周围的区域
    astar_ptr_->updateLocalMapCenter(odom_data.p);

    // 2. 提取【完整】的剩余路径
    vec_Vec3f remain_path;
    {
        // 快速加锁读取路径
        std::lock_guard<std::mutex> lock(path_mutex_); 
        
        // 获取当前位置之后的索引
        int start_idx = std::min<int>(std::max<int>(astar_index_, 0), int(follow_path_.size())); //检查的是follow_path_，这里有问题
        
        // 获取A*结束点
        double check_dist = 12.0; 
        int check_steps = std::ceil(check_dist / path_dis_);
        // 【修改点】不再截断，直接提取从当前位置到终点的所有点
        // 由于你的总路径 < 50m，这里的点数通常 < 500个，CheckPathFree 耗时极短
        remain_path.insert(remain_path.begin(), follow_path_.begin() + start_idx, follow_path_.end());
    }
    // 3. 在新视界下执行碰撞检测
    if (!remain_path.empty()) {
        // 加地图读锁 (C++14 写法)
        std::shared_lock<std::shared_timed_mutex> lock(map_ptr_->map_mutex_);
        path_blocked_flag_ = false;
        
        //  重规划检测 (保持原样，默认 true，检查膨胀地图) ---
        // 只要侵入膨胀层，就准备重规划，保持舒适距离
        if (astar_ptr_->CheckPathFree(remain_path, true) == false) { // 显式写 true 也可以
            need_plan = true;
            path_blocked_flag_ = true;
        }
    }
    // 4. 路径耗尽检测 
    // 只有在没触发避障规划时，才检查是否需要“续命”
    if (!need_plan && !remain_path.empty()) {
        
        // A. 计算当前手中剩余路径的物理长度
        double current_remain_dist = remain_path.size() * path_dis_;
        
        // B. 计算离全局终点的直线距离
        double dist_to_global_goal = (current_goal - odom_data.p).norm();

        // C. 触发条件：
        //    1. 剩余路径不够长了 (例如只剩 15米，大概够飞 3-5秒)
        //    2. 且 离终点还很远 (说明当前的 remain_path 只是局部路径，并未通向终点)
        //       (判定标准：终点距离 比 剩余路径 长出 3米以上)
        double replenish_thresh = 5.0; 
        
        if (current_remain_dist < replenish_thresh && dist_to_global_goal > (current_remain_dist + 0.2)) {
            need_plan = true;
            // ROS_INFO_THROTTLE(1.0, "[Planner] Path running out (%.1fm left), extending horizon...", current_remain_dist);
        }
    }

    // --- 3. 执行唤醒 (Trigger Execution) ---
    if (need_plan) {
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            thread_start_pt_ = odom_data.p; 
            thread_goal_pt_ = current_goal;
            trigger_replan_flag_ = true;
        }
        plan_cv_.notify_one(); // 唤醒规划线程！
        
    }
}

void PlannerClass::MPCSetGoal(const Eigen::Vector3d& goal_pos,const Eigen::Vector3d& goal_vel,const Eigen::Vector3d& goal_acc,double yaw)
{
    for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
        mpc_->SetGoal(goal_pos, goal_vel, goal_acc, i);
    }
    yaw_r_ = yaw;
}

bool PlannerClass::CommitFallbackPathToSafePoint(const Eigen::Vector3d &current_pos,
                                                 Eigen::Vector3d &safe_point)
{
    vec_Vec3f path_copy;
    int start_idx = 0;
    {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        if (follow_path_.empty()) {
            return false;
        }
        path_copy = follow_path_;
        start_idx = std::min(std::max(astar_index_, 0), std::max(int(follow_path_.size()) - 1, 0));
    }

    int first_blocked_idx = -1;
    {
        std::shared_lock<std::shared_timed_mutex> map_lock(map_ptr_->map_mutex_);
        for (int i = start_idx; i < static_cast<int>(path_copy.size()); ++i) {
            if (!astar_ptr_->CheckPointFree(path_copy[i], true)) {
                first_blocked_idx = i;
                break;
            }
        }
    }

    if (first_blocked_idx <= 0) {
        return false;
    }

    const int backoff_steps = std::max(1, static_cast<int>(std::ceil(0.5 / std::max(path_dis_, 1e-3))));
    int safe_idx = std::max(start_idx, first_blocked_idx - backoff_steps);

    {
        std::shared_lock<std::shared_timed_mutex> map_lock(map_ptr_->map_mutex_);
        while (safe_idx >= start_idx && !astar_ptr_->CheckPointFree(path_copy[safe_idx], true)) {
            --safe_idx;
        }
    }

    if (safe_idx < start_idx) {
        return false;
    }

    vec_Vec3f trimmed_path(path_copy.begin(), path_copy.begin() + safe_idx + 1);
    if (trimmed_path.empty()) {
        return false;
    }

    safe_point = trimmed_path.back();
    {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        follow_path_ = trimmed_path;
        astar_index_ = 0;
        path_version_++;
        path_blocked_flag_ = false;
        corridor_generation_failed_ = false;
    }

    AstarPublish(trimmed_path, 2, path_dis_);
    return true;
}

void PlannerClass::PlanningThreadFunc()
{
    while (thread_running_)
    {
        // 1. 等待触发信号
        std::unique_lock<std::mutex> lk(data_mutex_);
        plan_cv_.wait(lk, [this]{ return trigger_replan_flag_.load() || !thread_running_; });
        
        if (!thread_running_) break;

        // 取消触发标志，标记正在规划
        trigger_replan_flag_ = false;
        is_planning_ = true;

        // 复制起点和终点（数据快照），避免在规划时 odom 发生变化
        Eigen::Vector3d start_pt = thread_start_pt_;
        Eigen::Vector3d goal_pt = thread_goal_pt_;
        lk.unlock(); // 解锁，让主线程可以继续更新 odom

        // 2. 执行耗时的 A* 规划 (使用局部变量)
        vec_Vec3f temp_astar_path;
        vec_Vec3f temp_waypoints;
        vec_Vec3f temp_follow_path;

        // 调用 PathSearch (注意：PathSearch 内部只读 map，通常是线程安全的，除非 map 正在被大幅更新)
        // 这里的逻辑就是原 PathReplan 的核心逻辑
        
        //--- 动态视距截断逻辑 
        
        Eigen::Vector3d vec_to_goal = goal_pt - start_pt;
        Eigen::Vector3d target_pt;
        if (vec_to_goal.norm() > planning_horizon_) {
            target_pt = start_pt + vec_to_goal.normalized() * planning_horizon_;
        } else {
            target_pt = goal_pt;
        }

        // target_pt = goal_pt;
        bool success = false;
        Vec3f s_pos(start_pt.x(), start_pt.y(), start_pt.z());
        Vec3f g_pos(target_pt.x(), target_pt.y(), target_pt.z());
        ros::Time t0 = ros::Time::now();
        {
            ros::Time t1 = ros::Time::now();
            std::shared_lock<std::shared_timed_mutex> lock(map_ptr_->map_mutex_);
            success = PathSearch(s_pos, g_pos, temp_astar_path);
            ros::Time t2 = ros::Time::now();

            double wait_time = (t1 - t0).toSec() * 1000.0; // 等待耗时
            double calc_time = (t2 - t1).toSec() * 1000.0; // 计算耗时
            set_log_time(0, calc_time);
            // ROS_INFO("A* Wait: %.2f ms, Calc: %.2f ms", wait_time, calc_time);
        }


        if (success) {
            // Floyd 平滑
            astar_ptr_->FloydHandle(temp_astar_path, temp_waypoints);
            
            // 插值生成稠密路径
            if (!temp_waypoints.empty()) {
                temp_follow_path.push_back(temp_waypoints.front());
                for (size_t i = 0; i + 1 < temp_waypoints.size(); ++i) {
                    Vec3f p0 = temp_waypoints[i];
                    Vec3f p1 = temp_waypoints[i + 1];
                    Vec3f seg = p1 - p0;
                    double seg_len = seg.norm();
                    if (seg_len < 1e-6) continue;
                    int inter_num = static_cast<int>(std::floor(seg_len / path_dis_));
                    for (int k = 1; k < inter_num; ++k) {
                        temp_follow_path.push_back(p0 + seg * static_cast<double>(k) / inter_num);
                    }
                    temp_follow_path.push_back(p1);
                }
            }
            // 确保包含末端
            if(!temp_waypoints.empty()) temp_follow_path.push_back(temp_waypoints.back());

            // 3. 【关键】加锁更新全局路径
            // 只有这一瞬间会锁住主线程，耗时极短 (<0.1ms)
            std::lock_guard<std::mutex> path_lock(path_mutex_);
            astar_path_ = temp_astar_path;   // 用于可视化
            waypoints_ = temp_waypoints;     // 用于可视化
            follow_path_ = temp_follow_path; // 核心控制路径
            path_blocked_flag_ = false;
            corridor_generation_failed_ = false;
            astar_index_ = 0; 
            path_version_++;
            
            // 可视化 (可以在这里发布，或者在主线程发布)
            AstarPublish(astar_path_, 0, 0.1);
            AstarPublish(follow_path_, 2, path_dis_);
        } else {
             // Keep the previous committed path if replanning fails. Clearing
             // the path causes the controller to abruptly fall back to holding
             // position, which is perceived as a twitch.
             ROS_WARN("Async A* failed. Keep the current committed path.");
             if (path_blocked_flag_) {
                 Eigen::Vector3d safe_point;
                 if (CommitFallbackPathToSafePoint(start_pt, safe_point) &&
                     (safe_point - start_pt).norm() > 0.3) {
                     emergency_stop_flag_ = false;
                     ROS_WARN("[Planner] Replan failed. Continue on committed path to safe pre-obstacle point (%.2f m away) and retry.",
                              (safe_point - start_pt).norm());
                 } else {
                     emergency_stop_flag_ = true;
                     ROS_ERROR_THROTTLE(1.0, "[Planner] Replan failed while committed path is blocked and no forward safe point is available. Trigger emergency hover.");
                 }
             }
        }

        is_planning_ = false;
    }    
}

// void PlannerClass::LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//     local_pc_mutex_.lock();

//     // update astar map


//     local_pc_mutex_.unlock();

// }

PlannerClass::~PlannerClass()
{
// 1. 标志位设为 false，通知线程该退出了
    thread_running_ = false;

    // 2. 唤醒线程（如果它正卡在 wait 处），让它有机会检查 thread_running_ 标志
    plan_cv_.notify_all();

    // 等待规划线程安全退出
    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }

    ROS_INFO("PlannerClass destroyed and planning thread stopped.");
    // 关闭日志文件
    if (write_time_.is_open()) write_time_.close();
    if (write_data_.is_open()) write_data_.close();

    // 重置智能指针（可选，帮助明确析构顺序）
    astar_ptr_.reset();
    corridor_gen_.reset();
    mpc_.reset();
    map_ptr_.reset();
    vis_ptr_.reset();
}
