#include "planner.h"
#include "../include/polytope/emvp.hpp"

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

// void PlannerClass::TimerCallback(const ros::TimerEvent &)
// {
//     // === 基础状态检查 ===
//     if (!has_odom_flag_) return;  // 没有里程计数据则直接返回
//     if (mode_ == Manual) {        // 手动模式：发送零控制指令
//         BodyrateCtrlPub(Eigen::Vector3d(0,0,0), 0.05, ros::Time::now());
//         return;
//     }

//     timer_mutex_.lock();  // 加锁防止定时器回调冲突
    
//     ros::Time t_start = ros::Time::now();  // 记录总执行开始时间
//     ros::Time t_0 = odom_time_;            // 保存里程计时间戳
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
//     // === Command模式：自主导航控制 ===
//     if (mode_ == Command) {
//         // 路径规划触发逻辑
//         if (new_goal_flag_) {
//             // 新目标点：执行完整路径规划（扩展模式）
//             new_goal_flag_ = false;
//             replan_flag_ = false;
//             PathReplan(true);
//         }
//         if (new_goal_flag_ == false && replan_flag_) {
//             // 障碍物触发：执行局部重规划（非扩展模式）
//             replan_flag_ = false;
//             PathReplan(false);
//         }
//         log_times_[1] = (ros::Time::now() - t_start).toSec() * 1000.0;  // 记录规划耗时

//         SetSFCAndGoal();
//     }

//     // === Hover模式：悬停控制 ===
//     if (mode_ == Hover) {
//         for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
//             mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
//         }
//     }
    

//     WriteLogTime();  // 写入性能日志

//     timer_mutex_.unlock();  // 解锁
// }
void PlannerClass::SetSFCAndGoal(const Odom_Data_t& odom)
{
    // === 安全飞行走廊(SFC)生成和MPC目标设置 ===
    ros::Time sfc_start = ros::Time::now();
    if (follow_path_.size() > 0) { // 存在有效路径
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
            GenerateAPolytope(odom.p, odom.p, planes, 0);
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetFSC(planes, i);  // 为整个MPC预测地平线设置相同SFC
            }
        } else { 
            // 正常路径跟踪：寻找最大最长的SFC
            Eigen::Matrix<double, Eigen::Dynamic, 4> planes, last_planes;
            GenerateAPolytope(follow_path_[astar_index_], follow_path_[astar_index_], planes, 0); 
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
            
            for (int i = end_index; i >= first_id && mpc_goal_index < mpc_->MPC_HORIZON - 1; i--) {
                // 检查路径点安全性和连通性
                if (local_astar_->CheckPoint(follow_path_[i]) == false) continue;
                if (local_astar_->CheckLineObstacleFree(follow_path_[first_id], follow_path_[i]) == false) continue;
                
                i = i - 0.2 / path_dis_;  // 回退确保安全
                if (i <= first_id) break;
                
                // 生成长距离SFC
                Eigen::Matrix<double, Eigen::Dynamic, 4> long_planes;
                GenerateAPolytope(follow_path_[first_id], follow_path_[i], long_planes, 1);
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
            
            // 计算参考速度
            Eigen::Vector3d v_r(0, 0, 0);
            if (i == 0) v_r = (follow_path_[index] - odom.p) / mpc_->MPC_STEP;         // 第一步：当前到目标
            else if (i == mpc_->MPC_HORIZON) v_r.setZero();                           // 最后一步：速度为零
            else v_r = (follow_path_[index] - last_p_ref) / mpc_->MPC_STEP;           // 中间步：点间速度
            last_p_ref = follow_path_[index];
            
            // 设置MPC目标：位置、速度、加速度（零）
            mpc_->SetGoal(follow_path_[index], v_r, Eigen::Vector3d::Zero(), i);
            mpc_goals_.push_back(follow_path_[index]);
        }
        AstarPublish(mpc_goals_, 3, 0.1);  // 发布MPC目标点可视化

        // === 偏航角控制 ===
        if (astar_index_ < follow_path_.size() - 0.3 / path_dis_) {
            // 偏航角指向路径终点方向
            yaw_r_ = std::atan2(follow_path_.back().y()-odom.p.y(), follow_path_.back().x()-odom.p.x());
        }
    } else { 
        // 无有效路径：保持在初始位置
        yaw_r_ = 0.0;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
        }
    }
    log_times_[2] = (ros::Time::now() - sfc_start).toSec() * 1000.0;  // 记录SFC生成耗时
}
void PlannerClass::MpcCalculate(const Odom_Data_t& odom,const Eigen::Vector3d& imu_a)
{
    // === MPC优化求解 ===
    ros::Time mpc_start = ros::Time::now();
    mpc_->SetStatus(odom.p, odom.v, odom.a);  // 设置当前状态 debug
    bool success_flag = mpc_->Run();             // 执行MPC优化
    log_times_[3] = (ros::Time::now() - mpc_start).toSec() * 1000.0;  // 记录MPC求解耗时

    // === 控制指令生成和发布 ===
    Eigen::Vector3d u_optimal, p_optimal, v_optimal, a_optimal, u_predict;
    Eigen::MatrixXd A1, B1;
    Eigen::VectorXd x_optimal = mpc_->X_0_;
    
    if (success_flag) {
        // MPC求解成功：使用优化结果
        last_mpc_time_ = ros::Time::now();
        
        // 考虑控制延迟，前向预测到实际执行时刻
        for (int i = 0; i <= ctrl_delay_/mpc_->MPC_STEP; i++) {
            mpc_->GetOptimCmd(u_optimal, i);                    // 获取控制指令
            mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);         // 获取系统矩阵
            x_optimal = A1 * x_optimal + B1 * u_optimal;       // 状态预测
        }
        mpc_ctrl_index_ = ctrl_delay_/mpc_->MPC_STEP;

        // 提取预测状态
        p_optimal << x_optimal(0,0), x_optimal(1,0), x_optimal(2,0);  // 位置
        v_optimal << x_optimal(3,0), x_optimal(4,0), x_optimal(5,0);  // 速度
        a_optimal << x_optimal(6,0), x_optimal(7,0), x_optimal(8,0);  // 加速度
        
        // 发布控制指令（根据仿真标志选择位置或速度控制）
        if (!perfect_simu_flag_) CmdPublish(odom.p, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);

        // 生成并发布MPC预测轨迹用于可视化
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
        // MPC求解失败：使用上次的控制序列
        double delta_t = (ros::Time::now()-last_mpc_time_).toSec();
        if (delta_t >= mpc_->MPC_STEP) {
            mpc_ctrl_index_ += delta_t / mpc_->MPC_STEP;  // 更新控制索引
            last_mpc_time_ = ros::Time::now();
        }
        
        // 使用上次优化结果中的对应控制指令
        mpc_->GetOptimCmd(u_optimal, mpc_ctrl_index_);
        mpc_->SystemModel(A1, B1, mpc_->MPC_STEP);
        x_optimal = A1 * mpc_->X_0_ + B1 * u_optimal;
        
        p_optimal << x_optimal(0,0), x_optimal(1,0), x_optimal(2,0);
        v_optimal << x_optimal(3,0), x_optimal(4,0), x_optimal(5,0);
        a_optimal << x_optimal(6,0), x_optimal(7,0), x_optimal(8,0);
        
        if (!perfect_simu_flag_) CmdPublish(odom.p, v_optimal, a_optimal, u_optimal);
        else CmdPublish(p_optimal, v_optimal, a_optimal, u_optimal);
    }
    
    // === 底层控制指令转换和发布 ===
    ros::Time df_start = ros::Time::now();
    estimateThrustModel(imu_a);       // 推力模型估计debug
    a_optimal = a_optimal + Gravity_;   // 加上重力补偿
    ComputeThrust(a_optimal,odom.q);          // 计算推力指令
    ConvertCommand(a_optimal, u_optimal);  // 转换为角速度指令
    // BodyrateCtrlPub(rate_, thrust_, ros::Time::now());  // 发布底层控制指令

    // 维护推力历史队列（用于推力模型估计）
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thrust_));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    log_times_[4] = (ros::Time::now() - df_start).toSec() * 1000.0;  // 记录底层控制耗时
}
void PlannerClass::GenerateAPolytope(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    planes.resize(0, 4);
    ros::Time start_t = ros::Time::now();
    Eigen::Vector3d box_max(10, 10, 3), box_min(-10, -10, -0.5);

    Eigen::Matrix<double, 6, 4> bd;
    bd.setZero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;
    bd(0, 3) = -p1.x()-box_max.x();
    bd(1, 3) =  p1.x()+box_min.x();
    bd(2, 3) = -p1.y()-box_max.y();
    bd(3, 3) =  p1.y()+box_min.y();
    bd(4, 3) = -box_max.z();
    bd(5, 3) = +box_min.z();

    Polytope p;
    if (local_pc_.empty()) { // 障碍物点云为空，直接返回一个方块
        planes.resize(6, 4); // Ax + By + Cz + D = 0
        planes.row(0) <<  1,  0,  0, -p1.x()-box_max.x();
        planes.row(1) <<  0,  1,  0, -p1.y()-box_max.y();
        planes.row(2) <<  0,  0,  1, -box_max.z();
        planes.row(3) << -1,  0,  0,  p1.x()+box_min.x();
        planes.row(4) <<  0, -1,  0,  p1.y()+box_min.y();
        planes.row(5) <<  0,  0, -1,  box_min.z();
        return ; 
    }
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(local_pc_[0].data(), 3, local_pc_.size());

    bool success = emvp::emvp(bd, pp, p1, p2, p, sfc_dis_, false, 1);
    if (success) {
        if (index == 0) p.Visualize(sfc_pub_, "emvp", true);
        p.Visualize(sfc_pub_, "emvp", false);
        planes = p.GetPlanes();
    } else {
        p.Reset();
    }
}

void PlannerClass::PathReplan(bool extend, const Odom_Data_t& odom)
{
    // === 清空上次规划结果 ===
    astar_path_.clear();    // 清空A*原始路径
    waypoints_.clear();     // 清空关键路径点
    follow_path_.clear();   // 清空最终跟踪路径

    // === 设置起点和终点 ===
    Eigen::Vector3d start_p, end_p;
    start_p = odom.p;  // 起点设为当前位置
    // start_p = odom_p_ + odom_v_ * 0.1;  // 备选：考虑速度的预测起点
    
    // 根据extend参数决定是否重新设置地图中心
    if (extend) {
        // 扩展模式：重新设置A*地图中心为当前位置（Z=0表示2D规划）
        local_astar_->SetCenter(Eigen::Vector3d(odom.p.x(), odom.p.y(), 0.0));
    } else {
        // 重规划模式：保持现有地图中心
        // ROS_WARN("[MPC FSM]: Replan!");
    }
    
    // 使用当前障碍物点云和动态膨胀半径更新A*地图
    local_astar_->setObsVector(local_pc_, expand_dyn_);

    // === 目标点范围检查和调整 ===
    bool add_goal_flag = false;  // 标记是否需要添加原始目标点
    end_p = goal_p_;  // 终点初始设为目标点
    double delta_x = goal_p_.x() - odom.p.x();
    double delta_y = goal_p_.y() - odom.p.y();
    
    // 检查目标点是否超出局部地图范围
    if (std::fabs(delta_x) > map_upp_.x() || std::fabs(delta_y) > map_upp_.y()) {
        add_goal_flag = true;  // 标记需要后续添加原始目标点
        
        // 将终点调整到地图边界内，保持方向不变
        if (std::fabs(delta_x) > std::fabs(delta_y)) {
            // X方向距离更大，以X轴为主调整
            end_p.x() = odom.p.x() + (delta_x/std::fabs(delta_x)) * (map_upp_.x() - resolution_);
            end_p.y() = odom.p.y() + ((map_upp_.x() - resolution_)/std::fabs(delta_x)) * delta_y;
        } else {
            // Y方向距离更大，以Y轴为主调整
            end_p.x() = odom.p.x() + ((map_upp_.y() - resolution_)/std::fabs(delta_y)) * delta_x;
            end_p.y() = odom.p.y() + (delta_y/std::fabs(delta_y)) * (map_upp_.y() - resolution_);
        }
    }
    // start_p.z() = end_p.z(); // 仅2D路径搜索时使用
    // std::cout << "odom: " << odom_p_.transpose() << " end_p: " << end_p.transpose() << std::endl;

    // === 起点安全性检查和调整 ===
    int point_num = 8;  // 在起点周围生成8个候选点
    double r = expand_fix_ / 2;  // 初始搜索半径
    
    if (local_astar_->CheckStartEnd(start_p) == false) {
        // 起点在障碍物中，需要寻找安全的起点
        ROS_INFO("\033[41;37m start point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            // 在当前半径r内的圆周上生成8个候选点
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << start_p.x() + r*sin(M_PI*2*i/point_num), 
                      start_p.y() + r*cos(M_PI*2*i/point_num), 
                      start_p.z();
                double dis_min = 10000.0;
                double dis = (odom.p - pt).norm();
                // 找到距离当前位置最近的安全点
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    start_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change start goal! %f %f %f\033[0m", start_p.x(), start_p.y(), start_p.z());
                break;
            }
            r += expand_fix_ / 2;  // 扩大搜索半径
        }
    }
    
    // === 终点安全性检查和调整 ===
    r = expand_fix_ / 2;  // 重置搜索半径
    if (local_astar_->CheckStartEnd(end_p) == false) {
        // 终点在障碍物中，需要寻找安全的终点
        ROS_INFO("\033[41;37m end point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            // 在当前半径r内的圆周上生成8个候选点
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << end_p.x() + r*sin(M_PI*2*i/point_num), 
                      end_p.y() + r*cos(M_PI*2*i/point_num), 
                      end_p.z();
                double dis_min = 10000.0;
                double dis = (odom.p - pt).norm();
                // 找到距离当前位置最近的安全点
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    end_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change end goal! %f %f %f\033[0m", end_p.x(), end_p.y(), end_p.z());
                break;
            }
            r += expand_fix_ / 2;  // 扩大搜索半径
        }
    }

    // === A*路径搜索 ===
    bool search_flag = local_astar_->SearchPath(start_p, end_p);
    if (search_flag) {
        // 路径搜索成功
        
        // 获取A*原始路径并发布可视化
        local_astar_->GetPath(astar_path_);
        AstarPublish(astar_path_, 0, 0.1);

        // === Floyd路径优化 ===
        // 使用Floyd算法移除冗余拐点，生成关键路径点
        local_astar_->FloydHandle(astar_path_, waypoints_);
        // waypoints_.insert(waypoints_.begin(), odom_p_);  // 可选：添加当前位置为第一个点
        
        // 如果原始目标点安全且之前被调整过，则添加到路径末尾
        if (add_goal_flag && local_astar_->CheckPoint(goal_p_)) 
            waypoints_.push_back(goal_p_);
        AstarPublish(waypoints_, 1, 0.1);

        // === 路径密化处理 ===
        // 在关键路径点之间插入密集点，确保路径连续性
        for (int i = 0; i < waypoints_.size()-1; i++) {
            Eigen::Vector3d vector = waypoints_[i+1] - waypoints_[i];
            int num = vector.norm() / path_dis_; // 每path_dis_米插入一个点（通常0.1m）
            for (int j = 0; j < num; j++) {
                Eigen::Vector3d pt = waypoints_[i] + vector * j / num;
                follow_path_.push_back(pt);
            }
        }
        // 添加最后一个关键点
        follow_path_.push_back(waypoints_.back());
        AstarPublish(follow_path_, 2, path_dis_);
    } else {
        // 路径搜索失败，保持当前位置
        ROS_INFO("\033[41;37m No path! Stay at current point! \033[0m");
        follow_path_.push_back(odom.p);
    }

    // === 发布目标点消息 ===
    // 向其他节点发布最终的目标位置（路径终点）
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = follow_path_.back().x();
    msg.pose.position.y = follow_path_.back().y();
    msg.pose.position.z = follow_path_.back().z();    
    goal_pub_.publish(msg);

    // === 清理A*数据结构 ===
    ros::Time now = ros::Time::now();
    local_astar_->Reset(); // 重置A*数据，释放内存
    // std::cout << "astar reset time is: " << (ros::Time::now() - now).toSec()*1000 << " ms. " << std::endl;
}

void PlannerClass::PointCloudCorpAndSetMap(const Odom_Data_t& odom, PointCloud_Data_t& pc2)
{
    //要不要加锁
    // 记录处理开始时间，用于性能统计
    ros::Time now = ros::Time::now();
    
    // 清空上一帧的局部点云数据
    local_pc_.clear();
    // === 点云裁剪过滤 ===
    // 使用CropBox滤波器删除无用点云（超出局部地图范围的点）
    pcl::CropBox<pcl::PointXYZ> cb;
    // 设置裁剪盒子的最小边界：以当前位置为中心，向各方向扩展（地图大小-0.5）的距离
    // Z轴最小高度设为0.2米（地面以上）
    cb.setMin(Eigen::Vector4f(odom.p.x() - (map_upp_.x()-0.5), 
                              odom.p.y() - (map_upp_.y()-0.5), 
                              0.2, 1.0));
    // 设置裁剪盒子的最大边界
    cb.setMax(Eigen::Vector4f(odom.p.x() + (map_upp_.x()-0.5), 
                              odom.p.y() + (map_upp_.y()-0.5), 
                              map_upp_.z(), 1.0));
    cb.setInputCloud(pc2.static_cloud_);
    cb.filter(*pc2.static_cloud_);
    
    // === 体素降采样处理 ===
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3f pos = odom.p.cast<float>();  // 当前位置（未在后续代码中使用）
    
    // 设置体素网格大小为0.2x0.2x0.2米，降低点云密度
    vf.setLeafSize(0.2, 0.2, 0.2);
    vf.setInputCloud(pc2.static_cloud_);
    vf.filter(pc2.static_map_);  // 降采样后的结果存储在static_map_中
    
    // === 点云格式转换 ===
    // 将PCL格式的点云转换为Eigen::Vector3d格式，存储到local_pc_容器中
    // 这样便于后续A*算法和轨迹规划使用
    for(auto &point: pc2.static_map_.points) {
        local_pc_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // === A*地图更新和路径检查 ===
    static int obs_count = 0;  // 静态变量，记录检测到障碍物的连续次数（当前未使用）
    
    // 设置局部A*算法的地图中心为当前位置（Z轴设为0，表示2D规划）
    local_astar_->SetCenter(Eigen::Vector3d(odom.p.x(), odom.p.y(), 0.0));
    
    // 使用处理后的点云更新A*算法的障碍物地图
    // expand_fix_是固定的障碍物膨胀半径
    local_astar_->setObsVector(local_pc_, expand_fix_);
    
    // === 路径安全性检查 ===
    // 提取从当前执行点到路径终点的剩余路径段
    std::vector<Eigen::Vector3d> remain_path;
    remain_path.insert(remain_path.begin(), 
                      follow_path_.begin()+astar_index_, 
                      follow_path_.end());
    
    // 检查剩余路径是否仍然无障碍物，如果有障碍物则触发重规划
    if (local_astar_->CheckPathFree(remain_path) == false) 
        replan_flag_ = true;
    
    // === 注释掉的备选路径检查策略 ===
    // 这是一种基于连续检测次数的重规划策略（当前未启用）
    // bool flag = local_astar_->CheckPathFree(follow_path_);     // 检查整条路径是否自由
    // if (flag == false) obs_count++;  // 检测到障碍物，计数加1
    // else obs_count = 0;               // 路径自由，重置计数
    // if (obs_count >= 2) {             // 连续2次检测到障碍物才触发重规划
    //     obs_count = 0;
    //     replan_flag_ = true;
    // }
    
    // === 注释掉的栅格地图发布代码 ===
    // 以下代码用于发布占用栅格地图用于可视化（当前未启用）
    // local_astar_->GetOccupyPcl(cloud);
    // cloud.width = cloud.points.size();
    // cloud.height = 1;
    // cloud.is_dense = true;
    // sensor_msgs::PointCloud2 map_msg;
    // pcl::toROSMsg(cloud, map_msg);
    // map_msg.header.frame_id = "world";
    // gird_map_pub_.publish(map_msg);

    // 记录点云处理的耗时（毫秒），用于性能分析
    // log_times_[0]对应mapping时间
    log_times_[0] = (ros::Time::now() - now).toSec() * 1000.0;
}

// void PlannerClass::LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//     // 加锁保护局部点云数据，防止多线程访问冲突
//     local_pc_mutex_.lock();

//     // 记录处理开始时间，用于性能统计
//     ros::Time now = ros::Time::now();
    
//     // 清空上一帧的局部点云数据
//     local_pc_.clear();
    
//     // 将ROS消息格式的点云转换为PCL格式
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::fromROSMsg(*msg, cloud);

//     // === 多帧点云融合处理 ===
//     // 将当前帧点云加入历史点云队列
//     vec_cloud_.push_back(cloud);
//     // 保持队列大小不超过10帧，移除最旧的点云
//     if (vec_cloud_.size() > 10) vec_cloud_.pop_front();
    
//     // 重置静态点云容器
//     static_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
//     // 融合所有历史帧点云到静态点云中
//     for (int i = 0; i < vec_cloud_.size(); i++) *static_cloud_ += vec_cloud_[i];
    


//     // 解锁，允许其他线程访问局部点云数据
//     local_pc_mutex_.unlock();
// }

// void PlannerClass::RCInCallback(const mavros_msgs::RCInConstPtr& msg)
// {
//     rc_mutex_.lock();

//     if (simu_flag_) {
//         mode_ = Command;
//     } else {
//         static UAVMode_e mode_last = Manual;
//         static uint16_t takeoff_ch_last = 0;
//         if (msg->channels[4] > 1650 && msg->channels[4] < 1800 && mode_last != Command) {
//             mode_ = Command;
//             ROS_INFO("\033[1;32m[MPC FSM]: Manual or Hover --> Command.\033[0m");
//         }
//         if (mode_ == Command && msg->channels[4] > 1850) {
//             mode_ = Hover;
//             goal_p_ = odom_p_;
//             ROS_INFO("\033[32m[MPC FSM]: Command --> Hover. Pos is: %f %f %f\033[32m", goal_p_.x(), goal_p_.y(), goal_p_.z());
//         }
//         if (mode_ == Hover && takeoff_ch_last < 1500 && msg->channels[10] > 1500) {
//             goal_p_.z() = 0.0;
//             ROS_WARN("[MPC FSM]: Land! Pos is: %f %f %f", goal_p_.x(), goal_p_.y(), goal_p_.z());
//         }
        
//         takeoff_ch_last = msg->channels[10];
//         mode_last = mode_;
//     }

//     rc_mutex_.unlock();
// }

// void PlannerClass::GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
// {
//     goal_mutex_.lock();

//     static Eigen::Vector3d last_goal;
//     Eigen::Vector3d new_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

//     if (last_goal != new_goal && mode_ == Command) {
//         // goal_p_ << msg->pose.position.x, msg->pose.position.y, goal_p_.z(); // 2d path searching
//         goal_p_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//         if (goal_p_.z() > map_upp_.z() - 0.5) goal_p_.z() = map_upp_.z() - 0.5;
//         if (goal_p_.z() < 0.5) goal_p_.z() = 0.5;
//         new_goal_flag_ = true;
//     }
//     last_goal = new_goal;

//     goal_mutex_.unlock();
// }

// void PlannerClass::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
// {
//     odom_mutex_.lock();
//     odom_time_ = msg->header.stamp;
//     odom_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
//     odom_v_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 
//     odom_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
//                                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
//     odom_a_ = odom_q_ * Eigen::Vector3d(0, 0, 1) * (thrust_ * thr2acc_) - Gravity_; 
//     if (perfect_simu_flag_) {
//         odom_v_.setZero();
//         // odom_a_.setZero();
//     }
//     yaw_ = tf::getYaw(msg->pose.pose.orientation);
//     has_odom_flag_ = true;
//     odom_mutex_.unlock();
// }

// void PlannerClass::IMUCallback(const sensor_msgs::ImuConstPtr& msg)
// {
//     imu_mutex_.lock();
//     imu_a_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z; // body frame
//     Eigen::Matrix3d Rotate = odom_q_.toRotationMatrix().inverse();
//     imu_a_ = Rotate * imu_a_;
//     imu_mutex_.unlock();
// }