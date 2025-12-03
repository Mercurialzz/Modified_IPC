/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include <ros/ros.h>
#include "Eigen/Dense"
#include "vector"
#include "rog_map_ros/rog_map_ros1.hpp"
#include "queue"
#include "utils/header/type_utils.hpp"
#include <vis_interface/vis_interface.hpp>


namespace path_search {
    using namespace super_utils;

    // NOTE: 原实现 "1 >> 20" 等于 0，导致初始得分为 0 破坏 A* 逻辑，这里改为一个足够大的正数。
    constexpr double inf = 1e9;
    struct GridNode;
    typedef GridNode *GridNodePtr;

    struct GridNode {
        enum enum_state {
            OPENSET = 1,
            CLOSEDSET = 2,
            UNDEFINED = 3
        } state{UNDEFINED};

        int rounds{0};
        rog_map::Vec3i id_g;
        double total_score{inf}, distance_score{inf};
        double distance_to_goal{inf};
        GridNodePtr father_ptr{nullptr};
    };

    class NodeComparator {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2) {
            return node1->total_score > node2->total_score;
        }
    };

    class FrontierComparator {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2) {
            return node1->distance_to_goal > node2->distance_to_goal;
        }
    };

    const int ON_INF_MAP = (1 << 0);
    const int ON_PROB_MAP = (1 << 1);
    const int UNKNOWN_AS_OCCUPIED = (1 << 3);
    const int UNKNOWN_AS_FREE = (1 << 4);
    const int USE_INF_NEIGHBOR = (1 << 5);
    const int DONT_USE_INF_NEIGHBOR = (1 << 6);

    class Astar {

        rog_map::ROGMapROS::Ptr map_ptr_;
        vis_interface::VisInterface::Ptr vis_ptr_;

        // PathSearchConfig cfg_;
        const double tie_breaker_ = 1.0 + 1e-5;
        rog_map::vec_Vec3i sorted_pts;
        rog_map::vec_Vec3i neighbor_list;

        vector<GridNodePtr> grid_node_buffer_;

        int rounds_{0};

        static constexpr int DIAG = 0;
        static constexpr int MANH = 1;
        static constexpr int EUCL = 2;


        struct MissionData {
            rog_map::Vec3f start_pt;
            rog_map::Vec3f goal_pt;
            bool use_inf_map{false};
            bool use_prob_map{false};
            bool unknown_as_occ{false};
            bool unknown_as_free{false};
            bool use_inf_neighbor{false};
            double resolution;
            rog_map::Vec3i local_map_center_id_g;
            rog_map::Vec3f local_map_center_d;
            double mission_rcv_WT{0};
            rog_map::Vec3f local_map_max_d, local_map_min_d;
            std::mutex mission_mtx;
        } md_;

        /*Astar_param*/
        struct Astar_param
        {
            Vec3i map_voxel_num, map_size_i;
            bool visual_process;
            bool debug_visualization_en;
            bool allow_diag{false};
            int heu_type{0};
        } cfg_;


        void init_param(const ros::NodeHandle &nh)
        {
            vector<int> vox_;
            read_essential_param(nh, "rog_astar/map_voxel_num", vox_);
            read_essential_param(nh, "rog_astar/visual_process", cfg_.visual_process);
            read_essential_param(nh, "rog_astar/allow_diag", cfg_.allow_diag);
            read_essential_param(nh, "rog_astar/heu_type", cfg_.heu_type);
            read_essential_param(nh, "rog_astar/debug_visualization_en", cfg_.debug_visualization_en);

            cfg_.map_voxel_num = Vec3i(vox_[0], vox_[1], vox_[2]);
            cfg_.map_size_i = cfg_.map_voxel_num / 2;
            cfg_.map_voxel_num = cfg_.map_size_i * 2 + Vec3i::Constant(1);
        }

        template <typename TName, typename TVal>
        void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
        {
            if (nh.getParam(name, val))
            {
                // pass
                ROS_INFO_STREAM("Read param: " << name << " successfully, value: " << val);
            }
            else
            {
                ROS_ERROR_STREAM("Read param: " << name << " failed.");
                ROS_BREAK();
            }
        };

        double getHeu(GridNodePtr node1, GridNodePtr node2, int type = DIAG) const;

         int getLocalIndexHash(const rog_map::Vec3i &id_in) const;

        void posToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const ;

        void globalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const;

        bool insideLocalMap(const rog_map::Vec3f &pos) const;

        bool insideLocalMap(const rog_map::Vec3i &id_g) const;

        bool neighborHaveOne(const rog_map::GridType &type, const rog_map::Vec3i &src_id);

    RET_CODE setup(const rog_map::Vec3f &start_pt, const rog_map::Vec3f &goal_pt, const int &flag);

        void retrievePath(GridNodePtr current, vector<GridNodePtr> &path);

        void ConvertNodePathToPointPath(const vector<GridNodePtr> &node_path, rog_map::vec_Vec3f &point_path);

    public:

        Astar(
              const ros::NodeHandle &nh,
              const vis_interface::VisInterface::Ptr &vis_ptr,
              rog_map::ROGMapROS::Ptr rm);

        ~Astar() {
            // 释放动态分配的 GridNode，避免内存泄漏
            for (auto *ptr : grid_node_buffer_) {
                delete ptr;
            }
        };

        typedef std::shared_ptr<Astar> Ptr;

        void setVisualProcessEn(const bool &en);

        void setFineInfNeighbors(const int & neighbor_step);

    RET_CODE pointToPointPathSearch(const rog_map::Vec3f &start_pt, const rog_map::Vec3f &end_pt,
                    const int &flag,
                    rog_map::vec_Vec3f &out_path,
                    const double &time_out = 0.1);

        /// @ brief: The escape path only for path search from prob map to inf map. from non-occupied point to
        ///          inf map free (or known freee) point . Aim to find a path from current point to (known) free point
        /// @ param:
    RET_CODE escapePathSearch(const rog_map::Vec3f &start_pt, const int flag, rog_map::vec_Vec3f &out_path);

    /**
     * @brief 对原始 A* 栅格路径进行折线抽稀，移除共线中间点。
     * @param astar_path 输入的原始离散路径（起点->终点）。
     * @param waypoint 输出的折线关键点序列（起点->终点）。
     */
    void SimplifyPath(const rog_map::vec_Vec3f &astar_path,
              rog_map::vec_Vec3f &waypoint);

    /**
     * @brief Floyd 路径平滑处理：两次迭代删除可直连的冗余拐点。
     * @param astar_path 输入的原始离散路径（起点->终点）。
     * @param waypoint 输出处理后的关键点序列。
     */
    void FloydHandle(const rog_map::vec_Vec3f &astar_path,
             rog_map::vec_Vec3f &waypoint);

    /**
     * @brief 检查两点之间直线是否无障碍（抽样检测）。
     * @param p1 起点
     * @param p2 终点
     * @return true 若两点间可直连（未穿过占据栅格或越界被视为可忽略），否则 false
     */
    bool CheckLineObstacleFree(const rog_map::Vec3f &p1, const rog_map::Vec3f &p2);

    /**
     * @brief 检查一条离散路径是否无碰，包括点占据与相邻段的直线采样碰撞检测。
     * @param path 连续坐标路径（起点->终点）。
     * @return true 路径完全可达；false 存在占据/不可通行段。
     */
    bool CheckPathFree(const rog_map::vec_Vec3f &path);

    bool CheckPointFree(const rog_map::Vec3f &point);
    };
}