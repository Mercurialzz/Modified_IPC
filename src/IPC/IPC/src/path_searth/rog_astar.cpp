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

#include <path_search/rog_astar.h>

using namespace color_text;
using namespace super_utils;

namespace path_search {
    using namespace rog_map;


    /**
     * @brief Astar 构造函数，初始化参数与内部网格节点缓存。
     * @param nh ROS NodeHandle，用于读取参数。
     * @param vis_ptr 可视化接口指针，用于过程调试与展示。
     * @param rm ROG 地图句柄（概率 / 信息地图统一接口）。
     * @details 主要步骤：
     *  1. 调用 init_param 读取搜索相关配置。
     *  2. 依据地图体素尺寸分配 GridNode 缓冲区并初始化 rounds。
     *  3. 预生成一个按距原点距离排序的整数 3D 偏移列表 sorted_pts，用于邻域或启发式辅助。
     */
    Astar::Astar(const ros::NodeHandle &nh,
                 const vis_interface::VisInterface::Ptr &vis_ptr,
                 rog_map::ROGMapROS::Ptr rm) : vis_ptr_(vis_ptr), map_ptr_(rm) {
        init_param(nh);
        cout << rog_map::GREEN << " -- [RM] Init Astar-map." << rog_map::RESET << endl;
        int map_buffer_size = cfg_.map_voxel_num(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2);
        grid_node_buffer_.resize(map_buffer_size);
        for (auto &i: grid_node_buffer_) {
            i = new GridNode;
            i->rounds = 0;
        }
        cout << rog_map::BLUE << "\tmap index size: " << cfg_.map_size_i.transpose() << rog_map::RESET << endl;
        cout << rog_map::BLUE << "\tmap vox_num: " << cfg_.map_voxel_num.transpose() << rog_map::RESET << endl;
        int test_num = 100;
        for (int i = -test_num; i <= test_num; i++) {
            for (int j = -test_num; j <= test_num; j++) {
                for (int k = -test_num; k <= test_num; k++) {
                    rog_map::Vec3i delta(i, j, k);
                    sorted_pts.push_back(delta);
                }
            }
        }
        sort(sorted_pts.begin(), sorted_pts.end(),
             [](const rog_map::Vec3i &pt1, const rog_map::Vec3i &pt2) {
                 double dist1 = pt1.x() * pt1.x() + pt1.y() * pt1.y() + pt1.z() * pt1.z();
                 double dist2 = pt2.x() * pt2.x() + pt2.y() * pt2.y() + pt2.z() * pt2.z();
                 return dist1 < dist2;
             });
    }

    /**
     * @brief 设置一次 A* 搜索的上下文（起终点、地图类型等）。
     * @param start_pt 起点世界/地图坐标。
     * @param goal_pt 终点世界/地图坐标（若与起点相同，可用于 escape 搜索中心在起点）。
     * @param flag 按位标志，控制使用信息地图 / 概率地图、未知标记策略、邻域扩展策略等。
     * @return RET_CODE 成功 SUCCESS；配置非法 INIT_ERROR。
     * @details 主要逻辑：
     *  1. 解析 flag，设置使用的地图与未知格处理策略。
     *  2. 决定局部地图中心（默认取起点与终点中点；若两者相同则中心为起点）。
     *  3. 根据分辨率与 map_size_i 计算局部包围盒 min/max。
     *  4. 可选可视化局部搜索边界。
     */
    RET_CODE Astar::setup(const Vec3f &start_pt, const Vec3f &goal_pt, const int &flag) {
        md_.start_pt = start_pt;
        md_.goal_pt = goal_pt;
    md_.mission_rcv_WT = vis_ptr_->getSimTime();
        md_.use_inf_map = flag & ON_INF_MAP;
        md_.use_prob_map = flag & ON_PROB_MAP;
        md_.unknown_as_occ = flag & UNKNOWN_AS_OCCUPIED;
        md_.unknown_as_free = flag & UNKNOWN_AS_FREE;
        md_.use_inf_neighbor = flag & USE_INF_NEIGHBOR;
        if (flag & DONT_USE_INF_NEIGHBOR) {
            md_.use_inf_neighbor = false;
        }
        if ((md_.use_inf_map && md_.use_prob_map) ||
            (!md_.use_inf_map && !md_.use_prob_map)) {
            cout << YELLOW << " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << ": cannot use both inf map and prob map." << RESET << endl;
            return INIT_ERROR;
        }
        if (md_.unknown_as_occ && md_.unknown_as_free) {
            cout << YELLOW << " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << ": cannot use both unknown_as_occupied and unknown_as_free." << RESET << endl;
            return INIT_ERROR;
        }
        if (md_.use_prob_map) {
            md_.resolution = map_ptr_->getResolution();
        } else {
            md_.resolution = map_ptr_->getInfResolution();
        }
 
        md_.local_map_center_d = start_pt;

        //md_.local_map_center_d = (start_pt + goal_pt) / 2;


        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
        md_.local_map_min_d = md_.local_map_center_d - md_.resolution * cfg_.map_size_i.cast<double>();
        md_.local_map_max_d = md_.local_map_center_d + md_.resolution * cfg_.map_size_i.cast<double>();;
        if (cfg_.visual_process||cfg_.debug_visualization_en) {
            vis_ptr_->vizAstarBoundingBox(md_.local_map_min_d, md_.local_map_max_d);
        }

        return SUCCESS;
    }

    /**
     * @brief 计算启发式 (Heuristic) 距离。
     * @param node1 当前节点指针。
     * @param node2 目标节点指针。
     * @param type 启发式类型：DIAG / MANH / EUCL。
     * @return 加入 tie_breaker_ 后的启发式估计值。
     * @note DIAG 使用 3D 对角线 + 2D 对角线混合；MANH 为曼哈顿；EUCL 为欧氏距离。
     */
    double Astar::getHeu(GridNodePtr node1, GridNodePtr node2, int type) const {
        switch (type) {
            case DIAG: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                double h = 0.0;
                int diag = std::min(std::min(dx, dy), dz);
                dx -= diag;
                dy -= diag;
                dz -= diag;

                if (dx == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * std::abs(dy - dz);
                }
                if (dy == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * std::abs(dx - dz);
                }
                if (dz == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * std::abs(dx - dy);
                }
                return tie_breaker_ * h;
            }
            case MANH: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                return tie_breaker_ * (dx + dy + dz);
            }
            case EUCL: {
                return tie_breaker_ * (node2->id_g - node1->id_g).norm();
            }
            default: {
                fmt::print(fg(fmt::color::indian_red), " -- [A*] Wrong hue type.\n");
                return 0;
            }
        }
    }

    /**
     * @brief 将全局格子索引转换为局部缓存线性哈希。
     * @param id_in 全局索引（映射到当前局部地图）。
     * @return 线性数组下标，用于访问 grid_node_buffer_。
     * @warning 假设 id_in 已在局部地图范围内，调用方需保证合法性。
     */
    int Astar::getLocalIndexHash(const Vec3i &id_in) const {
        rog_map::Vec3i id = id_in - md_.local_map_center_id_g + cfg_.map_size_i;
        return id(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2) +
               id(1) * cfg_.map_voxel_num(2) +
               id(2);
    }

    /**
     * @brief 将连续坐标转换为地图全局栅格索引。
     * @param pos 连续空间坐标。
     * @param id_g 输出的整型栅格索引。
     * @details 根据当前使用的是信息地图还是概率地图调用不同的转换函数。
     */
    void Astar::posToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const {
        if (md_.use_inf_map) {
            map_ptr_->infMapPosToGlobalIndex(pos, id_g);
        } else if (md_.use_prob_map) {
            map_ptr_->probMapPosToGlobalIndex(pos, id_g);
        } else {
            throw std::runtime_error(" -- [A*] Map type not defined.");
        }
    }

    /**
     * @brief 将地图全局栅格索引转换回连续坐标。
     * @param id_g 输入格子索引。
     * @param pos 输出连续坐标。
     */
    void Astar::globalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const {
        if (md_.use_inf_map) {
            map_ptr_->infMapGlobalIndexToPos(id_g, pos);
        } else if (md_.use_prob_map) {
            map_ptr_->probMapGlobalIndexToPos(id_g, pos);
        } else {
            throw std::runtime_error(" -- [A*] Map type not defined.");
        }
    }

    /**
     * @brief 判断一个连续坐标是否落在当前局部地图包围盒内。
     * @param pos 连续坐标。
     * @return true 在局部地图范围内；false 不在。
     */
    bool Astar::insideLocalMap(const rog_map::Vec3f &pos) const {
        rog_map::Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        return insideLocalMap(id_g);
    }

    /**
     * @brief 判断一个栅格索引是否在局部地图范围。
     * @param id_g 全局索引。
     * @return true 在范围内；false 超出。
     */
    bool Astar::insideLocalMap(const rog_map::Vec3i &id_g) const {
        rog_map::Vec3i delta = id_g - md_.local_map_center_id_g;
        if (fabs(delta.x()) > cfg_.map_size_i.x() ||
            fabs(delta.y()) > cfg_.map_size_i.y() ||
            fabs(delta.z()) > cfg_.map_size_i.z()) {
            return false;
        }
        return true;
    }

    /**
     * @brief 设置是否启用过程可视化（实时节点扩展显示）。
     * @param en 启用标志。
     */
    void Astar::setVisualProcessEn(const bool &en) {
        cfg_.visual_process = en;
    }

    /**
     * @brief 从当前终止节点回溯父指针，生成节点路径（逆序 push）。
     * @param current 终点或提前终止点。
     * @param path 输出的节点序列（从终点到起点）。
     */
    void Astar::retrievePath(GridNodePtr current, vector<GridNodePtr> &path) {
        path.push_back(current);
        while (current->father_ptr != NULL) {
            current = current->father_ptr;
            path.push_back(current);
        }
    }

    /**
     * @brief 将节点路径转换为坐标路径，并翻转为起点到终点顺序。
     * @param node_path 节点链表（通常终点到起点顺序）。
     * @param point_path 输出的连续坐标序列（起点到终点）。
     */
    void Astar::ConvertNodePathToPointPath(const vector<GridNodePtr> &node_path, rog_map::vec_Vec3f &point_path) {
        point_path.clear();
        for (auto ptr: node_path) {
            rog_map::Vec3f pos;
            globalIndexToPos(ptr->id_g, pos);
            point_path.push_back(pos);
        }
        reverse(point_path.begin(), point_path.end());
    }

    /**
     * @brief 根据给定步长构建一个球形（半径 = neighbor_step）离散邻域列表。
     * @param neighbor_step 最大偏移步长（栅格单位）。
     * @note 过滤掉原点与超出圆球半径的点，结果用于信息地图细化邻域查询。
     */
    void Astar::setFineInfNeighbors(const int &neighbor_step) {
        neighbor_list.clear();
        for (int i = -neighbor_step; i <= neighbor_step; i++) {
            for (int j = -neighbor_step; j <= neighbor_step; j++) {
                for (int k = -neighbor_step; k <= neighbor_step; k++) {
                    if (i == 0 && j == 0 && k == 0) {
                        continue;
                    }
                    if (i * i + j * j + k * k > neighbor_step * neighbor_step) {
                        continue;
                    }
                    neighbor_list.emplace_back(i, j, k);
                }
            }
        }
    }


    /**
     * @brief 执行点到点 A* 路径搜索（始终搜索到终点，不进行距离地平线截断）。
     * @param start_pt 起点。
     * @param end_pt 终点。
     * @param flag 搜索配置标志位（地图类型 / 未知处理等）。
     * @param out_path 输出连续坐标路径。
     * @param time_out 超时时间（秒）。
     * @return REACH_GOAL 达到终点；REACH_HORIZON 达到地平线；INIT_ERROR 初始化失败；NO_PATH 未找到；TIME_OUT 超时。
     * @details 主要步骤：
     *  1. setup 环境与局部地图。
     *  2. 若起终点在局部地图外，尝试投影到边界附近安全点。
     *  3. 采用优先队列维护 open_set 与 frontier_queue（用于未知区域启发）。
     *  4. 按允许的对角策略扩展 26 邻域，过滤占据 / 未知（基于策略）。
     *  5. 达到终点或地平线条件时回溯路径并转换坐标。
     */
    RET_CODE Astar::pointToPointPathSearch(const rog_map::Vec3f &start_pt, const rog_map::Vec3f &end_pt,
                                           const int &flag,
                                           rog_map::vec_Vec3f &out_path, const double &time_out) {
        RET_CODE setup_ret = setup(start_pt, end_pt, flag);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }
        out_path.clear();
        double time_1 = vis_ptr_->getSimTime();
        ++rounds_;
        /// 2) Switch both start and end point to local map

        rog_map::Vec3f hit_pt;
        rog_map::Vec3f local_start_pt, local_end_pt;
        bool start_pt_out_local_map = false;

        local_start_pt = start_pt;
        local_end_pt = end_pt;

        if (!insideLocalMap(start_pt)) {
            vis_ptr_->warn(" -- [A*] Start point [{}] is out of local map, find a waypoint to the map edge.",
                           start_pt.transpose());
            if (rog_map::lineIntersectBox(start_pt, md_.local_map_center_d, md_.local_map_min_d,
                                          md_.local_map_max_d, hit_pt)) {
                rog_map::Vec3f dir = (hit_pt - start_pt).normalized();
                double dis = (hit_pt - start_pt).norm();
                local_start_pt = start_pt + dir * (dis + md_.resolution * 2);
                start_pt_out_local_map = true;
                if (!map_ptr_->getNearestInfCellNot(OCCUPIED, local_start_pt,
                                                    local_start_pt, 3.0)) {
                    if (cfg_.visual_process || cfg_.debug_visualization_en) {
                        vis_ptr_->vizAstarPoints(local_start_pt, Color::Orange(),
                                                 "local_start_pt",
                                                 0.3, 1);
                    }
                    cout << rog_map::RED <<
                         " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                         << " : start point deeply occupied, cannot find feasible path.\n" << rog_map::RESET << endl;
                    return INIT_ERROR;
                }
            }
        }

        if (!insideLocalMap(end_pt)) {
            rog_map::Vec3f seed_pt = start_pt_out_local_map ? md_.local_map_center_d : start_pt;
            if (rog_map::lineIntersectBox(end_pt, seed_pt, md_.local_map_min_d, md_.local_map_max_d,
                                          hit_pt)) {
                rog_map::Vec3f dir = (hit_pt - end_pt).normalized();
                double dis = (hit_pt - end_pt).norm();
                local_end_pt = end_pt + dir * (dis + 2.5);

                if (!map_ptr_->getNearestInfCellNot(OCCUPIED, local_end_pt, local_end_pt, 2.0)) {
                    vis_ptr_->error(
                            " -- [A*] Error with: {}, Goal point [{}] deeply occupied, cannot find feasible path.",
                            RET_CODE_STR[INIT_ERROR],
                            local_end_pt.transpose());
                    if (cfg_.visual_process || cfg_.debug_visualization_en) {
                        vis_ptr_->vizAstarPoints(local_end_pt, Color::Red(), "local_end_pt",
                                                 0.5,
                                                 1);
                    }
                    return INIT_ERROR;
                }
            }
        }

        if (cfg_.visual_process) {
            vis_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.3,
                                     1);
            vis_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3,
                                     1);
        }
        rog_map::Vec3i start_idx, end_idx;
        posToGlobalIndex(local_start_pt, start_idx);
        posToGlobalIndex(local_end_pt, end_idx);
        if (cfg_.visual_process) {
            vis_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
            vis_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
        }
        if (!insideLocalMap(start_idx) || !insideLocalMap(end_idx)) {
            cout << rog_map::RED << " -- [RM] Start or end point is out of local map, which should not happen." <<
                 rog_map::RESET
                 << endl;
            vis_ptr_->error(" -- [RM] Start [{}] or end point [{}] is out of local map, which should not happen.",
                            local_start_pt.transpose(),
                            local_end_pt.transpose()
                            );
            if(cfg_.visual_process || cfg_.debug_visualization_en) {
                vis_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
                vis_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
            }
            return INIT_ERROR;
        }


        GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
        GridNodePtr endPtr = grid_node_buffer_[getLocalIndexHash(end_idx)];
        endPtr->id_g = end_idx;

        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, FrontierComparator> frontier_queue;


        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = getHeu(startPtr, endPtr, cfg_.heu_type);
        startPtr->state = GridNode::OPENSET; //put start node in open set
        startPtr->father_ptr = NULL;
        open_set.push(startPtr); //put start in open set
        int num_iter = 0;
        vector<GridNodePtr> node_path;

        if (cfg_.visual_process) {
            vis_ptr_->vizAstarPoints(
                    start_pt,
                    Color::Green(),
                    "start_pt",
                    0.3, 1);
            vis_ptr_->vizAstarPoints(
                    end_pt,
                    Color::Blue(),
                    "goal_pt",
                    0.3, 1);
        }

        while (!open_set.empty()) {
            num_iter++;
            current = open_set.top();
            open_set.pop();
            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                vis_ptr_->vizAstarPoints(
                        local_pt,
                        Color(Color::Pink(), 0.5),
                        "astar_process",
                        0.1);
                usleep(1000);
            }
            if (current->id_g(0) == endPtr->id_g(0) &&
                current->id_g(1) == endPtr->id_g(1) &&
                current->id_g(2) == endPtr->id_g(2)) {
                retrievePath(current, node_path);
                if (start_pt_out_local_map) {
                    rog_map::Vec3i start_idx_g;
                    posToGlobalIndex(start_pt, start_idx_g);
                    GridNodePtr temp_ptr(new GridNode);
                    temp_ptr->id_g = start_idx_g;
                    node_path.push_back(temp_ptr);
                }
                ConvertNodePathToPointPath(node_path, out_path);
                return REACH_GOAL;
            }

            // 已移除 searching_horizon 提前终止逻辑，始终寻至终点或失败


            current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        if (!cfg_.allow_diag &&
                            (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1)) {
                            continue;
                        }

                        rog_map::Vec3i neighborIdx;
                        rog_map::Vec3f neighborPos;
                        neighborIdx(0) = (current->id_g)(0) + dx;
                        neighborIdx(1) = (current->id_g)(1) + dy;
                        neighborIdx(2) = (current->id_g)(2) + dz;
                        globalIndexToPos(neighborIdx, neighborPos);

                        if (!insideLocalMap(neighborIdx)) {
                            continue;
                        }

                        rog_map::GridType neighbor_type;

                        if (md_.use_inf_map) {
                            neighbor_type = map_ptr_->getInfGridType(neighborPos);
                        } else {
                            if (!md_.use_inf_neighbor) {
                                neighbor_type = map_ptr_->getGridType(neighborPos);
                            } else {
                                // use prob map, but query all neighbors of the current node
                                // if there is one neighbor is occupied, then the neighbor is occupied.
                                neighbor_type = neighborHaveOne(OCCUPIED, neighborIdx) ? OCCUPIED : UNDEFINED;
                                // if there is one known free neighbor, then the neighbor is known free.
                                if (md_.unknown_as_occ && neighbor_type != OCCUPIED) {
                                    neighbor_type = neighborHaveOne(KNOWN_FREE, neighborIdx) ? KNOWN_FREE : UNKNOWN;
                                }
                            }
                        }

                        if (neighbor_type == OCCUPIED || neighbor_type == OUT_OF_MAP) {
                            continue;
                        }

                        if (md_.unknown_as_occ && neighbor_type == UNKNOWN) {
                            continue;
                        }

                        neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                        if (neighborPtr == nullptr) {
                            cout << rog_map::RED << " -- [RM] neighborPtr is null, which should not happen." <<
                                 rog_map::RESET
                                 << endl;
                            continue;
                        }
                        neighborPtr->id_g = neighborIdx;

                        bool flag_explored = neighborPtr->rounds == rounds_;

                        if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                            continue; //in closed set.
                        }

                        if (md_.unknown_as_occ && neighbor_type == UNKNOWN && neighborPtr) {
                            // the frontier is recorded but not expand.
                            neighborPtr->father_ptr = current;
                            rog_map::Vec3f pos;
                            globalIndexToPos(neighborIdx, pos);
                            neighborPtr->distance_to_goal = getHeu(neighborPtr, endPtr, cfg_.heu_type);
                            frontier_queue.push(neighborPtr);
                            continue;
                        }

                        neighborPtr->rounds = rounds_;
                        double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                        distance_score = current->distance_score + distance_score;
                        rog_map::Vec3f pos;
                        globalIndexToPos(neighborIdx, pos);
                        double heu_score = getHeu(neighborPtr, endPtr, cfg_.heu_type);

                        if (!flag_explored) {
                            //discover a new node
                            neighborPtr->state = GridNode::OPENSET;
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->distance_to_goal = heu_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            open_set.push(neighborPtr); //put neighbor in open set and record it.
                        } else if (distance_score < neighborPtr->distance_score) {
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->distance_to_goal = heu_score;
                            neighborPtr->total_score = distance_score + heu_score;
                        }
                    }
            double time_2 = vis_ptr_->getSimTime();
            if (!cfg_.visual_process && (time_2 - time_1) > time_out) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching !!! {} seconds time limit exceeded.\n", time_out);
                return TIME_OUT;
            }
        }
        double time_2 = vis_ptr_->getSimTime();
        if ((time_2 - time_1) > time_out) {
            fmt::print(fg(fmt::color::indian_red), "Time consume in A star path finding is {} s, iter={}.\n",
                       (time_2 - time_1),
                       num_iter);
            return NO_PATH;
        }

        if (md_.unknown_as_occ && !frontier_queue.empty()) {
            GridNodePtr local_goal;
            while (!frontier_queue.empty()) {
                local_goal = frontier_queue.top();
                frontier_queue.pop();
                rog_map::Vec3f pos;
                globalIndexToPos(local_goal->id_g, pos);
                if ((pos - start_pt).norm() < 1.0) {
                    continue;
                }
                break;
            }
            if (frontier_queue.empty()) {
                cout << rog_map::RED << " -- [A*] Frontier queue is empty, return." << rog_map::RESET << endl;
                return NO_PATH;
            }
            retrievePath(local_goal, node_path);
            if (start_pt_out_local_map) {
                node_path.push_back(startPtr);
            }
            ConvertNodePathToPointPath(node_path, out_path);
            cout << rog_map::BLUE << "Frontier queue: " << frontier_queue.size() << endl;
            return REACH_HORIZON;
        }
        vis_ptr_->error(" -- [A*] Point to point path cannot find path with iter num: {}, return.", num_iter);
        return NO_PATH;
    }

    /**
     * @brief 从高风险 / 占据附近位置向外快速扩展，寻找最近的安全点。
     * @param start_pt 起点（可能在危险区）。
     * @param flag 标志位配置。
     * @param out_path 输出逃逸路径。
     * @return REACH_HORIZON 找到安全点；NO_PATH 未找到；INIT_ERROR 初始化失败；TIME_OUT 超时。
     * @note 与 pointToPointPathSearch 不同：无需终点；启发式为 0；强调快速扩展与安全条件判定。
     */
    RET_CODE Astar::escapePathSearch(const rog_map::Vec3f &start_pt, const int flag, rog_map::vec_Vec3f &out_path) {
        // 逃逸搜索使用起点作为中心，goal 传入同起点
        RET_CODE setup_ret = setup(start_pt, start_pt, flag);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }

        double time_1 = vis_ptr_->getSimTime();
        ++rounds_;

        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
        /// 2) Check start point

        if (!insideLocalMap(start_pt) ||
            !map_ptr_->insideLocalMap(start_pt)) {
            fmt::print(fg(fmt::color::indian_red), " -- [A*] {}: escape start point is not inside local map.\n",
                       RET_CODE_STR[INIT_ERROR].c_str());
            return INIT_ERROR;
        }
        rog_map::Vec3f local_start_pt = start_pt;
//        rog_map::GridType start_type = map_ptr_->getGridType(local_start_pt);
        if (!map_ptr_->getNearestCellNot(OCCUPIED, start_pt, local_start_pt,3.0)) {
            cout << rog_map::RED <<
                 " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << " : escape start point deeply occupied, cannot find feasible path.\n" << rog_map::RESET << endl;
            return INIT_ERROR;
        }

        rog_map::Vec3i start_idx;
        posToGlobalIndex(local_start_pt, start_idx);

        GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = 0;
        startPtr->state = GridNode::OPENSET; //put start node in open set
        startPtr->father_ptr = NULL;
        open_set.push(startPtr); //put start in open set
        int num_iter = 0;

        vector<GridNodePtr> node_path;

        if (cfg_.visual_process) {
            vis_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.05,
                                     1);
        }
        while (!open_set.empty()) {
            num_iter++;
            current = open_set.top();
            open_set.pop();
            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                vis_ptr_->vizAstarPoints(local_pt,
                                         Color::Pink(),
                                         "astar_process",
                                         0.05);
                usleep(1000);
            }
            rog_map::Vec3f cur_pos;
            globalIndexToPos(current->id_g, cur_pos);
            rog_map::GridType cur_inf_type = map_ptr_->getInfGridType(cur_pos);
            if (md_.unknown_as_occ && cur_inf_type != OCCUPIED && cur_inf_type != UNKNOWN) {
                retrievePath(current, node_path);
                ConvertNodePathToPointPath(node_path, out_path);
//                double time_2 = vis_ptr_->getSimTime();
                //                    printf("\033[34m Escape: A star iter:%d, time:%.3f ms\033[0m\n", num_iter, (time_2 - time_1).toSec() * 1000);
                return REACH_HORIZON;
            }

            if (md_.unknown_as_free && cur_inf_type != OCCUPIED) {
                retrievePath(current, node_path);
                ConvertNodePathToPointPath(node_path, out_path);
//                double time_2 = vis_ptr_->getSimTime();
                //                    printf("\033[34m Escape: A star iter:%d, time:%.3f ms\033[0m\n", num_iter, (time_2 - time_1).toSec() * 1000);
                return REACH_HORIZON;
            }

            current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        rog_map::Vec3i neighborIdx;
                        rog_map::Vec3f neighborPos;
                        neighborIdx(0) = (current->id_g)(0) + dx;
                        neighborIdx(1) = (current->id_g)(1) + dy;
                        neighborIdx(2) = (current->id_g)(2) + dz;
                        globalIndexToPos(neighborIdx, neighborPos);
                        if (!map_ptr_->insideLocalMap(neighborPos) ||
                            !insideLocalMap(neighborIdx)) {
                            continue;
                        }

                        rog_map::GridType neighbor_type;
                        if (md_.use_inf_map) {
                            neighbor_type = map_ptr_->getInfGridType(neighborPos);
                        } else {
                            neighbor_type = map_ptr_->getGridType(neighborPos);
                        }

                        if (neighbor_type == OCCUPIED || neighbor_type == OUT_OF_MAP) {
                            continue;
                        }

                        if (md_.unknown_as_occ && neighbor_type == UNKNOWN) {
                            continue;
                        }

                        neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                        if (neighborPtr == nullptr) {
                            cout << rog_map::RED << " -- [RM] neighborPtr is null, which should not happen" <<
                                 rog_map::RESET << endl;
                            continue;
                        }
                        neighborPtr->id_g = neighborIdx;

                        bool flag_explored = neighborPtr->rounds == rounds_;

                        if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                            continue; //in closed set.
                        }

                        neighborPtr->rounds = rounds_;
                        double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                        distance_score = current->distance_score + distance_score;
                        rog_map::Vec3f pos;
                        globalIndexToPos(neighborIdx, pos);
                        double heu_score = 0;
                        if (!flag_explored) {
                            //discover a new node
                            neighborPtr->state = GridNode::OPENSET;
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            open_set.push(neighborPtr); //put neighbor in open set and record it.
                        } else if (distance_score < neighborPtr->distance_score) {
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->total_score = distance_score + heu_score;
                        }
                    }
            double time_2 = vis_ptr_->getSimTime();
            if (!cfg_.visual_process && (time_2 - time_1) > 0.2) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching !!! 0.2 seconds time limit exceeded.\n");
                return TIME_OUT;
            }
        }
        double time_2 = vis_ptr_->getSimTime();
        if ((time_2 - time_1) > 0.1) {
            fmt::print(fg(fmt::color::indian_red), "Time consume in A star path finding is {} s, iter={}.\n",
                       (time_2 - time_1),
                       num_iter);
        }
        cout << rog_map::RED << " -- [A*] Escape path searcher, cannot find path, return." << rog_map::RESET << endl;
        return NO_PATH;
    }

    /**
     * @brief 判断源栅格邻域内是否存在至少一个指定类型栅格。
     * @param type 目标类型（OCCUPIED / KNOWN_FREE 等）。
     * @param src_id 源格索引。
     * @return true 存在一个匹配类型；false 不存在。
     * @details 使用预生成的 neighbor_list；只检查局部地图内格子。
     */
    bool Astar::neighborHaveOne(const rog_map::GridType& type, const rog_map::Vec3i& src_id) {
        for (const auto& nei : neighbor_list) {
            rog_map::Vec3i nei_id = src_id + nei;
            if (!insideLocalMap(nei_id)) {
                continue;
            }
            rog_map::Vec3f nei_pos;
            globalIndexToPos(nei_id, nei_pos);
            rog_map::GridType nei_type;
            if (md_.use_inf_map) {
                nei_type = map_ptr_->getInfGridType(nei_pos);
            }
            else {
                nei_type = map_ptr_->getGridType(nei_pos);
            }
            if (nei_type == type) {
                return true;
            }
        }
        return false;
    }
    void Astar::SimplifyPath(const rog_map::vec_Vec3f &astar_path,
                              rog_map::vec_Vec3f &waypoint) {
        waypoint.clear();
        if (astar_path.size() <= 1) {
            if (!astar_path.empty()) waypoint.push_back(astar_path.front());
            return;
        }
        waypoint.push_back(astar_path[0]);
        if (astar_path.size() <= 2) {
            waypoint.push_back(astar_path[1]);
            return;
        }

        rog_map::Vec3f vec_last = astar_path[1] - astar_path[0];
        for (size_t i = 2; i < astar_path.size(); i++) {
            rog_map::Vec3f vec = astar_path[i] - astar_path[i - 1];
            // 保持原逻辑：完全共线才跳过中间点
            if (vec.dot(vec_last) == vec.norm() * vec_last.norm()) {
                continue;
            }
            waypoint.push_back(astar_path[i - 1]);
            vec_last = vec;
        }
        waypoint.push_back(astar_path.back());
    }

    void Astar::FloydHandle(const rog_map::vec_Vec3f &astar_path,
                             rog_map::vec_Vec3f &waypoint) {
        waypoint.clear();
        SimplifyPath(astar_path, waypoint);

        // Floyd 两次迭代：若后点与前点直线可达，移除中间拐点
        for (int time = 0; time < 2; time++) {
            for (int i = static_cast<int>(waypoint.size()) - 1; i > 0; i--) {
                for (int j = 0; j < i - 1; j++) {
                    if (CheckLineObstacleFree(waypoint[i], waypoint[j])) {
                        for (int k = i - 1; k > j; k--) {
                            waypoint.erase(waypoint.begin() + k);
                        }
                        i = j; // 重新定位 i
                        break;
                    }
                }
            }
        }
    }

    bool Astar::CheckLineObstacleFree(const rog_map::Vec3f &p1, const rog_map::Vec3f &p2) {
        rog_map::Vec3f vec = p2 - p1;
        int sample_num = vec.norm() / md_.resolution; // 依据栅格分辨率做等距采样
        if (sample_num <= 0) return true;
        for (int i = 1; i <= sample_num; i++) {
            rog_map::Vec3f pos = p1 + vec * (double(i) / (sample_num + 1));
            if (!insideLocalMap(pos)) continue; // 局部地图外忽略
            rog_map::GridType pos_type;
            if (md_.use_inf_map) {
                pos_type = map_ptr_->getInfGridType(pos);
            } else {
                pos_type = map_ptr_->getGridType(pos);
            }
            if (pos_type == OCCUPIED) return false;
        }
        return true;
    }
    bool Astar::CheckPathFree(const rog_map::vec_Vec3f& path) {
        if (path.empty()) return true;

        auto is_blocked_point = [&](const rog_map::Vec3f &p) -> bool {
            if (!insideLocalMap(p)) return false; // 局部地图外不判定为阻塞
            rog_map::GridType gt = md_.use_inf_map ? map_ptr_->getInfGridType(p) : map_ptr_->getGridType(p);
            if (gt == OCCUPIED || gt == OUT_OF_MAP) return true;
            if (md_.unknown_as_occ && gt == UNKNOWN) return true;
            return false;
        };

        // 1) 点占据快速检查
        for (const auto &pt : path) {
            if (is_blocked_point(pt)) return false;
        }

        // 2) 相邻段直线采样检查
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const rog_map::Vec3f &p1 = path[i];
            const rog_map::Vec3f &p2 = path[i + 1];
            rog_map::Vec3f vec = p2 - p1;
            double len = vec.norm();
            if (len < 1e-6) continue;
            int sample_num = std::max(1, int(len / std::max(1e-3, md_.resolution)));
            for (int s = 1; s <= sample_num; ++s) {
                rog_map::Vec3f pos = p1 + vec * (double(s) / (sample_num + 1));
                if (is_blocked_point(pos)) return false;
            }
        }
        return true;
    }
}
