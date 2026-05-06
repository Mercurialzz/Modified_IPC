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


#ifndef SRC_ROS1_VISUALIZER_HPP
#define SRC_ROS1_VISUALIZER_HPP

#include "vis_interface/vis_adapter.hpp"


namespace vis_interface {

    class VisInterface : public MyInterface {
    public:

        explicit VisInterface(const ros::NodeHandle &nh)
                : nh_(nh) {

            /*=============================FOR Planner========================================*/
            goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/goal", 100);

            exp_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_traj", 100);
            backup_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/backup_traj", 100);
            committed_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/committed_traj", 100);

            exp_sfcs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_sfc", 100);
            sfc_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/ipc_sfc", 100);

            guide_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/frontend_path", 100);

            yaw_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/yaw_traj", 100);

            /*=============================FOR A* debug ========================================*/
            astar_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/astar_debug", 100);

            /*=============================FOR A* debug ========================================*/
            ciri_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/ciri_debug_mkr", 100);
            ciri_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visualization/ciri_debug_pc", 100);
            /*=============================FOR replan log ========================================*/
            replan_log_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visualization/replan_log_pc", 100);
            replan_log_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/replan_log_mkr", 100);

            /* [新增] For Velocity Text */
            vel_text_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/velocity_text", 10);
        }


        /*=============================FOR ROS logger ========================================*/
        void debug(const std::string& msg) override { ROS_DEBUG("%s", msg.c_str()); }
        void info(const std::string& msg) override { ROS_INFO("%s", msg.c_str()); }
        void warn(const std::string& msg) override { ROS_WARN("%s", msg.c_str()); }
        void error(const std::string& msg) override { ROS_ERROR("%s", msg.c_str()); }
        void fatal(const std::string& msg) override { ROS_FATAL("%s", msg.c_str()); }

        double getSimTime() override {
            return ros::Time::now().toSec();
        }

        void getSimTime(int32_t &sec, uint32_t &nsec) override{
            ros::Time now = ros::Time::now();
            sec = now.sec;
            nsec = now.nsec;
        }

        void setSimTime(const double &sim_time) override {
            ros::Time::setNow(ros::Time(sim_time));
        }

        void vizFrontendPath(const super_utils::vec_Vec3f &path) override {
            if (!visualization_en_) {
                return;
            }
            if(path.empty()){
                return;
            }
            VisAdapter::deleteAllMarkerArray(guide_path_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            VisAdapter::addPathToMarkerArray(mkr_arr, path, Color::Pink(), "guide_path", 0.1, 0.05);
            guide_path_pub_.publish(mkr_arr);
        }

        void vizExpSfc(const PolytopeVec &sfcs) override {
            if (!visualization_en_) {
                return;
            }
            if (exp_sfcs_pub_.getNumSubscribers() <= 0) {
                return;
            }
            if(sfcs.empty()){
                return;
            }
            VisAdapter::deleteAllMarkerArray(exp_sfcs_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            int color_num = sfcs.size();
            int color_id = 0;
            for (auto p: sfcs) {
                double color_ratio = 1.0 - (double) color_id / color_num;
                Vec3f color_mag = tinycolormap::GetColor(color_ratio, tinycolormap::ColormapType::Jet).ConvertToEigen();
                color_id++;
                Color c(color_mag[0], color_mag[1], color_mag[2]);
                VisAdapter::addPolytopeToMarkerArray(mkr_arr, p,
                                                      "exp_sfc", false,
                                                      Color::SteelBlue(), c,
                                                      Color::Orange(), 0.15,
                                                      resolution_ / 2);
            }
            exp_sfcs_pub_.publish(mkr_arr);
        }

        void vizCurSfc(const Polytope &sfc) override {
            if (!visualization_en_) {
                return;
            }
            if (sfc_pub_.getNumSubscribers() <= 0) {
                return;
            }
            VisAdapter::deleteAllMarkerArray(sfc_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            VisAdapter::addPolytopeToMarkerArray(mkr_arr, sfc, "cur_sfc", false, Color::Chartreuse(),
                                                  Color::Green(),
                                                  Color::Green(),
                                                  0.15,
                                                  resolution_ / 2);
            sfc_pub_.publish(mkr_arr);
        }

        void vizGoalPath(const super_utils::vec_Vec3f &path) override {
            if (!visualization_en_) {
                return;
            }

            if (goal_pub_.getNumSubscribers() <= 0) {
                return;
            }

            if(path.empty()){
                return;
            }

            VisAdapter::deleteAllMarkerArray(goal_pub_);

            visualization_msgs::MarkerArray mkr_arr;
            VisAdapter::addPathToMarkerArray(mkr_arr, path, Color::Yellow(), "goal", 0.3, 0.15);
            goal_pub_.publish(mkr_arr);
        }

        /*=============================FOR A* debug ========================================*/
        void vizAstarBoundingBox(const super_utils::Vec3f &bbox_min, const super_utils::Vec3f &bbox_max) override {
            if (!visualization_en_) {
                return;
            }

            if (astar_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            VisAdapter::addBoundingBoxToMarkerArray(mkr_arr, bbox_min, bbox_max, "local_map",
                                                     Color::Chartreuse());
            astar_mkr_pub_.publish(mkr_arr);
        }

        void vizAstarPoints(const super_utils::Vec3f &position, const Color &c,
                            const std::string &ns = "none", const double &size = 0.1,
                            const int &id = 0) override {
            if (!visualization_en_) {
                return;
            }

            if (astar_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            VisAdapter::addPointToMarkerArray(mkr_arr, position, c, ns, size);
            astar_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriSeedLine(const super_utils::Vec3f &a, const super_utils::Vec3f &b, const double &robot_r) override {
            if (!visualization_en_) {
                ROS_INFO("Visualization disabled");
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                ROS_INFO("No ciri seed line subscribers");
                return;
            }

            ROS_INFO("vizCiriSeedLine");
            visualization_msgs::MarkerArray mkr_arr;
            vis_interface::VisAdapter::addLineToMarkerArray(mkr_arr, a, b,
                                                             Color::Pink(), Color::Orange(), "seed_line",
                                                             robot_r * 2,
                                                             robot_r * 2);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriEllipsoid(const geometry_utils::Ellipsoid &ellipsoid) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            vis_interface::VisAdapter::addEllipsoidToMarkerArray(mkr_arr, ellipsoid, "ellipsoid", Color(Color::Orange(), 0.3));
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriInfeasiblePoint(const super_utils::Vec3f p) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            vis_interface::VisAdapter::addPointToMarkerArray(mkr_arr, p, Color::Red(), "infeasible_pt", 0.1);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriPolytope(const geometry_utils::Polytope &polytope, const std::string & ns) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            vis_interface::VisAdapter::addPolytopeToMarkerArray(mkr_arr, polytope, ns, true,
                                                                 Color::Chartreuse(), Color::Green(),
                                                                 Color::Green(),
                                                                 0.15,
                                                                 0.02);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriPointCloud(const vec_Vec3f & points) override {
            if (!visualization_en_) {
                return;
            }

            if (ciri_pc_pub_.getNumSubscribers() <= 0) {
                return;
            }

            sensor_msgs::PointCloud2 pc2;
            vis_interface::VisAdapter::addVecPointsToPointCloud2(points, pc2);
            ciri_pc_pub_.publish(pc2);
        }
        /* [新增] 实现速度可视化 */
        void vizVelocity(const super_utils::Vec3f &position, const double &velocity, const std::string &ns = "velocity_text") override {
            if (!visualization_en_) return;
            if (vel_text_pub_.getNumSubscribers() <= 0) return;

            visualization_msgs::MarkerArray mkr_arr;
            
            // 构造显示文本，保留两位小数
            std::string text = "Vel: " + std::to_string(velocity).substr(0, 4) + " m/s";
            
            // 为了不遮挡无人机模型，将文字显示在无人机上方 0.5m 处
            super_utils::Vec3f text_pos = position;
            text_pos.z() += 0.5;

            // 使用黑色显示
            VisAdapter::addTextToMarkerArray(mkr_arr, text_pos, text, ns, Color::Black(), 1.0);
            
            vel_text_pub_.publish(mkr_arr);
        }
    private:
        ros::NodeHandle nh_;
        // viz markers
        ros::Publisher goal_pub_, sfc_pub_, backup_traj_pub_, committed_traj_pub_,
                receding_traj_pub_, exp_sfcs_pub_, point_pub_, fov_pub_,
                exp_traj_pub_, astar_pub_, receding_sfc_pub_, backup_traj_star_point_, yaw_traj_pub_, guide_path_pub_;

        ros::Publisher astar_mkr_pub_;

        ros::Publisher replan_log_mkr_pub_, replan_log_pc_pub_;

        ros::Publisher ciri_mkr_pub_, ciri_pc_pub_;

        ros::Publisher vel_text_pub_;

    };
}

#endif //SRC_ROS1_VISUALIZER_HPP

