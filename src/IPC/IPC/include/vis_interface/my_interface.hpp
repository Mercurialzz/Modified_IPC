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


#ifndef SRC_VISUALIZER_INTERFACE_HPP
#define SRC_VISUALIZER_INTERFACE_HPP

#include <memory>
#include <utils/header/eigen_alias.hpp>
#include <utils/header/color_msg_utils.hpp>
#include <fmt/color.h>
#include <sfc_core/polytope.h>


#define TINYCOLORMAP_WITH_EIGEN
#include "utils/header/tinycolormap.hpp"

enum SUPER_RET_CODE {
    SUPER_SUCCESS_WITH_BACKUP = 3,
    SUPER_SUCCESS_NO_BACKUP = 2,
    SUPER_SUCCESS = 1,
    SUPPER_UNDEFINED = -0,
    SUPER_NO_ODOM = -1,
    SUPER_NO_START_POINT = -2,

};

static std::string SUPER_RET_CODE_STR(const int& ret) {
    switch (ret) {
    case SUPER_SUCCESS_WITH_BACKUP:
        return "Success, with backup trajectory also success";
    case SUPER_SUCCESS_NO_BACKUP:
        return "Success, without need of backup";
    case SUPER_SUCCESS:
        return "Success";
    case SUPPER_UNDEFINED:
        return "Undefined";
    case SUPER_NO_ODOM:
        return "No odom, return at the start of the replan";
    case SUPER_NO_START_POINT:
        return "Cannot find a start point in the local map";
    }
};

namespace vis_interface{
    using namespace geometry_utils;
    using namespace super_utils;

    class MyInterface {
    protected:
        bool visualization_en_{false};
        double resolution_{0.1};
    public:
        using Ptr = std::shared_ptr<MyInterface>;

        virtual ~MyInterface() = default;
        /*===================For logger interface =======================*/
        template<typename... Args>
        static std::string format(const char* fmt, Args&&... args) {
            return fmt::format(fmt, std::forward<Args>(args)...);
        }
        // 核心日志接口
        virtual void debug(const std::string& msg) = 0;
        virtual void info(const std::string& msg) = 0;
        virtual void warn(const std::string& msg) = 0;
        virtual void error(const std::string& msg) = 0;
        virtual void fatal(const std::string& msg) = 0;

        // 格式化工具函数
        template<typename... Args>
        void debug(const char* fmt, Args... args) {
            debug(format(fmt, args...));
        }

        template<typename... Args>
        void info(const char* fmt, Args... args) {
            info(format(fmt, args...));
        }

        template<typename... Args>
        void warn(const char* fmt, Args... args) {
            warn(format(fmt, args...));
        }

        template<typename... Args>
        void error(const char* fmt, Args... args) {
            error(format(fmt, args...));
        }

        template<typename... Args>
        void fatal(const char* fmt, Args... args) {
            fatal(format(fmt, args...));
        }


        /*===================For time interface =======================*/
        virtual void setSimTime(const double &sim_time) = 0;

        virtual double getSimTime() = 0;

        virtual void getSimTime(int32_t &sec, uint32_t &nsec) = 0;
        
        /* [新增] Velocity Visualization */
        virtual void vizVelocity(const Vec3f& position, const double& velocity, const std::string& ns = "velocity_text") = 0;
        /*===================For viz interface =======================*/
        virtual void vizFrontendPath(const vec_Vec3f & path) = 0;

        virtual void vizExpSfc(const PolytopeVec & sfc) = 0;

        virtual void vizCurSfc(const Polytope & sfc) = 0;

        virtual void vizGoalPath(const vec_Vec3f & path) = 0;

        /*For Astar debug ==================================*/
        virtual void vizAstarBoundingBox(const Vec3f & bbox_min, const Vec3f & bbox_max) = 0;

        virtual void vizAstarPoints(const Vec3f & position, const Color & c, const std::string & ns,
                                    const double & size = 0.1, const int & id = 0) = 0;

        /*For CIRI debug==================================*/
        virtual void vizCiriSeedLine(const Vec3f & a, const Vec3f & b, const double & robot_r) = 0;

        virtual void vizCiriEllipsoid(const Ellipsoid & ellipsoid) = 0;

        virtual void vizCiriInfeasiblePoint(const Vec3f p) = 0;

        virtual void vizCiriPolytope(const Polytope & polytope, const std::string & ns) = 0;

        virtual void vizCiriPointCloud(const vec_Vec3f & points) = 0;


        /*Set some parameters==================================*/
        virtual void setResolution(const double &resolution) {
            resolution_ = resolution;
            fmt::print(fg(fmt::color::lime_green)," -- [Viz] Set resolution_ to {}\n", resolution_);
        }

        virtual void setVisualizationEn(const bool &  en) {
            visualization_en_ = en;
            fmt::print(fg(fmt::color::lime_green)," -- [Viz] Set visualization_en_ to {}\n", en);
        }

    };
}


#endif //SRC_VISUALIZER_INTERFACE_HPP
