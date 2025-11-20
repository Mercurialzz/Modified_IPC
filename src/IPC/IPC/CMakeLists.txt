cmake_minimum_required(VERSION 3.0.2)
project(ipc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

# 定义 ROGMap 所需的宏
add_definitions(-DUSE_ROS1)
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  quadrotor_msgs
  eigen_conversions
  visualization_msgs
  sensor_msgs
  pcl_ros
  rog_map
  mavros
  rosfmt
  uav_utils
  dynamic_reconfigure
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/fake_rc.cfg
)

catkin_package(
        INCLUDE_DIRS
        # CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
  SYSTEM
  include 
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include 
)

add_executable(ipc_node
  src/main.cpp
  src/planner.cpp
  src/input.cpp
  src/param.cpp
  include/astar.cpp
  include/mpc.cpp
  include/local_astar.cpp
  src/sfc_core/ciri.cpp
  src/sfc_core/corridor_generator.cpp
  src/utils/polytope.cpp
  src/utils/ellipsoid.cpp
  src/utils/mvie.cpp
  src/utils/quickhull.cpp
  src/utils/geometry_utils.cpp
  src/utils/optimization_utils.cpp
  src/utils/sdlp.cpp
  src/utils/lbfgs.cpp
)
add_dependencies(ipc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ipc_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  -ldw
  tbb
  OsqpEigen::OsqpEigen
)

add_executable(fast_avoid
  test/fast_avoid.cpp
)
add_dependencies(fast_avoid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fast_avoid
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(trans_odom_node
  src/trans_odom.cpp)
add_dependencies(trans_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trans_odom_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

