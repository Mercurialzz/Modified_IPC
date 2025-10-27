#include <ros/ros.h>
#include "planner.h"
#include <signal.h>
#include "param.h"
#include "planner.h"

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ipc_node");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    PlannerClass planner(nh,param);
    
    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &planner.state_data, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &planner.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &planner.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());


    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("imu", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &planner.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &planner.rc_data, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &planner.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("/takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &planner.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());
    
                                                  ros::Duration(0.5).sleep();
    ros::Subscriber goal_sub = 
        nh.subscribe<geometry_msgs::PoseStamped>("goal",
                                                  10,
                                                  boost::bind(&Goal_Data_t::feed, &planner.goal_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());
    ros::Subscriber point_cloud_sub =
        nh.subscribe<sensor_msgs::PointCloud2>("local_pc",
                                               10,
                                               boost::bind(&PlannerClass::LocalPcCallback, &planner, _1),
                                               ros::VoidConstPtr(),
                                               ros::TransportHints().tcpNoDelay());

    // ros topic pub
    planner.astar_pub_ = nh.advertise<visualization_msgs::Marker>("astar_path", 1);
    planner.gird_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map", 1);
    planner.cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 1);
    planner.mpc_path_pub_ = nh.advertise<nav_msgs::Path>("mpc_path", 1);
    planner.sfc_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc", 1);
    planner.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("px4ctrl", 10);
    planner.goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_pub", 1);
    // ros topic service                                     
    planner.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    planner.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    planner.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    dynamic_reconfigure::Server<ipc::fake_rcConfig> server;
    dynamic_reconfigure::Server<ipc::fake_rcConfig>::CallbackType f;


    if (param.takeoff_land.no_RC)
    {
        f = boost::bind(&Dynamic_Data_t::feed, &planner.dy_data ,_1); //绑定回调函数
        server.setCallback(f); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (planner.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !planner.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        planner.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }


    return 0;
}
