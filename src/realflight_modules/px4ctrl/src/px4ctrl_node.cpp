#include <ros/ros.h>  // 包含ROS的头文件
#include "PX4CtrlFSM.h"  // 包含自定义的PX4CtrlFSM类的头文件
#include <signal.h>  // 包含处理信号的头文件

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");  // 打印信息到ROS日志
    ros::shutdown();  // 关闭ROS节点
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");  // 初始化ROS节点
    ros::NodeHandle nh("~");  // 创建ROS节点句柄

    signal(SIGINT, mySigintHandler);  // 注册SIGINT信号的处理函数
    ros::Duration(1.0).sleep();  // 等待1秒钟

    Parameter_t param;
    param.config_from_ros_handle(nh);  // 从ROS参数服务器中获取参数配置

    LinearControl controller(param);  // 创建LinearControl对象
    PX4CtrlFSM fsm(param, controller);  // 创建PX4CtrlFSM状态机对象

    // 订阅各种传感器数据和控制命令
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 100, boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("cmd", 100, boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    
    // 根据是否连接遥控器订阅遥控器数据
    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC)
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }
    
    // 订阅电池状态和起飞降落指令
    ros::Subscriber bat_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 100, boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    ros::Subscriber takeoff_land_sub = nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land", 100, boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    // 发布控制命令和调试信息
    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10);  // 调试信息

    // 创建服务客户端用于设置FCU模式、解锁和重启飞控
    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("[PX4CTRL] Remote controller disabled, be careful!");  // 打印警告信息
    }
    else
    {
        ROS_INFO("[PX4CTRL] Waiting for RC");  // 打印信息
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");  // 打印信息
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    // 等待连接到PX4飞控
    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connect to PX4!!!");  // 打印错误信息
    }

    ros::Rate r(param.ctrl_freq_max);  // 设置控制频率
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process();  // 处理状态机逻辑
    }

    return 0;
}
