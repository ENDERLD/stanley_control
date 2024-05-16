#ifndef MW_API_MW_API_HPP
#define MW_API_MW_API_HPP

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

using EmptyT = std_msgs::Empty;
using EmptyConstPtrT = std_msgs::EmptyConstPtr;
using UInt8MultiArrayT = std_msgs::UInt8MultiArray;
using UInt8MultiArrayConstPtrT = std_msgs::UInt8MultiArrayConstPtr;
using Int64MultiArrayT = std_msgs::Int64MultiArray;
using Int64MultiArrayConstPtrT = std_msgs::Int64MultiArrayConstPtr;
using UInt64MultiArrayT = std_msgs::UInt64MultiArray;
using UInt64MultiArrayConstPtrT = std_msgs::UInt64MultiArrayConstPtr;
using Float64MultiArrayT = std_msgs::Float64MultiArray;
using Float64MultiArrayConstPtrT = std_msgs::Float64MultiArrayConstPtr;
using StringT = std_msgs::String;
using StringConstPtrT = std_msgs::StringConstPtr;
using Int64T = std_msgs::Int64;
using Int64ConstPtrT = std_msgs::Int64ConstPtr;
using Float64T = std_msgs::Float64;
using Float64ConstPtrT = std_msgs::Float64ConstPtr;

#define mw_nh_type boost::shared_ptr<ros::NodeHandle>
#define mw_pub_type(MessageT) std::shared_ptr<ros::Publisher>
#define mw_sub_type(MessageT) ros::Subscriber
#define mw_timer_type ros::SteadyTimer

#define mw_init(node) \
    int mw_reserved_argc = 0; \
    ros::init(mw_reserved_argc, nullptr, node)

#define mw_master_check \
    while (!ros::master::check()) { \
        std::cout << "Waiting for ROS master to start..." << std::endl; \
        sleep(1); \
    }

#define mw_nh(node) boost::make_shared<ros::NodeHandle>()

#define mw_create_timer(seconds, callback) \
    nh->createSteadyTimer(ros::WallDuration((seconds)), [&](const ros::SteadyTimerEvent &event) { (callback)(); } )

#define mw_subscribe(MessageT, topic, callback) \
    nh->template subscribe<MessageT>((topic), 1, (callback))

#define mw_advertise(MessageT, topic) \
    std::make_shared<ros::Publisher>(nh->template advertise<MessageT>((topic), 1))

#define mw_spin ros::spin()

#define mw_multi_spin \
    ros::AsyncSpinner spinner(0); \
    spinner.start(); \
    ros::waitForShutdown()

#define mw_shutdown ros::shutdown()

#define mw_ok ros::ok()

#endif //MW_API_MW_API_HPP
