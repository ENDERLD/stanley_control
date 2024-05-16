#ifndef MW_API_MW_API_HPP
#define MW_API_MW_API_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>

using EmptyT = std_msgs::msg::Empty;
using EmptyConstPtrT = std_msgs::msg::Empty::ConstSharedPtr;
using UInt8MultiArrayT = std_msgs::msg::UInt8MultiArray;
using UInt8MultiArrayConstPtrT = std_msgs::msg::UInt8MultiArray::ConstSharedPtr;
using Int64MultiArrayT = std_msgs::msg::Int64MultiArray;
using Int64MultiArrayConstPtrT = std_msgs::msg::Int64MultiArray::ConstSharedPtr;
using UInt64MultiArrayT = std_msgs::msg::UInt64MultiArray;
using UInt64MultiArrayConstPtrT = std_msgs::msg::UInt64MultiArray::ConstSharedPtr;
using Float64MultiArrayT = std_msgs::msg::Float64MultiArray;
using Float64MultiArrayConstPtrT = std_msgs::msg::Float64MultiArray::ConstSharedPtr;
using StringT = std_msgs::msg::String;
using StringConstPtrT = std_msgs::msg::String::ConstSharedPtr;
using Int64T = std_msgs::msg::Int64;
using Int64ConstPtrT = std_msgs::msg::Int64::ConstSharedPtr;
using Float64T = std_msgs::msg::Float64;
using Float64ConstPtrT = std_msgs::msg::Float64::ConstSharedPtr;

#define mw_nh_type std::shared_ptr<rclcpp::Node>
#define mw_pub_type(MessageT) std::shared_ptr<rclcpp::Publisher<MessageT>>
#define mw_sub_type(MessageT) std::shared_ptr<rclcpp::Subscription<MessageT>>
#define mw_timer_type std::shared_ptr<rclcpp::TimerBase>

#define mw_init(node) rclcpp::init(0, nullptr)

#define mw_master_check

#define mw_nh(node) std::make_shared<rclcpp::Node>(node)

#define mw_create_timer(seconds, callback) \
    nh->create_wall_timer(std::chrono::milliseconds(int64_t((seconds) * 1000)), (callback))

#define mw_subscribe(MessageT, topic, callback) \
    nh->template create_subscription<MessageT>((topic), 1, (callback))

#define mw_advertise(MessageT, topic) \
    nh->template create_publisher<MessageT>((topic), 1)

#define mw_spin rclcpp::spin(nh)

#define mw_multi_spin \
    rclcpp::executors::MultiThreadedExecutor executor; \
    executor.add_node(nh); \
    executor.spin()

#define mw_shutdown rclcpp::shutdown()

#define mw_ok rclcpp::ok()

#endif //MW_API_MW_API_HPP
