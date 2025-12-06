#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/pwms.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"


class SoftMux : public rclcpp::Node {
    public:
        SoftMux();
        void pwm_ctrl_callback(custom_interfaces::msg::Pwms::UniquePtr pwm);
        void pwm_cli_callback(custom_interfaces::msg::Pwms::UniquePtr pwm);
        void set_mode_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    private:
        void ctrl_heartbeat_callback(std_msgs::msg::Bool::UniquePtr heartbeat);
        void ctrl_heartbeat_check_callback();
        void cli_heartbeat_callback(std_msgs::msg::Bool::UniquePtr heartbeat);
        void cli_heartbeat_check_callback();
        void pwm_cmd_publish(custom_interfaces::msg::Pwms::UniquePtr pwm);
        void publish_stop_command();
        bool is_matlab_mode;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_mode;
        rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_ctrl_subscriber;
        rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_cli_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ctrl_heartbeat_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cli_heartbeat_subscriber;
        rclcpp::Publisher<custom_interfaces::msg::Pwms>::SharedPtr pwm_cmd_publisher; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr current_control_mode;
        rclcpp::TimerBase::SharedPtr ctrl_heartbeat_timer;
        rclcpp::TimerBase::SharedPtr cli_heartbeat_timer;
        bool no_ctrl_heartbeat = true;
        bool no_cli_heartbeat = true;
        std::chrono::time_point<std::chrono::steady_clock> recent_ctrl_heartbeat;
        std::chrono::time_point<std::chrono::steady_clock> recent_cli_heartbeat;
};
#endif