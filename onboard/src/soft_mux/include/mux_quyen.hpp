#ifndef MUX_QUYEN_HPP
#define MUX_QUYEN_HPP

#include "rclcpp/rclcpp.hpp"
#include  "custom_interfaces/msg/pwms.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace rclcpp;

class SoftMux : public rclcpp::Node {
    public:
    SoftMux();
    void set_mode_srv(std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    private:
        void pwm_cmd_publish(custom_interfaces::msg::Pwms::SharedPtr pwm);
        bool is_matlab_mode;

        rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_ctrl_subscriber;
        rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_cli_subscriber;
        rclcpp::Publisher<custom_interfaces::msg::Pwms>::SharedPtr pwm_cmd_publisher;
        rclcpp::Publisher<custom_interfaces::msg::Pwms>::SharedPtr current_control_mode_publisher;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_mode_service;    
};
#endif