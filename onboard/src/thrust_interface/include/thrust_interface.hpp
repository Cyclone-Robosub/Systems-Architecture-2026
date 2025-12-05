#ifndef THRUST_INTERFACE_HPP
#define THRUST_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/pwms.hpp>
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <string>
#include <chrono>

using namespace rclcpp;

class Thrust_Interface : public rclcpp::Node {
public:
    Thrust_Interface(std::vector<int> thrusters, int pico_fd, 
                     int min_pwm, int max_pwm);
    
    static int open_pico_serial(std::string pico_path);
    
private:
    void pwm_received_subscription_callback(custom_interfaces::msg::Pwms::UniquePtr pwms_msg);
    void send_to_pico(int thruster, int pwm);
    
    void mux_heartbeat_received_callback(std_msgs::msg::Bool::UniquePtr heartbeat);
    void heartbeat_check_callback();
    rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_received_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_subscription;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    std::vector<int> thrusters;
    int pico_fd;
    int min_pwm;
    int max_pwm;
    std::chrono::time_point<std::chrono::steady_clock> most_recent_heartbeat;
    bool no_heartbeat;
};

#endif // THRUST_INTERFACE_HPP