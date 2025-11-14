#ifndef THRUST_INTERFACE_HPP
#define THRUST_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/pwms.hpp"

using namespace rclcpp;

class Thrust_Interface : public Node {
    public:
    Thrust_Interface(std::vector<int> thrusters, char* pico_path, int min_pwm, int max_pwm);
    
    private:
    void pwm_received_subscription_callback(custom_interfaces::msg::Pwms::UniquePtr pwms_msg);
    rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_received_subscription;

    void send_to_pico(int thruster, int pwm);
    int open_pico_serial(char* path);
    int pico_fd = -1;
    std::vector<int> thrusters;
    int min_pwm;
    int max_pwm;
};

#endif // THRUST_INTERFACE_HPP