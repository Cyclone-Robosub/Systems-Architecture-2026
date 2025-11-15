#ifndef THRUST_INTERFACE_HPP
#define THRUST_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/pwms.hpp>
#include <vector>
#include <string>

class Thrust_Interface : public rclcpp::Node {
public:
    // Constructor for production use (opens serial port)
    Thrust_Interface(std::vector<int> thrusters, char* pico_path, 
                     int min_pwm, int max_pwm);
    
    // Constructor for testing (uses provided file descriptor)
    Thrust_Interface(std::vector<int> thrusters, int pico_fd, 
                     int min_pwm, int max_pwm, bool is_test_mode = true);
    
    ~Thrust_Interface();

private:
    void pwm_received_subscription_callback(custom_interfaces::msg::Pwms::UniquePtr pwms_msg);
    void send_to_pico(int thruster, int pwm);
    int open_pico_serial(char* pico_path);
    
    rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_received_subscription;
    std::vector<int> thrusters;
    int pico_fd;
    int min_pwm;
    int max_pwm;
    bool owns_fd;  // Track whether we should close the fd in destructor
};

#endif // THRUST_INTERFACE_HPP