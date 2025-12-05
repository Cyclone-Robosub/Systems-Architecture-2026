#include "mux_sherry.hpp"

using namespace std::chrono_literals;

SoftMux::SoftMux() : rclcpp::Node("SoftMux") {
    pwm_cmd_publisher = this->create_publisher<custom_interfaces::msg::Pwms>("pwm_cmd", 10);
    pwm_ctrl_subscriber = this->create_subscription<custom_interfaces::msg::Pwms>("pwm_ctrl", 10, std::bind(&SoftMux::pwm_ctrl_callback, this, std::placeholders::_1));
    pwm_cli_subscriber =  this->create_subscription<custom_interfaces::msg::Pwms>("pwm_cli", 10, std::bind(&SoftMux::pwm_cli_callback, this, std::placeholders::_1));
    current_control_mode = this->create_publisher<std_msgs::msg::Bool>("current_mode", 10);
    control_mode = this->create_service<std_srvs::srv::SetBool>("control_mode", std::bind(&SoftMux::set_mode_srv, this, std::placeholders::_1, std::placeholders::_2));
    ctrl_heartbeat_subscriber = this->create_subscription<std_msgs::msg::Bool>("ctrl_heartbeat", 10, std::bind(&SoftMux::ctrl_heartbeat_callback, this, std::placeholders::_1));
    cli_heartbeat_subscriber= this->create_subscription<std_msgs::msg::Bool>("cli_heartbeat",10, std::bind(&SoftMux::cli_heartbeat_callback, this, std::placeholders::_1));
    ctrl_heartbeat_timer = this->create_wall_timer(500ms, std::bind(&SoftMux::ctrl_heartbeat_check_callback, this));
    cli_heartbeat_timer= this->create_wall_timer(500ms, std::bind(&SoftMux::cli_heartbeat_check_callback, this));
}


void SoftMux::pwm_ctrl_callback(custom_interfaces::msg::Pwms::UniquePtr pwm) {
    if (no_heartbeat) {
        return;
    }
    if (is_matlab_mode) {
        pwm_cmd_publish(std::move(pwm));
    }
}

void SoftMux::pwm_cli_callback(custom_interfaces::msg::Pwms::UniquePtr pwm) {
    if (no_heartbeat) {
        return;
    }
    if (!is_matlab_mode) {
        pwm_cmd_publish(std::move(pwm));
    }
}

void SoftMux::set_mode_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    bool mode_change = false;
    if (this->is_matlab_mode != request->data) {
        mode_change = true;
    }
    this->is_matlab_mode = request->data;
    if (this->is_matlab_mode == request->data) {
        if (mode_change) {
            std_msgs::msg::Bool message;
            message.data = this->is_matlab_mode;
            this->current_control_mode->publish(message);
            std::cout << message.data << std::endl; // TODO: Remove prints before merging to main
        }
        response->success = true;
    } else {
        response->success = false;
    }
}

void SoftMux::pwm_cmd_publish(custom_interfaces::msg::Pwms::UniquePtr pwm) {
    this->pwm_cmd_publisher->publish(*(std::move(pwm)));
    for (int i = 0; i < 8; i++) { // TODO: Remove prints before merging to main
         std::cout << "PWM " << i << " is sent as " << pwm->pwms[i] << "\n";
    }

}

void SoftMux::ctrl_heartbeat_callback(std_msgs::msg::Bool::UniquePtr heartbeat) {
    if (is_matlab_mode) {
        recent_ctrl_heartbeat = std::chrono::steady_clock::now();
    }
    (void) heartbeat;
}

void SoftMux::ctrl_heartbeat_check_callback() {
    if (is_matlab_mode) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - recent_ctrl_heartbeat > 1s) {
            // stop command
            no_heartbeat = true;
            auto stop_pwms = custom_interfaces::msg::Pwms();
            for (int i = 0; i < 8; i++) {
                stop_pwms.pwms[i] = 1500;
            }
            pwm_cmd_publish(std::make_unique<custom_interfaces::msg::Pwms>(stop_pwms));
        }
    }
}
void SoftMux::cli_heartbeat_callback(std_msgs::msg::Bool::UniquePtr heartbeat){
    if(!is_matlab_mode){
        recent_ctrl_heartbeat = std::chrono::steady_clock::now();
    }
    (void) heartbeat;
}
void SoftMux::cli_heartbeat_check_callback() {
    if (!is_matlab_mode) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - recent_ctrl_heartbeat > 1s) {
            // stop command
            no_heartbeat = true;
            auto stop_pwms = custom_interfaces::msg::Pwms();
            for (int i = 0; i < 8; i++) {
                stop_pwms.pwms[i] = 1500;
            }
            pwm_cmd_publish(std::make_unique<custom_interfaces::msg::Pwms>(stop_pwms));
        }

    }   
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoftMux>());
    rclcpp::shutdown();
    return 0;
}
