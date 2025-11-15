#include "mux_quyen.hpp"

using namespace rclcpp;

SoftMux::SoftMux() : Node("Soft_Mux") {
    pwm_cmd_publisher = this->create_publisher<custom_interfaces::msg::Pwms>("pwm_cmd", 10);
    control_mode_service = this->create_service<std_srvs::srv::SetBool>("control_mode", std::bind(&SoftMux::set_mode_srv, this, std::placeholders::_1, std::placeholders::_2));

    auto pwm_ctrl_callback =
        [this](custom_interfaces::msg::Pwms::SharedPtr pwm) -> void {
            if(this->is_matlab_mode) pwm_cmd_publish(pwm);
        };
    pwm_ctrl_subscriber =
        this->create_subscription<custom_interfaces::msg::Pwms>("pwm_ctrl", 10, pwm_ctrl_callback);

    auto pwm_cli_callback =
        [this](custom_interfaces::msg::Pwms::SharedPtr pwm) -> void {
            if(!this->is_matlab_mode) pwm_cmd_publish(pwm);
        };
    pwm_cli_subscriber =
        this->create_subscription<custom_interfaces::msg::Pwms>("pwm_cli", 10, pwm_cli_callback);
}

void SoftMux::set_mode_srv(std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        this->is_matlab_mode = request->data;
        response->message = "succeeded";
        response->success = true;
}

void SoftMux::pwm_cmd_publish(custom_interfaces::msg::Pwms::SharedPtr pwm) {
    this->pwm_cmd_publisher->publish(*pwm);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SoftMux>());
  rclcpp::shutdown();
  return 0;
}   