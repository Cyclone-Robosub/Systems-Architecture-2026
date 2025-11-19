// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "remote_control_interface/msg/gamepad.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](remote_control_interface::msg::Gamepad::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "X : '%f'", msg->x);
        RCLCPP_INFO(this->get_logger(), "Y : '%f'", msg->y);
        RCLCPP_INFO(this->get_logger(), "Rise : '%f'", msg->rise);
        RCLCPP_INFO(this->get_logger(), "Sink : '%f'", msg->sink);
        RCLCPP_INFO(this->get_logger(), "Yaw : '%f'", msg->yaw);
        RCLCPP_INFO(this->get_logger(), "Pitch : '%f'", msg->pitch);

        // Get the values: x, y, rise, sink, yaw, pitch
        // Convert to PWM signals for 8 thrusters

        // Publish the PWM signals to thrusters
      };
    subscription_ =
      this->create_subscription<remote_control_interface::msg::Gamepad>("ps5_controller", 10, topic_callback);
  }

private:
  rclcpp::Subscription<remote_control_interface::msg::Gamepad>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
