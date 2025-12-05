#include "WaypointExecutive.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <filesystem>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPath.JSON";
  std::shared_ptr<WaypointExecutive> Node = std::make_shared<WaypointExecutive>(MissionPath);
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(Node);
  std::cout << "ROS2 Waypoint running" << std::endl;
  std::jthread spin_ros([executor]()
                        { executor->spin(); });
  int ResultCode = Node->Controller();
  // This will end when it needs to ^.
  rclcpp::shutdown();
  return ResultCode;
}