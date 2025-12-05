cmake_minimum_required(VERSION 3.8)
project(waypoint_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interface REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Create main executable
add_executable(${PROJECT_NAME}
  src/mainWaypoint.cpp
  src/WaypointExecutive.cpp
  src/Task.cpp
)

# Link libraries to main executable (plain signature)
target_link_libraries(${PROJECT_NAME}
  tomlplusplus
)

# Add ament dependencies to main executable
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Create test executable
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_waypoint.cpp
    src/WaypointExecutive.cpp
    src/Task.cpp
  )
  
  # Link libraries to test executable (plain signature)
  target_link_libraries(${PROJECT_NAME}_test
  tomlplusplus
  )
  
  # Add ament dependencies to test executable
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
    geometry_msgs
  )
endif()

# Installation
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()