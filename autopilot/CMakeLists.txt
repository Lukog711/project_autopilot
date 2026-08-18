cmake_minimum_required(VERSION 3.8)
project(autopilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++17 for std::filesystem and std::async
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenCV REQUIRED)

# Threads for async and UI multithreading
find_package(Threads REQUIRED)

# 1. Obstacle Node
add_executable(closest_obstacle_node src/closest_obstacle_node.cpp)
ament_target_dependencies(closest_obstacle_node rclcpp sensor_msgs std_msgs livox_ros_driver2)

# 2. Braking Node
add_executable(safe_braking_node src/safe_braking_node.cpp)
ament_target_dependencies(safe_braking_node rclcpp std_msgs)

# 3. Health Monitor Node (Needs Threads for std::async)
add_executable(health_monitor_node src/health_monitor_node.cpp)
ament_target_dependencies(health_monitor_node rclcpp std_msgs)
target_link_libraries(health_monitor_node Threads::Threads)

# 4. Decision Node (UI) (Needs OpenCV and Threads)
add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node rclcpp std_msgs OpenCV)
target_link_libraries(decision_node ${OpenCV_LIBS} Threads::Threads)

install(TARGETS 
  closest_obstacle_node 
  safe_braking_node 
  health_monitor_node 
  decision_node 
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
