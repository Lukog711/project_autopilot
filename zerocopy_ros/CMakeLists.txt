cmake_minimum_required(VERSION 3.8)
project(zerocopy_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

#  Video Node
add_executable(video_pub src/video_pub.cpp)
target_link_libraries(video_pub ${OpenCV_LIBS})
ament_target_dependencies(video_pub rclcpp sensor_msgs cv_bridge image_transport)

install(TARGETS
  video_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/yolo_sub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
