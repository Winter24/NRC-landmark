cmake_minimum_required(VERSION 3.5)
project(landmark_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Executables
add_executable(landmark_detection_circle
  src/landmark_detection_circle.cpp
)
ament_target_dependencies(landmark_detection_circle
  rclcpp
  sensor_msgs
  visualization_msgs
)
target_include_directories(landmark_detection_circle
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(landmark_detection_line_corner
  src/landmark_detection_line_corner.cpp
)
ament_target_dependencies(landmark_detection_line_corner
  rclcpp
  sensor_msgs
  visualization_msgs
)
target_include_directories(landmark_detection_line_corner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Install executables
install(TARGETS
  landmark_detection_circle
  landmark_detection_line_corner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
