cmake_minimum_required(VERSION 3.8)
project(turtlebot3_wall_follower)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlebot3_lidar_processing REQUIRED)

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  turtlebot3_lidar_processing
)

include_directories(include)

# The navigator and its ROS wrapper build into one executable. Lidar and
# camera processing live in their own packages
# (turtlebot3_lidar_processing, turtlebot3_camera_processing).
add_executable(turtlebot3_navigator
  src/navigator_wrapper.cpp
  src/navigator.cpp
)
ament_target_dependencies(turtlebot3_navigator ${DEPENDENCIES})

# ── Install ──
install(TARGETS
  turtlebot3_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
