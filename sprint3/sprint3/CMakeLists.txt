cmake_minimum_required(VERSION 3.8)
project(sprint3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

set(laserprocess_SRCS
  src/main.cpp
  src/laserprocess.cpp
)

add_executable(sprint3 ${laserprocess_SRCS})

ament_target_dependencies(
  sprint3
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  visualization_msgs
  nav_msgs
)

# Link against OpenCV
target_link_libraries(sprint3 ${OpenCV_LIBS})

install(TARGETS
  sprint3
  DESTINATION lib/${PROJECT_NAME})

ament_package()
