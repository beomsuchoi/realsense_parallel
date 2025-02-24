cmake_minimum_required(VERSION 3.8)
project(realsense_parallel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(parallel_detector src/parallel.cpp)
ament_target_dependencies(parallel_detector
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
)

install(TARGETS
  parallel_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()