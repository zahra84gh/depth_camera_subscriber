#  PCL version: 1.12.1
# VTKversion: 9.0

cmake_minimum_required(VERSION 3.8)
project(depth_camera_subscriber)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)   
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS})

link_directories(
  ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  EIGEN3
  PCL
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  tf2_eigen)

add_executable(depth_camera_subscriber
src/depth_camera_subscriber.cpp
src/depth_camera_subscriber_node.cpp)

target_link_libraries (depth_camera_subscriber
${PCL_LIBRARIES})

ament_target_dependencies(depth_camera_subscriber
  pcl_conversions
  rclcpp
  sensor_msgs
  EIGEN3
  PCL
  nav_msgs
  visualization_msgs
  tf2_eigen
  ${dependencies}  )

install(TARGETS depth_camera_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION sinclude)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${dependencies})
ament_package()
