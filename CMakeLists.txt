cmake_minimum_required(VERSION 3.8)
project(traj_interp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Trajectory Interpolator node
add_executable(trajectory_interpolator src/trajectory_interpolator.cpp)
ament_target_dependencies(trajectory_interpolator 
  rclcpp 
  px4_msgs 
  geometry_msgs 
  nav_msgs 
  std_msgs 
  trajectory_msgs 
  tf2
  tf2_geometry_msgs
  tf2_ros
  Eigen3
  tf2
  tf2_geometry_msgs
)
target_include_directories(trajectory_interpolator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(trajectory_interpolator Eigen3::Eigen)
install(TARGETS trajectory_interpolator DESTINATION lib/${PROJECT_NAME})

# Install launch files and config
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3 px4_msgs geometry_msgs nav_msgs std_msgs trajectory_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
