cmake_minimum_required(VERSION 3.8)
project(ackermann_trajectory_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ackermann_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs_utils REQUIRED)

add_executable(${PROJECT_NAME}_node src/AckermannTrajectoryControlNode.cpp src/PID.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  ackermann_msgs
  perception_msgs
  perception_msgs_utils
  rclcpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_trajectory_planning_msgs
  trajectory_planning_msgs
  trajectory_planning_msgs_utils
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_dependencies(
  ackermann_msgs
  perception_msgs
  perception_msgs_utils
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_trajectory_planning_msgs
  trajectory_planning_msgs
  trajectory_planning_msgs_utils
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
 DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
)

install(
 DIRECTORY config
 DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Todo
endif()

ament_package()

