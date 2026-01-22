cmake_minimum_required(VERSION 3.5)
project(tm12_gazebo_moveit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(tm12_pick_and_place
  src/tm12_pick_and_place.cpp
)

target_include_directories(tm12_pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  tm12_pick_and_place
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
  tf2_geometry_msgs
)

add_library(joint_position_example_controller SHARED
  src/joint_position_example_controller.cpp
)


target_include_directories(joint_position_example_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  joint_position_example_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  sensor_msgs
  Eigen3
)

install(TARGETS
  tm12_pick_and_place
  joint_position_example_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    tm_config
    rviz2
    launch
    urdf
    xacro
    meshes
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface arm_controllers_plugin.xml)
install(FILES arm_controllers_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()

