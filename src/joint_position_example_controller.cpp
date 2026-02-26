// Copyright (c) 2025 Tampere University, Autonomous Mobile Machines
// Licensed under the MIT License.

#include <arm_controllers/joint_position_example_controller.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace arm_controllers {

controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("joint_" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("joint_" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time&,
    const rclcpp::Duration& /*period*/) {
  
  if (initialization_flag_) {
    // Initialize all 6 joints
    for (int i = 0; i < 6; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
      target_q_.at(i) = initial_q_.at(i);
    }
    initialization_flag_ = false;
  }

  // Send commands to all 6 joints
  for (int i = 0; i < 6; ++i) {
    command_interfaces_[i].set_value(target_q_.at(i));
  }

  return controller_interface::return_type::OK;
}

void JointPositionExampleController::joint_command_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {

  //RCLCPP_INFO(
    //get_node()->get_logger(),
    //"[joint_command] received positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //msg->position.size() > 0 ? msg->position[0] : 0.0,
    //msg->position.size() > 1 ? msg->position[1] : 0.0,
    //msg->position.size() > 2 ? msg->position[2] : 0.0,
    //msg->position.size() > 3 ? msg->position[3] : 0.0,
    //msg->position.size() > 4 ? msg->position[4] : 0.0,
    //msg->position.size() > 5 ? msg->position[5] : 0.0
  //);

  if (msg->position.size() >= 6) {
    // Update all 6 joints
    for (int i = 0; i < 6; ++i) {
      target_q_.at(i) = msg->position[i];
    }
  }
}

CallbackReturn JointPositionExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();

  auto client = std::make_shared<rclcpp::SyncParametersClient>(
      get_node(), "/robot_state_publisher");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for service.");
      return CallbackReturn::ERROR;
    }
  }
  
  try {
    auto parameters = client->get_parameters({"robot_description"});
    if (!parameters.empty()) {
      robot_description_ = parameters[0].value_to_string();
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception: %s", e.what());
  }

  joint_command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "joint_command", 10,
      std::bind(&JointPositionExampleController::joint_command_callback,
                this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Controller subscribed to /joint_command");

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  return CallbackReturn::SUCCESS;
}

}  // namespace arm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arm_controllers::JointPositionExampleController,
  controller_interface::ControllerInterface
)
