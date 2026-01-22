// Copyright (c) 2025 Tampere University, Autonomous Mobile Machines
// Licensed under the MIT License.

#include <arm_controllers/joint_position_example_controller.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace arm_controllers {

// Helper function to convert Duration to seconds
inline double duration_to_seconds(const builtin_interfaces::msg::Duration& duration) {
  return duration.sec + duration.nanosec * 1e-9;
}

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
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  
  if (initialization_flag_) {
    // Initialize all 6 joints
    for (int i = 0; i < 6; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
      target_q_.at(i) = initial_q_.at(i);
      current_target_q_.at(i) = initial_q_.at(i);
    }
    trajectory_start_time_ = time;
    has_active_trajectory_ = false;
    initialization_flag_ = false;
  }

  // Trajectory interpolation
  if (has_active_trajectory_ && !trajectory_points_.empty()) {
    double elapsed = (time - trajectory_start_time_).seconds();
    
    // Find the current segment
    size_t current_idx = 0;
    for (size_t i = 0; i < trajectory_points_.size(); ++i) {
      double point_time = duration_to_seconds(trajectory_points_[i].time_from_start);
      if (point_time <= elapsed) {
        current_idx = i;
      } else {
        break;
      }
    }
    
    if (current_idx < trajectory_points_.size()) {
      // Interpolate between points
      if (current_idx + 1 < trajectory_points_.size()) {
        auto& p1 = trajectory_points_[current_idx];
        auto& p2 = trajectory_points_[current_idx + 1];
        
        double t1 = duration_to_seconds(p1.time_from_start);
        double t2 = duration_to_seconds(p2.time_from_start);
        double alpha = (elapsed - t1) / (t2 - t1);
        alpha = std::clamp(alpha, 0.0, 1.0);
        
        for (int i = 0; i < 6; ++i) {
          current_target_q_.at(i) = p1.positions[i] + alpha * (p2.positions[i] - p1.positions[i]);
        }
      } else {
        // Last point - use it directly
        for (int i = 0; i < 6; ++i) {
          current_target_q_.at(i) = trajectory_points_[current_idx].positions[i];
        }
      }
      
      // Check if trajectory is complete
      if (current_idx == trajectory_points_.size() - 1 &&
          elapsed >= duration_to_seconds(trajectory_points_.back().time_from_start)) {
        has_active_trajectory_ = false;
      }
    }
  }

  // Send commands to all 6 joints
  for (int i = 0; i < 6; ++i) {
    command_interfaces_[i].set_value(current_target_q_.at(i));
  }

  return controller_interface::return_type::OK;
}

void JointPositionExampleController::joint_command_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {

  if (msg->position.size() >= 6) {
    // Update all 6 joints immediately (no interpolation)
    for (int i = 0; i < 6; ++i) {
      target_q_.at(i) = msg->position[i];
      current_target_q_.at(i) = msg->position[i];
    }
    has_active_trajectory_ = false;  // Cancel any active trajectory
  }
}

void JointPositionExampleController::joint_trajectory_callback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {

  if (msg->points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Received empty trajectory");
    return;
  }

  // Verify that we have 6 joints
  if (msg->joint_names.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                 "Expected 6 joints, got %zu", msg->joint_names.size());
    return;
  }

  // Store trajectory
  trajectory_points_.clear();
  trajectory_points_ = msg->points;
  trajectory_start_time_ = get_node()->now();
  has_active_trajectory_ = true;

  RCLCPP_INFO(get_node()->get_logger(), 
              "Received trajectory with %zu points", trajectory_points_.size());
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

  // Subscribe to both JointState (for backward compatibility) and JointTrajectory
  joint_command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_command", 10,
      std::bind(&JointPositionExampleController::joint_command_callback,
                this, std::placeholders::_1));

  joint_trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_command_trajectory", 10,
      std::bind(&JointPositionExampleController::joint_trajectory_callback,
                this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), 
              "Controller subscribed to /joint_command and /joint_command_trajectory");

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  has_active_trajectory_ = false;
  trajectory_points_.clear();
  return CallbackReturn::SUCCESS;
}

}  // namespace arm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arm_controllers::JointPositionExampleController,
  controller_interface::ControllerInterface
)
