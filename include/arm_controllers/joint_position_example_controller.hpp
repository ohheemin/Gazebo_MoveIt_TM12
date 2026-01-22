#ifndef ARM_CONTROLLERS__JOINT_POSITION_EXAMPLE_CONTROLLER_HPP_
#define ARM_CONTROLLERS__JOINT_POSITION_EXAMPLE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"  // 추가!

namespace arm_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JointPositionExampleController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, 
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joint_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

 private:
  bool is_gazebo_;
  std::string robot_description_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;  // 추가!
  
  static constexpr int num_joints = 6;
  std::array<double, 6> initial_q_;
  std::array<double, 6> target_q_;
  std::array<double, 6> current_target_q_;  // 추가! (보간된 현재 목표)
  
  bool initialization_flag_ = true;
  bool has_active_trajectory_ = false;  // 추가!
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;  // 추가!
  rclcpp::Time trajectory_start_time_;  // 추가!
};

}  // namespace arm_controllers

#endif  // ARM_CONTROLLERS__JOINT_POSITION_EXAMPLE_CONTROLLER_HPP_
