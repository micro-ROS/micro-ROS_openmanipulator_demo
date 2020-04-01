/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_X_TELEOP_KEYBOARD_HPP_
#define OPEN_MANIPULATOR_X_TELEOP_KEYBOARD_HPP_

#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>

#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#define PI 3.14159265359
#define NUM_OF_JOINT 4


namespace open_manipulator_x_teleop_keyboard
{
class OpenManipulatorXTeleopKeyboard : public rclcpp::Node
{
 public:
  OpenManipulatorXTeleopKeyboard();
  virtual ~OpenManipulatorXTeleopKeyboard();

 public:
  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  bool going = false;
  float target;

  /*****************************************************************************
  ** ROS Timer and callback functions
  *****************************************************************************/
  rclcpp::TimerBase::SharedPtr update_timer_;

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_sub;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr kinematics_pose_sub_;

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);
  void tof_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void move();

  bool set_joint_space_path_coordinates(double angle1, double angle2, double angle3, double angle4, double path_time);
  bool set_task_space_path_from_present_position_only_coordinates(double dx, double dy, double dz, double path_time);
  bool set_joint_space_path_from_present_coordinates(double dangle1, double dangle2, double dangle3, double dangle4, double path_time);
  bool set_tool_control_bool(bool state, double path_time);

  bool set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time);
  bool set_tool_control(std::vector<double> joint_angle);
  bool set_joint_space_path_from_present(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);

  /*****************************************************************************
  ** ROS Clients
  *****************************************************************************/
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_from_present_client_;

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  std::vector<double> get_present_joint_angle();
  std::vector<double> get_present_kinematics_pose();
};
}  // namespace open_manipulator_x_teleop_keyboard
#endif  // OPEN_MANIPULATOR_X_TELEOP_KEYBOARD_HPP_
