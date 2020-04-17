#ifndef open_manipulator_x_tof_HPP_
#define open_manipulator_x_tof_HPP_

#include <termios.h>
#include <math.h> 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#define PI 3.14159265359
#define NUM_OF_JOINT 4
#define HEAD_OFFSET 0.050
#define MAX_DISTANCE 0.33
#define GRAB_THRESHOLD 0.02
#define MOVE_LOOP_MS 400
#define GRAB_WAIT_TIME_MS 4000

#define OBJECT_HEIGHT 0.12

namespace open_manipulator_x_tof
{
class OpenManipulatorXToFDemo : public rclcpp::Node
{
 public:
  OpenManipulatorXToFDemo();
  virtual ~OpenManipulatorXToFDemo();

 public:
  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  bool grabbing = false;
  std::thread moving;
  float target;
  float filtered_target;
  float grabbing_height;

  /*****************************************************************************
  ** ROS Timer and callback functions
  *****************************************************************************/
  rclcpp::TimerBase::SharedPtr update_timer_;

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_sub_;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr kinematics_pose_sub_;
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tof_marker_pub_;
  visualization_msgs::msg::Marker tof_marker;
  std::array<std::array<float,3>,6> tof_marker_colors;

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);
  void tof_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void move();

  bool set_joint_space_path(double angle1, double angle2, double angle3, double angle4, double path_time);
  bool set_task_space_path_from_present_position_only(double dx, double dy, double dz, double path_time);
  bool set_joint_space_path_from_present(double dangle1, double dangle2, double dangle3, double dangle4, double path_time);
  bool set_tool_control(bool state, double path_time);

  bool set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time);
  bool set_tool_control(std::vector<double> joint_angle);
  bool set_joint_space_path_from_present(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool home_position();
  double euler_distance(double x1, double y1, double z1, double x2, double y2, double z2);

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
}  // namespace open_manipulator_x_tof
#endif  // open_manipulator_x_tof_HPP_
