#include "open_manipulator_x_teleop/open_manipulator_x_teleop_keyboard.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace open_manipulator_x_teleop_keyboard
{
OpenManipulatorXTeleopKeyboard::OpenManipulatorXTeleopKeyboard()
: Node("open_manipulator_x_teleop_keyboard")
{
  /********************************************************************************
  ** Initialise variables
  ********************************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /********************************************************************************
  ** Initialise Subscribers
  ********************************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  auto qosbe = rclcpp::QoS(rclcpp::KeepLast(1));
  qosbe.best_effort();

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "open_manipulator_x/joint_states", qos, std::bind(&OpenManipulatorXTeleopKeyboard::joint_states_callback, this, _1));
  kinematics_pose_sub_ = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "open_manipulator_x/kinematics_pose", qos, std::bind(&OpenManipulatorXTeleopKeyboard::kinematics_pose_callback, this, _1));
  tof_sub = this->create_subscription<std_msgs::msg::Float32>(
    "sensors/tof", qosbe, std::bind(&OpenManipulatorXTeleopKeyboard::tof_sensor_callback, this, _1));

  /********************************************************************************
  ** Initialise Clients
  ********************************************************************************/
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_tool_control");
  goal_task_space_path_from_present_position_only_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>(
    "open_manipulator_x/goal_task_space_path_from_present_position_only");
    // "open_manipulator_x/goal_task_space_path_from_present_position_only");
  goal_joint_space_path_from_present_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path_from_present");

  /********************************************************************************
  ** Display in terminal
  ********************************************************************************/

  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Teleop Keyboard Initialised");

  update_timer_ = this->create_wall_timer(500ms, std::bind(&OpenManipulatorXTeleopKeyboard::move, this));
  // this->run();
}

OpenManipulatorXTeleopKeyboard::~OpenManipulatorXTeleopKeyboard() 
{
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Teleop Keyboard Terminated");
}

/********************************************************************************
** Callback Functions
********************************************************************************/
void OpenManipulatorXTeleopKeyboard::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (uint8_t i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1")) temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2")) temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3")) temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4")) temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorXTeleopKeyboard::kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void OpenManipulatorXTeleopKeyboard::tof_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  target =  msg->data + 0.040;
  target = (target > 0.3) ? 0.3 : target;
}

void OpenManipulatorXTeleopKeyboard::move()
{
  float x =  target - present_kinematic_position_[0];
  std::cout << "POSITION: " << present_kinematic_position_[0] << " Going to " << target << "( move: " << x << ")\n";
  set_task_space_path_from_present_position_only_coordinates(x, 0 , 0, 0.4);
  
}

/********************************************************************************
** Callback Functions and Relevant Functions
********************************************************************************/

bool OpenManipulatorXTeleopKeyboard::set_joint_space_path_coordinates(double angle1, double angle2, double angle3, double angle4, double path_time)
{
  std::vector<double> joint_angle;
  std::vector<std::string> joint_name;

  joint_name.push_back("joint1"); joint_angle.push_back(angle1);
  joint_name.push_back("joint2"); joint_angle.push_back(angle2);
  joint_name.push_back("joint3"); joint_angle.push_back(angle3);
  joint_name.push_back("joint4"); joint_angle.push_back(angle4);

  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_tool_control_bool(bool state, double path_time)
{
  std::vector<double> joint_angle;
  if (state)
  {
      joint_angle.push_back(-0.01);
  }else{
      joint_angle.push_back(0.01);
  }
  
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper");
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_task_space_path_from_present_position_only_coordinates(double dx, double dy, double dz, double path_time)
{ 
  auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
  request->planning_group = "gripper";
  request->kinematics_pose.pose.position.x = dx;
  request->kinematics_pose.pose.position.y = dy;
  request->kinematics_pose.pose.position.z = dz;
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    going = false;
    return result->is_planned;
  };
  going = true;
  auto future_result = goal_task_space_path_from_present_position_only_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
  request->planning_group = "gripper";
  request->kinematics_pose.pose.position.x = kinematics_pose.at(0);
  request->kinematics_pose.pose.position.y = kinematics_pose.at(1);
  request->kinematics_pose.pose.position.z = kinematics_pose.at(2);
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_task_space_path_from_present_position_only_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_joint_space_path_from_present_coordinates(double dangle1, double dangle2, double dangle3, double dangle4, double path_time)
{
  std::vector<double> joint_angle;
  std::vector<std::string> joint_name;

  joint_name.push_back("joint1"); joint_angle.push_back(dangle1);
  joint_name.push_back("joint2"); joint_angle.push_back(dangle2);
  joint_name.push_back("joint3"); joint_angle.push_back(dangle3);
  joint_name.push_back("joint4"); joint_angle.push_back(dangle4);

  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_from_present_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_joint_space_path_from_present(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_from_present_client_->async_send_request(request, response_received_callback);

  return false;
}

/********************************************************************************
** Other Functions
********************************************************************************/


std::vector<double> OpenManipulatorXTeleopKeyboard::get_present_joint_angle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorXTeleopKeyboard::get_present_kinematics_pose()
{
  return present_kinematic_position_;
}

}  // namespace open_manipulator_x_teleop_keyboard

/********************************************************************************
** Main
********************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<open_manipulator_x_teleop_keyboard::OpenManipulatorXTeleopKeyboard>();

  std::vector<double> goalPose; goalPose.resize(3);
  std::vector<double> goalJoint; goalJoint.resize(4);
  const double delta = 0.01;
  const double joint_delta = 0.05;
  double path_time = 1;

  auto full = std::chrono::milliseconds(int(path_time* 1000));
  rclcpp::sleep_for (full);

  node->set_joint_space_path_coordinates(0.0, -PI/3, PI/9, PI*2/9, path_time);
  node->set_tool_control_bool(false, path_time);
  rclcpp::sleep_for (full);

  node->set_task_space_path_from_present_position_only_coordinates(0.100, 0 , -0.08, path_time);
  rclcpp::sleep_for (full);

  bool grip = true;
  node->set_tool_control_bool(grip, path_time);
  rclcpp::sleep_for (full);

  int N = 6;
  while (rclcpp::ok()) {
    rclcpp::spin(node);
  //   for (size_t i = 0; i < N; i++){
  //     node->set_task_space_path_from_present_position_only_coordinates(0, 0 , 0.100, path_time);
  //     rclcpp::sleep_for (full);
  //     node->set_joint_space_path_from_present_coordinates(PI/N, 0, 0, 0, path_time);
  //     rclcpp::sleep_for (full);
  //     node->set_task_space_path_from_present_position_only_coordinates(0, 0 , -0.100, path_time);
  //     rclcpp::sleep_for (full);
  //     grip = !grip;
  //     node->set_tool_control_bool(grip, path_time);
  //     rclcpp::sleep_for (full);
  //   }

  //   node->set_task_space_path_from_present_position_only_coordinates(0, 0 , 0.200, path_time);
  //   rclcpp::sleep_for (full);
  //   node->set_joint_space_path_from_present_coordinates(-PI, 0, 0, 0, 2*path_time);
  //   rclcpp::sleep_for (2*full);
  //   node->set_task_space_path_from_present_position_only_coordinates(0, 0 , -0.200, path_time);
  //   rclcpp::sleep_for (full);
  //   grip = !grip;
  //   node->set_tool_control_bool(grip, path_time);
  //   rclcpp::sleep_for (full);
  // }
  }
  rclcpp::shutdown();

  return 0;
}

