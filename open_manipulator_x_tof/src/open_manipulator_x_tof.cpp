#include "open_manipulator_x_tof/open_manipulator_x_tof.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace open_manipulator_x_tof
{
OpenManipulatorXToFDemo::OpenManipulatorXToFDemo()
: Node("open_manipulator_x_tof")
{
  /********************************************************************************
  ** Initialise variables
  ********************************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);
  count_to_retrieve = 0;
  last_target = 0;

  /********************************************************************************
  ** Initialise Subscribers
  ********************************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  auto qos_besteffort = rclcpp::QoS(rclcpp::KeepLast(1));
  qos_besteffort.best_effort();

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "open_manipulator_x/joint_states", qos, std::bind(&OpenManipulatorXToFDemo::joint_states_callback, this, _1));
  kinematics_pose_sub_ = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "open_manipulator_x/kinematics_pose", qos, std::bind(&OpenManipulatorXToFDemo::kinematics_pose_callback, this, _1));
  tof_sub = this->create_subscription<std_msgs::msg::Float32>(
    "sensors/tof", qos_besteffort, std::bind(&OpenManipulatorXToFDemo::tof_sensor_callback, this, _1));

  /********************************************************************************
  ** Initialise Clients
  ********************************************************************************/
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_tool_control");
  goal_task_space_path_from_present_position_only_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>(
    "open_manipulator_x/goal_task_space_path_from_present_position_only");
  goal_joint_space_path_from_present_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path_from_present");

  /********************************************************************************
  ** Display in terminal
  ********************************************************************************/

  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Teleop Keyboard Initialised");

  /********************************************************************************
  ** Execute home() and then move() in a loop
  ********************************************************************************/
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  home_position();
  update_timer_ = this->create_wall_timer(500ms, std::bind(&OpenManipulatorXToFDemo::move, this));
}

OpenManipulatorXToFDemo::~OpenManipulatorXToFDemo() 
{
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Teleop Keyboard Terminated");
}

/********************************************************************************
** Callback Functions
********************************************************************************/
void OpenManipulatorXToFDemo::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

void OpenManipulatorXToFDemo::kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void OpenManipulatorXToFDemo::tof_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  target =  msg->data + 0.040;
  target = (target > 0.33) ? 0.33 : target;
}


/********************************************************************************
** Move task
********************************************************************************/

void OpenManipulatorXToFDemo::move()
{ 
  float x =  target - present_kinematic_position_[0];
  std::cout << "POSITION: " << present_kinematic_position_[0] << " Going to " << target << "( move: " << x << ")\n";

  set_task_space_path_from_present_position_only(x, 0 , 0, 0.4);

  if ((fabs(last_target - target) < 0.01) && target <= 0.33){
    count_to_retrieve++;
    std::cout << "count_to_retrieve: " << count_to_retrieve << "\n";
  }else{
    count_to_retrieve = 0;
  }
  
  if (count_to_retrieve >= 8)
  {
    set_tool_control(false, 1.0);
    set_task_space_path_from_present_position_only(0.0, 0.0 , -0.150, 1.0);
    set_tool_control(true, 1.0);
    set_task_space_path_from_present_position_only(0.0, 0.0 , 0.150, 1.0);
    set_joint_space_path_from_present(-PI/2, 0.0, 0.0, 0.0, 1.0);
    set_task_space_path_from_present_position_only(0.0, 0.0 , -0.150, 1.0);
    set_tool_control(false, 1.0);
    set_task_space_path_from_present_position_only(0.0, 0.0 , 0.150, 1.0);
    home_position();
    count_to_retrieve = 0;
  }
  
  last_target = target;
}

/********************************************************************************
** Path functions
********************************************************************************/

bool OpenManipulatorXToFDemo::set_joint_space_path(double angle1, double angle2, double angle3, double angle4, double path_time)
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

  rclcpp::sleep_for(std::chrono::milliseconds(int(path_time* 1000)));

  return false;
}

bool OpenManipulatorXToFDemo::set_tool_control(bool state, double path_time)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(state ? -0.01 : 0.01);
  
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

  rclcpp::sleep_for(std::chrono::milliseconds(int(path_time* 1000)));

  return false;
}

bool OpenManipulatorXToFDemo::set_task_space_path_from_present_position_only(double dx, double dy, double dz, double path_time)
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

  rclcpp::sleep_for(std::chrono::milliseconds(int(path_time* 1000)));

  return false;
}

bool OpenManipulatorXToFDemo::set_joint_space_path_from_present(double dangle1, double dangle2, double dangle3, double dangle4, double path_time)
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

  rclcpp::sleep_for(std::chrono::milliseconds(int(path_time* 1000)));

  return false;
}

bool OpenManipulatorXToFDemo::home_position()
{
  set_joint_space_path(0.0, -PI/3, PI/9, PI*2/9, 1.0);
  // set_joint_space_path(0.0, 0.0, 0.0, PI/2, 1.0);

  set_task_space_path_from_present_position_only(0.200, 0.0 , -0.05, 2.0);
  set_tool_control(true, 1.0);
  
  return false;
}

}  // namespace open_manipulator_x_tof

/********************************************************************************
** Main
************************************************************************untu ********/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<open_manipulator_x_tof::OpenManipulatorXToFDemo>();
  
  while (rclcpp::ok()) {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}

