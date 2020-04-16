#include "open_manipulator_x_tof/open_manipulator_x_tof.hpp"
#include <thread>
#include <cmath>


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
  tof_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "sensors/tof", qos_besteffort, std::bind(&OpenManipulatorXToFDemo::tof_sensor_callback, this, _1));
  
  /********************************************************************************
  ** Initialise Marker Publishers
  ********************************************************************************/
  tof_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sensors/tof/marker", 1);
  tof_marker = visualization_msgs::msg::Marker();
  tof_marker.header.frame_id = "world";
  tof_marker.id = 0;
  tof_marker.type = 1;
  tof_marker.color.a = 1;
  tof_marker.color.r = 1;
  tof_marker.scale.x = 0.06;
  tof_marker.scale.y = 0.04;
  tof_marker.scale.z = 0.1;
  tof_marker.pose.position.x = 0.1;
  tof_marker.pose.position.y = 0;
  tof_marker.pose.position.z = tof_marker.scale.z/2;

  grabbing = false;

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

  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X ToF Controller Initialised");

  /********************************************************************************
  ** Execute home() and then move() in a loop
  ********************************************************************************/
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  home_position();
  // update_timer_ = this->create_wall_timer(std::chrono::milliseconds(MOVE_LOOP_MS), std::bind(&OpenManipulatorXToFDemo::move, this));
  moving = std::thread(std::bind(&OpenManipulatorXToFDemo::move, this));
}

OpenManipulatorXToFDemo::~OpenManipulatorXToFDemo() 
{
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X ToF Controller Terminated");
  moving.join();
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
  target =  msg->data + HEAD_OFFSET;
  target = (target > MAX_DISTANCE) ? MAX_DISTANCE : target;

  if (!grabbing && msg->data < MAX_DISTANCE){
    tof_marker.pose.position.x = msg->data + tof_marker.scale.x/2;
    tof_marker.pose.position.y = 0;
    tof_marker.pose.position.z = tof_marker.scale.z/2;
    tof_marker_pub_->publish(tof_marker);
  }else if(grabbing){
    tof_marker.pose.position.x = present_kinematic_position_[0];
    tof_marker.pose.position.y = present_kinematic_position_[1];
    tof_marker.pose.position.z = present_kinematic_position_[2];
    tof_marker_pub_->publish(tof_marker);
  }
}


/********************************************************************************
** Move task
********************************************************************************/

void OpenManipulatorXToFDemo::move()
{  
  int count_to_retrieve = 0;
  float last_target = 0;

  int positions = 3;
  int current_position = 0;
  int current_level = 0;


  while (rclcpp::ok()){
    float x =  target - present_kinematic_position_[0];
    std::cout << "POSITION: " << present_kinematic_position_[0] << " Going to " << target << "( move: " << x << ")\n";

    set_task_space_path_from_present_position_only(x, 0 , 0, (MOVE_LOOP_MS-100.0)/1000.0);

    if ((fabs(last_target - target) < GRAB_THRESHOLD) && target <= MAX_DISTANCE && target != 0){
      count_to_retrieve++;
      std::cout << "count_to_retrieve: " << count_to_retrieve << "\n";
    }else{
      count_to_retrieve = 0;
    }
    
    if (count_to_retrieve >= GRAB_WAIT_TIME_MS/MOVE_LOOP_MS){
      set_tool_control(false, 1.0);
      set_task_space_path_from_present_position_only(0.0, 0.0 , -0.100, 0.5);
      set_tool_control(true, 1.0);

      // Z = 0.63
      
      grabbing = true;
      set_task_space_path_from_present_position_only(0.0, 0.0 , 0.150, 0.5);

      // set_joint_space_path_from_present(-PI/2, 0.0, 0.0, 0.0, 1.0);

      set_joint_space_path(-PI/2, -PI/3, PI/9, PI*2/9, 1.0);
      std::cout << "grab position: " << current_position << "\n";

      set_task_space_path_from_present_position_only(0.0, -0.220 + (0.05 * (current_position % positions)) , -0.05, 1.0);
      current_position++;

      set_task_space_path_from_present_position_only(0.0, 0.0 , -0.100 + (0.12 * current_level), 2.0);
      current_level = ceil(current_position / positions);
      
      set_tool_control(false, 1.0);
      tof_marker.id++;
      grabbing = false;
      set_task_space_path_from_present_position_only(0.0, 0.0 , 0.150, 1.0);
      home_position();
      count_to_retrieve = 0;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    last_target = target;
  }
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
    return result->is_planned;
  };
  auto future_result = goal_task_space_path_from_present_position_only_client_->async_send_request(request, response_received_callback);

  rclcpp::sleep_for(std::chrono::milliseconds(int(path_time * 1000)));

  return false;
}

bool OpenManipulatorXToFDemo::home_position()
{
  set_joint_space_path(0.0, -PI/3, PI/9, PI*2/9, 1.0);
  set_task_space_path_from_present_position_only(0.200, 0.0 , -0.05, 2.0);
  set_tool_control(true, 1.0);

  return false;
}

double OpenManipulatorXToFDemo::euler_distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  double xSqr = pow((x1 - x2),2);
  double ySqr = pow((y1 - y2),2);
  double zSqr = pow((z1 - z2),2);

  return sqrt(xSqr + ySqr + zSqr);
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

  // rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor executor;
  // auto node = std::make_shared<open_manipulator_x_tof::OpenManipulatorXToFDemo>();
  // executor.add_node(node);
  // executor.spin();
  // rclcpp::shutdown();
  // return 0;
}

