// Copyright 2020 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <moveit2_wrapper/moveit2_wrapper.hpp>

namespace moveit2_wrapper
{

Moveit2Wrapper::Moveit2Wrapper(const std::string node_name) : node_name_{node_name}
{  
}


bool Moveit2Wrapper::init()
{ 
  // Create node to be used by Moveit2
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>(node_name_, "", node_options);
  robot_state_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 
    10, std::bind(&Moveit2Wrapper::joint_state_callback, this, std::placeholders::_1));

  // Using the moveit_cpp API
  moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
  moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);
  populate_hashs();
  return true;
}


bool Moveit2Wrapper::state_to_state_motion(std::string planning_component, std::vector<double> state, int retries, 
                                           bool visualize, bool blocking)
{
  //std::cout << "state_to_state_motion() called for planning_component '" << planning_component << "'" << std::endl;

  if(planning_components_hash_.find(planning_component) == planning_components_hash_.end())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning component '" << planning_component << "' not found");
    return false;
  }
  if(state.size() != planning_components_hash_.at(planning_component).num_joints)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "State vector must contain same number of elements as planning component");
    return false;
  }

  // Set goal
  robot_state::RobotState goal_state(moveit_cpp_.get()->getRobotModel());
  goal_state.setJointGroupPositions(planning_component, &state[0]);
  bool ret = planning_components_hash_.at(planning_component).planning_component->setGoal(goal_state);
  if(!ret)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"Unable to set goal for planning component '" << planning_component << "'");
    return false;
  }

  // Plan solution
  int retries_left = retries;
  auto planned_solution = planning_components_hash_.at(planning_component).planning_component->plan();
  while(!planned_solution)
  {
    if(retries_left) 
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning for planning component '" << planning_component << 
        "' failed, " << retries_left  << " retries left.");
      planned_solution = planning_components_hash_.at(planning_component).planning_component->plan();
      retries_left--;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Planning for planning component '" << planning_component 
        << "' failed after " << retries << ". Aborting.");
      return false;
    } 
  } 

  if(visualize) 
  { 
    if(blocking) { visualize_trajectory(*planned_solution.trajectory); }
    else { RCLCPP_WARN_STREAM(node_->get_logger(), "Visualization is only available for blocking motion."); } 
  }

  // Generate msg and publish trajectory
  moveit_msgs::msg::RobotTrajectory traj_msg;
  planned_solution.trajectory->getRobotTrajectoryMsg(traj_msg);
  planning_components_hash_.at(planning_component).trajectory_publisher->publish(traj_msg.joint_trajectory);
  planning_components_hash_.at(planning_component).goal_reached = false;
  
  // Execute
  if(blocking) block_until_reached(state, planning_component);
  planning_components_hash_.at(planning_component).goal_reached = true;
  std::cout << "Goal state of planning component '" << planning_component << "' considered reached" << std::endl;
  return true;
}


bool Moveit2Wrapper::pose_to_pose_motion(std::string planning_component, std::vector<double> pose, int retries, 
                                         bool visualize, bool quat, bool blocking)
{
  //std::cout << "pose_to_pose_motion() called for planning_component '" << planning_component << "'" << std::endl;
  
  // Create pose message
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)->now();
  msg.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  msg.pose.position.x = pose[0];
  msg.pose.position.y = pose[1];
  msg.pose.position.z = pose[2];

  // Whether the orientation of the pose is given in Euler angles or Quaternions
  if(!quat)
  {
    KDL::Frame frame = pose_to_frame(pose);
    frame.M.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);  
  }
  else
  {
    msg.pose.orientation.x = pose[3];
    msg.pose.orientation.y = pose[4];
    msg.pose.orientation.z = pose[5];
    msg.pose.orientation.w = pose[6];
  }

  // Set goal
  if(!planning_components_hash_.at(planning_component).planning_component->setGoal(
      msg, planning_components_hash_.at(planning_component).joint_group->getLinkModelNames().back()))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"Unable to set goal for planning component '" << planning_component << "'");
    return false;
  }

  // Plan solution
  int retries_left = retries;
  auto planned_solution = planning_components_hash_.at(planning_component).planning_component->plan();
  while(!planned_solution)
  {
    if(retries_left) 
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Planning for planning component '" << planning_component << 
        "' failed, " << retries_left  << " retries left.");
      planned_solution = planning_components_hash_.at(planning_component).planning_component->plan();
      retries_left--;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Planning for planning component '" << planning_component 
        << "' failed after " << retries << ". Aborting.");
      return false;
    } 
  } 

  if(visualize) 
  { 
    if(blocking){ visualize_trajectory(*planned_solution.trajectory); }
    else { RCLCPP_WARN_STREAM(node_->get_logger(), "Visualization is only available for blocking motion."); }
  }

  // Generate msg and publish trajectory
  moveit_msgs::msg::RobotTrajectory traj_msg;
  planned_solution.trajectory->getRobotTrajectoryMsg(traj_msg);
  planning_components_hash_.at(planning_component).trajectory_publisher->publish(traj_msg.joint_trajectory);
  planning_components_hash_.at(planning_component).goal_reached = false;
  
  if(blocking) 
  {
    block_until_reached(pose, planning_component, 
      planning_components_hash_.at(planning_component).joint_group->getLinkModelNames().back());
  }
  planning_components_hash_.at(planning_component).goal_reached = true;
  std::cout << "Goal pose of planning component '" << planning_component << "' considered reached." << std::endl;
  return true;
}


bool Moveit2Wrapper::dual_arm_state_to_state_motion(std::vector<double> state_left, std::vector<double> state_right, 
                                                    int retries, bool visualize, bool blocking)
{
  //std::cout << "dual_arm_state_to_state_motion() called for planning component 'both_arms' " << std::endl;

  std::vector<double> state;
  state.insert(state.begin(), state_left.begin(), state_left.end());
  state.insert(state.end(), state_right.begin(), state_right.end());
  
  if( planning_components_hash_.at("left_arm").num_joints != state_left.size() )
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "state_left must have " << 
      planning_components_hash_.at("left_arm").num_joints << " joints");
    return false;
  }
  if( planning_components_hash_.at("right_arm").num_joints != state_right.size() )
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "state_right must have " << 
      planning_components_hash_.at("right_arm").num_joints << " joints");
    return false;
  }

  // Set goal
  robot_state::RobotState goal_state(moveit_cpp_.get()->getRobotModel());
  goal_state.setJointGroupPositions("both_arms", state);
  bool ret = planning_components_hash_.at("both_arms").planning_component->setGoal(goal_state);
  if(!ret)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to set goal for planning component 'both_arms'. Aborting.");
    return false;
  }

  // Plan solution
  int retries_left = retries;
  auto planned_solution = planning_components_hash_.at("both_arms").planning_component->plan();
  while(!planned_solution)
  {
    if(retries_left) 
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Planning for planning component " << "both_arms" << " failed, " << 
        retries_left  << " retries left.");
      planned_solution = planning_components_hash_.at("both_arms").planning_component->plan();
      retries_left--;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Planning for planning component " << "both_arms" << " failed after " 
        << retries << ". Aborting.");
      return false;
    } 
  } 

  if(visualize) { visualize_trajectory(*planned_solution.trajectory); }

  // Generate msg 
  moveit_msgs::msg::RobotTrajectory traj_msg, traj_msg_left, traj_msg_right;
  planned_solution.trajectory->getRobotTrajectoryMsg(traj_msg);

  // Copy header
  traj_msg_left.joint_trajectory.header = traj_msg.joint_trajectory.header;
  traj_msg_right.joint_trajectory.header = traj_msg.joint_trajectory.header;

  // Allocate for joint names[] and points[]
  traj_msg_left.joint_trajectory.joint_names.resize(7);
  traj_msg_right.joint_trajectory.joint_names.resize(7);
  traj_msg_left.joint_trajectory.points.resize(traj_msg.joint_trajectory.points.size());
  traj_msg_right.joint_trajectory.points.resize(traj_msg.joint_trajectory.points.size());

  // Copy joint_names
  std::copy_n(traj_msg.joint_trajectory.joint_names.begin(), 7, traj_msg_left.joint_trajectory.joint_names.begin());
  std::copy_n(traj_msg.joint_trajectory.joint_names.begin() + 7, 7, traj_msg_right.joint_trajectory.joint_names.begin());
  int index{0};

  // Copy the rest
  for(auto p : traj_msg.joint_trajectory.points)
  {
    // Allocate left
    traj_msg_left.joint_trajectory.points[index].positions.resize(7);
    traj_msg_left.joint_trajectory.points[index].velocities.resize(7);
    traj_msg_left.joint_trajectory.points[index].accelerations.resize(7);

    // Allocate right
    traj_msg_right.joint_trajectory.points[index].positions.resize(7);
    traj_msg_right.joint_trajectory.points[index].velocities.resize(7);
    traj_msg_right.joint_trajectory.points[index].accelerations.resize(7);
    
    // Copy left
    std::copy_n(p.positions.begin(), 7, traj_msg_left.joint_trajectory.points[index].positions.begin());
    std::copy_n(p.velocities.begin(), 7, traj_msg_left.joint_trajectory.points[index].velocities.begin());
    std::copy_n(p.accelerations.begin(), 7, traj_msg_left.joint_trajectory.points[index].accelerations.begin());

    // Copy right
    std::copy_n(p.positions.begin() + 7, 7, traj_msg_right.joint_trajectory.points[index].positions.begin());
    std::copy_n(p.velocities.begin() + 7, 7, traj_msg_right.joint_trajectory.points[index].velocities.begin());
    std::copy_n(p.accelerations.begin() + 7, 7, traj_msg_right.joint_trajectory.points[index].accelerations.begin());

    traj_msg_left.joint_trajectory.points[index].time_from_start = p.time_from_start;
    traj_msg_right.joint_trajectory.points[index].time_from_start = p.time_from_start;
    
    index++;
  }

  planning_components_hash_.at("left_arm").trajectory_publisher->publish(traj_msg_left.joint_trajectory);
  planning_components_hash_.at("right_arm").trajectory_publisher->publish(traj_msg_right.joint_trajectory);
  planning_components_hash_.at("both_arms").goal_reached = false;
  
  if(blocking) block_until_reached(state, "both_arms");
  planning_components_hash_.at("both_arms").goal_reached = true;
  std::cout << "Goal state of planning component 'both_arms' considered reached." << std::endl;
  return true;
}


void Moveit2Wrapper::launch_planning_scene()
{
  //std::cout << "Launching the planning scene." << std::endl;

  // Wait until robot is reporting its state on the the joint_states topic.
  while(!robot_ready_){ sleep(0.1); };
  joint_state_subscription_.reset(); // Subscription no longer needed.
  construct_planning_scene();
  sleep(3); // Wait so planning_scene is ready.
}


void Moveit2Wrapper::populate_hashs()
{
  /* Manual population of the hash table. Should be substituted by automatic loading of config files */

  PlanningComponentInfo right_arm_info;
  right_arm_info.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "right_arm", moveit_cpp_);
  right_arm_info.joint_group = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("right_arm"));
  right_arm_info.joint_names = right_arm_info.joint_group->getJointModelNames();
  right_arm_info.num_joints = right_arm_info.joint_names.size();
  right_arm_info.trajectory_publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/r/joint_trajectory_controller/joint_trajectory", 1);
  right_arm_info.goal_reached = false;
  right_arm_info.stop_signal_publisher = node_->create_publisher<std_msgs::msg::Bool>(
    "/r/joint_trajectory_controller/arm_stop", 1);

  PlanningComponentInfo left_arm_info;
  left_arm_info.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "left_arm", moveit_cpp_);
  left_arm_info.joint_group = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("left_arm"));
  left_arm_info.joint_names = left_arm_info.joint_group->getJointModelNames();
  left_arm_info.num_joints = left_arm_info.joint_names.size();
  left_arm_info.trajectory_publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/l/joint_trajectory_controller/joint_trajectory", 1);
  left_arm_info.goal_reached = false;
  left_arm_info.stop_signal_publisher =  node_->create_publisher<std_msgs::msg::Bool>(
    "/l/joint_trajectory_controller/arm_stop", 1);

  PlanningComponentInfo both_arms_info;
  both_arms_info.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "both_arms", moveit_cpp_);
  both_arms_info.joint_group = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("both_arms"));
  both_arms_info.joint_names = both_arms_info.joint_group->getJointModelNames();
  both_arms_info.num_joints = both_arms_info.joint_names.size();
  both_arms_info.trajectory_publisher = NULL;
  both_arms_info.goal_reached = false;
  both_arms_info.stop_signal_publisher = NULL;

  // Filling hash table of joint_states
  for(auto j : right_arm_info.joint_names)
  {
    joint_states_hash_[j] = 0.0;
  }
  for(auto j : left_arm_info.joint_names)
  {
    joint_states_hash_[j] = 0.0;
  }

  // Filling hash table of planning components
  planning_components_hash_["right_arm"] = right_arm_info;
  planning_components_hash_["left_arm"] = left_arm_info;
  planning_components_hash_["both_arms"] = both_arms_info;
}


void Moveit2Wrapper::construct_planning_scene()
{
  // Adding "desk_marius" - box representing Marius' desk.
  moveit_msgs::msg::CollisionObject col_obj;
  col_obj.header.frame_id = "yumi_base_link"; 
  col_obj.id = "desk_marius";
  shape_msgs::msg::SolidPrimitive desk;
  desk.type = desk.BOX;
  desk.dimensions = { 0.75, 0.5, 1.0 };
  geometry_msgs::msg::Pose desk_pose;
  desk_pose.position.x =  0.33 + desk.dimensions[0]/2.0;
  desk_pose.position.y =  0.56 + desk.dimensions[1]/2.0;
  desk_pose.position.z = -0.21 + desk.dimensions[2]/2.0;
  col_obj.primitives.push_back(desk);
  col_obj.primitive_poses.push_back(desk_pose);
  col_obj.operation = col_obj.ADD;

  // Adding "desk_endre" - box representing Endre's desk.
  moveit_msgs::msg::CollisionObject col_obj2;
  col_obj2.header.frame_id = "yumi_base_link"; 
  col_obj2.id = "desk_endre";
  shape_msgs::msg::SolidPrimitive desk2;
  desk2.type = desk2.BOX;
  desk2.dimensions = { 0.8, 0.1, 0.75 };
  geometry_msgs::msg::Pose desk2_pose;
  desk2_pose.position.x =   0.15 - desk2.dimensions[0]/2.0;
  desk2_pose.position.y = -(0.48 + desk2.dimensions[1]/2.0 - safety_margin_);
  desk2_pose.position.z =  -0.21 + desk2.dimensions[2]/2.0;
  col_obj2.primitives.push_back(desk2);
  col_obj2.primitive_poses.push_back(desk2_pose);
  col_obj2.operation = col_obj2.ADD;

  // Adding "pallet" - box representing the wooden pallet on which yumi is placed.
  moveit_msgs::msg::CollisionObject col_obj3;
  col_obj3.header.frame_id = "yumi_base_link"; 
  col_obj3.id = "pallet";
  shape_msgs::msg::SolidPrimitive pallet;
  pallet.type = pallet.BOX;
  pallet.dimensions = { 0.85, 0.70, 0.12 };
  geometry_msgs::msg::Pose pallet_pose;
  pallet_pose.position.x = 0.22 - pallet.dimensions[0]/2.0;
  pallet_pose.position.y = 0.0;
  pallet_pose.position.z = -0.09 - pallet.dimensions[2]/2.0;
  col_obj3.primitives.push_back(pallet);
  col_obj3.primitive_poses.push_back(pallet_pose);
  col_obj3.operation = col_obj3.ADD;

  // Adding "camera" - box representing the zivid camera.
  moveit_msgs::msg::CollisionObject col_obj4;
  col_obj4.header.frame_id = "yumi_base_link"; 
  col_obj4.id = "camera";
  shape_msgs::msg::SolidPrimitive camera;
  camera.type = pallet.BOX;
  camera.dimensions = { 0.40, 0.30, 0.20 };
  geometry_msgs::msg::Pose camera_pose;
  camera_pose.position.x = 0.10;
  camera_pose.position.y = 0.0;
  camera_pose.position.z = 0.70;
  col_obj4.primitives.push_back(camera);
  col_obj4.primitive_poses.push_back(camera_pose);
  col_obj4.operation = col_obj4.ADD;

  // Adding objects to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(col_obj);
    scene->processCollisionObjectMsg(col_obj2);
    scene->processCollisionObjectMsg(col_obj3);
    scene->processCollisionObjectMsg(col_obj4);
  }  // Unlock PlanningScene
}


void Moveit2Wrapper::update_joint_state_hash(std::string& planning_component)
{
  for(auto n : planning_components_hash_.at(planning_component).joint_names)
  {
    if(joint_states_hash_.find(n) != joint_states_hash_.end())
    {
      joint_states_hash_.at(n) = *moveit_cpp_->getCurrentState()->getJointPositions(n);
    }
  }
}


double Moveit2Wrapper::sum_error(std::vector<double>& goal_state, std::string& planning_component)
{
  double sum = 0.0;
  for(uint i = 0; i < goal_state.size(); ++i)
  {
    sum += std::abs(goal_state[i]-joint_states_hash_[planning_components_hash_.at(planning_component).joint_names[i]]);
  }
  return sum;
}


std::vector<double> Moveit2Wrapper::sum_error(std::vector<double>& goal_pose, std::vector<double>& curr_pose)
{
  double sum_pos = 0.0, sum_or = 0.0;
  for(int i = 0; i < 3; i++)
  {
    sum_pos += std::abs(goal_pose[i] - curr_pose[i]);
  }
  for(int i = 0; i < 4; i++)
  {
    sum_or += std::abs(goal_pose[3+i] - curr_pose[3+i]);
  }
  return std::vector<double>{sum_pos, sum_or};
}


void Moveit2Wrapper::block_until_reached(std::vector<double>& goal, std::string planning_component) 
{
  double summed_error = 1;
  update_joint_state_hash(planning_component);
  while(summed_error > 0.001)
  {
    summed_error = sum_error(goal, planning_component);
    if(summed_error > 1.22) rclcpp::sleep_for(std::chrono::seconds(1)); // Average of 10 degrees error on all joints
    else rclcpp::sleep_for(std::chrono::milliseconds(50));
    update_joint_state_hash(planning_component);
    //std::cout << "summed error: " << summed_error << std::endl;
  }
}


void Moveit2Wrapper::block_until_reached(std::vector<double>& goal_pose, std::string planning_component, 
                                         std::string link_name) 
{
  std::vector<double> errors(2, 1);
  std::vector<double> curr_pose = find_pose(link_name);
  while((errors[0] > 0.002) && (errors[1] > 0.001))
  {
    errors = sum_error(goal_pose, curr_pose);
    if(errors[0] > 0.10) rclcpp::sleep_for(std::chrono::seconds(1)); // Summed position error of 10cm
    else rclcpp::sleep_for(std::chrono::milliseconds(50));
    curr_pose = find_pose(link_name);
    //std::cout << "summed error, position: " << errors[0] << " summed error, orientation: " << errors[1]  << std::endl;
  }
}


KDL::Frame Moveit2Wrapper::pose_to_frame(std::vector<double>& pose)
{
  KDL::Vector vec(pose[0], pose[1], pose[2]);
  KDL::Rotation rot = KDL::Rotation().EulerZYX(angles::from_degrees(pose[3]), angles::from_degrees(pose[4]), 
                                               angles::from_degrees(pose[5]));
  return KDL::Frame(rot, vec);
}


std::vector<double> Moveit2Wrapper::find_pose(std::string link_name)
{
  const Eigen::Isometry3d& pose_eigen = moveit_cpp_->getCurrentState()->getGlobalLinkTransform(
      moveit_cpp_->getCurrentState()->getLinkModel(link_name));  
  
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  msg.pose = tf2::toMsg(pose_eigen);

  std::vector<double> vec; vec.resize(7);
  vec[0] = msg.pose.position.x;
  vec[1] = msg.pose.position.y;
  vec[2] = msg.pose.position.z;
  vec[3] = msg.pose.orientation.x;
  vec[4] = msg.pose.orientation.y;
  vec[5] = msg.pose.orientation.z;
  vec[6] = msg.pose.orientation.w;
  return vec;
}


void Moveit2Wrapper::visualize_trajectory(const robot_trajectory::RobotTrajectory& trajectory)
{
  moveit_msgs::msg::DisplayRobotState waypoint;
  const auto start_time = node_->now();

  // for all waypoints in trajectory ...
  for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
  {
    moveit::core::robotStateToRobotStateMsg(trajectory.getWayPoint(i), waypoint.state);
    const auto waypoint_time = start_time + rclcpp::Duration::from_seconds(trajectory.getWayPointDurationFromStart(i));
    const auto now = node_->now();

    // to ensure a timeapporpiate visualization of the waypoints
    if (waypoint_time > now)
    {
      rclcpp::sleep_for(std::chrono::nanoseconds((waypoint_time - now).nanoseconds()));
    }
    // publish the DisplayRobotState msg
    robot_state_publisher_->publish(waypoint);
  }
}


void Moveit2Wrapper::joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg)
{
  int counter = 14;
  for(auto value : msg->position)
  {
    if(value == 0) --counter;
  }
  if(counter != 14) robot_ready_ = true;
}


void Moveit2Wrapper::stop_planning_component(std::string planning_component)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  if(planning_component == "both_arms")
  {
    planning_components_hash_.at("left_arm").stop_signal_publisher->publish(msg);
    planning_components_hash_.at("right_arm").stop_signal_publisher->publish(msg);
  }
  else
  {
    planning_components_hash_.at(planning_component).stop_signal_publisher->publish(msg);
  }
}


} // namepsace moveit2_wrapper

