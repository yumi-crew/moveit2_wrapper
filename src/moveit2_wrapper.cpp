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


Moveit2Wrapper::Moveit2Wrapper(const std::string node_name)
{ 
  std::cout << "entering Moveit2Wrapper constructor" << std::endl;
  
  // Create node to be used by Moveit2
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>(node_name, "", node_options);
  robot_state_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);
}


bool Moveit2Wrapper::init()
{ 
  std::cout << "entering Moveit2Wrapper::init()" << std::endl;
  
  std::cout << "before constructing moveit_cpp_" << std::endl;
  // Using the moveit_cpp API
  moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);

  moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);
  safety_margin_ = 0.04; //4cm

  std::cout << "before populate_hashs()" << std::endl;
  populate_hashs();
  return true;
}


bool Moveit2Wrapper::state_to_state_motion(std::vector<double> state, std::string planning_component, bool visualize, bool retry)
{
  std::cout << "entering Moveit2Wrapper::state_to_state_motion, planning_component: " << planning_component << std::endl;
  
  if(planning_components_hash_.find(planning_component) == planning_components_hash_.end())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning component " << planning_component << " not found");
    return false;
  }
  if(state.size() != planning_components_hash_.at(planning_component).num_joints)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "State vector must contain same number of elements as planning component");
    return false;
  }

  std::cout << "before set_goal()" << std::endl;
  // Set goal
  robot_state::RobotState goal_state(moveit_cpp_.get()->getRobotModel());
  goal_state.setJointGroupPositions(planning_component, &state[0]);
  bool ret = planning_components_hash_.at(planning_component).planning_component->setGoal(goal_state);
  if(!ret)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to set goal for planning component " << planning_component);
    return false;
  }

  std::cout << "before plan to goal()" << std::endl;
  // Plan solution
  const auto planned_solution = planning_components_hash_.at(planning_component).planning_component->plan();
  if (!planned_solution)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to plan a solution for planning component " << planning_component);
    if(retry) 
    {
      RCLCPP_INFO(node_->get_logger(), "Retry is activated. Second attempt initiated.");
      state_to_state_motion(state, planning_component, visualize, false);
    }
    else return false;
  } 

  std::cout << "before visualize" << std::endl;
  if(visualize) { visualize_trajectory(*planned_solution.trajectory); }

  // Generate msg and publish trajectory
  moveit_msgs::msg::RobotTrajectory traj_msg;
  planned_solution.trajectory->getRobotTrajectoryMsg(traj_msg);
  planning_components_hash_.at(planning_component).trajectory_publisher->publish(traj_msg.joint_trajectory);
  planning_components_hash_.at(planning_component).goal_reached = false;
  
  std::cout << "before checking if goal is reached" << std::endl;
  block_until_reached(state, planning_component);
  planning_components_hash_.at(planning_component).goal_reached = true;
  std::cout << "goal considered reached" << std::endl;

  return true;
}

void Moveit2Wrapper::block_until_reached(std::vector<double>& goal, std::string& planning_component) 
{
  double summed_error = 1;
  update_joint_state_hash(planning_component);
  while(summed_error > 0.001)
  {
    summed_error = sum_error(goal, planning_component);
    if(summed_error > 1.22) rclcpp::sleep_for(std::chrono::seconds(1)); //average of 10 degrees error on all joints
    else rclcpp::sleep_for(std::chrono::milliseconds(50));
    update_joint_state_hash(planning_component);
    std::cout << "summed error: " << summed_error << std::endl;
  }
}


double Moveit2Wrapper::sum_error(std::vector<double>& goal, std::string& planning_component)
{
  double sum = 0.0;
  for(uint i = 0; i < goal.size(); ++i)
  {
    sum += std::abs(goal[i]-joint_states_hash_[planning_components_hash_.at(planning_component).joint_names[i]]);
  }
  return sum;
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


bool Moveit2Wrapper::pose_to_pose_motion(std::vector<double> pose, std::string planning_component, bool visualize)
{
  //planning_components_hash_.at("right_arm").joint_group->getGroupKinematics().first.solver_instance_->getPositionIK()
  return true;
}


void Moveit2Wrapper::populate_hashs()
{
  /* Manual population of the hash table. Should be substituted by automatic loading of config file */

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

  PlanningComponentInfo both_arms_info;
  both_arms_info.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "both_arms", moveit_cpp_);
  both_arms_info.joint_group = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("both_arms"));
  both_arms_info.joint_names = both_arms_info.joint_group->getJointModelNames();
  both_arms_info.num_joints = both_arms_info.joint_names.size();
  both_arms_info.trajectory_publisher = NULL;
  both_arms_info.goal_reached = false;

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


void Moveit2Wrapper::launch_planning_scene()
{
  std::cout << "before construct_planning_scene" << std::endl;
  construct_planning_scene();
  std::cout << "after construct_planning_scene" << std::endl;
}


void Moveit2Wrapper::construct_planning_scene()
{
  // adding tall vertical box
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "yumi_base_link"; //reference frame, cannot be yumi_base_link
  collision_object.id = "box";
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.10, 0.10, 1.0 };
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.28;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.21 + box.dimensions[2]/2.0;
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // adding "desk" - box representing Marius' desk.
  moveit_msgs::msg::CollisionObject col_obj;
  col_obj.header.frame_id = "yumi_base_link"; //reference frame, cannot be yumi_base_link
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

  // adding "desk" - box representing Endre's desk.
  moveit_msgs::msg::CollisionObject col_obj2;
  col_obj2.header.frame_id = "yumi_base_link"; //reference frame, cannot be yumi_base_link
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

  // adding "wooden pallet" - box
  moveit_msgs::msg::CollisionObject col_obj3;
  col_obj3.header.frame_id = "yumi_base_link"; //reference frame, cannot be yumi_base_link
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

  // Adding objects to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(collision_object);
    scene->processCollisionObjectMsg(col_obj);
    scene->processCollisionObjectMsg(col_obj2);
    scene->processCollisionObjectMsg(col_obj3);
  }  // Unlock PlanningScene
}


void Moveit2Wrapper::visualize_trajectory(const robot_trajectory::RobotTrajectory& trajectory)
{
  /* Mutex access controlled as trajecotries cannot be visualized concurrently */

  visualize_mutex_.lock();
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
  visualize_mutex_.unlock();
}


bool Moveit2Wrapper::dual_arm_move(std::vector<double> state, bool visualize, bool retry)
{
  std::string planning_component = "both_arms";
  std::cout << "entering Moveit2Wrapper::dual_arm_move" <<  std::endl;
  
  if(planning_components_hash_.find(planning_component) == planning_components_hash_.end())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning component " << planning_component << " not found");
    return false;
  }
  if(state.size() != planning_components_hash_.at(planning_component).num_joints)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "State vector must contain same number of elements as planning component");
    return false;
  }

  std::cout << "before set_goal()" << std::endl;
  // Set goal
  robot_state::RobotState goal_state(moveit_cpp_.get()->getRobotModel());
  goal_state.setJointGroupPositions(planning_component, &state[0]);
  bool ret = planning_components_hash_.at(planning_component).planning_component->setGoal(goal_state);
  if(!ret)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to set goal for planning component " << planning_component);
    return false;
  }

  std::cout << "before plan to goal()" << std::endl;
  // Plan solution
  const auto planned_solution = planning_components_hash_.at(planning_component).planning_component->plan();
  if (!planned_solution)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to plan a solution for planning component " << planning_component);
    if(retry) 
    {
      RCLCPP_INFO(node_->get_logger(), "Retry is activated. Second attempt initiated.");
      state_to_state_motion(state, planning_component, visualize, false);
    }
    else return false;
  } 

  std::cout << "before visualize" << std::endl;
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
  std::cout << "before copying joint names" << std::endl;
  std::copy_n(traj_msg.joint_trajectory.joint_names.begin(), 7, traj_msg_left.joint_trajectory.joint_names.begin());
  std::copy_n(traj_msg.joint_trajectory.joint_names.begin() + 7, 7, traj_msg_right.joint_trajectory.joint_names.begin());
  int index{0};

  // Copy the rest
  std::cout << "before for loop" << std::endl;
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

    //std::cout << "before copy (for-loop)" << std::endl;
    
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

  std::cout << "before publishing left" << std::endl;
  planning_components_hash_.at("left_arm").trajectory_publisher->publish(traj_msg_left.joint_trajectory);

  std::cout << "before publishing right" << std::endl;
  planning_components_hash_.at("right_arm").trajectory_publisher->publish(traj_msg_right.joint_trajectory);

  planning_components_hash_.at("both_arms").goal_reached = false;
  
  std::cout << "before checking if goal is reached" << std::endl;;
  block_until_reached(state, planning_component);
  std::cout << "goal considered reached" << std::endl;

  planning_components_hash_.at("both_arms").goal_reached = true;
  return true;
}
