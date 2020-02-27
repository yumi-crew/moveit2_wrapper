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


Moveit2Wrapper::Moveit2Wrapper(std::string node_name)
: node_(node_name)
{}


bool Moveit2Wrapper::init()
{
  // The planning interface
  moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
  moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

  populate_hash();
  construct_planning_scene();

  return true;
}


bool Moveit2Wrapper::state_to_state_motion(std::vector<double>& state, std::string planning_component, bool visualize)
{
  if(planning_components_map_.find(planning_component) != planning_components_map_.end())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning component " << planning_component << " not found");
    return false;
  }
  if(state.size() != planning_components_map_.at(planning_component).second.second)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "State vector must contain same number of elements as planning component");
    return false;
  }

  // Set goal
  robot_state::RobotState goal_state(moveit_cpp_.get()->getRobotModel());
  goal_state.setJointGroupPositions(planning_component, state);
  bool ret = planning_components_map_.at(planning_component).setGoal(goal_state);
  if(!ret_r)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to set goal");
    return false;
  }

  // Plan solution
  const auto planned_solution = right_planning_components_map_.at(planning_component).plan();
  if (!planned_solution)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to plan a solution");
    return false;
  } 

  if(visualize) visualize_trajectory(*planned_solution.trajectory);

  // Generate msg and publish trajectory
  moveit_msgs::msg::RobotTrajectory traj_msg;
  planned_solution.trajectory->getRobotTrajectoryMsg(traj_msg);
  planning_component_hash_.at(planning_component).trajectory_publisher->publish(traj_msg.joint_trajectory);
  
  // Block until goal state is reached 
  while(!planning_component_hash_.at(planning_component).joint_group->satisfiesPositionBounds(&state))
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  return true;
}


bool Moveit2Wrapper::pose_to_pose_motion(std::vector<double>& pose, std::string planning_component, bool visualize)
{
  return true;
}


void Moveit2Wrapper::populate_hash()
{
  /* Manual population of the hash table. Should probably be substituted by loading of a config file */

  PlanningComponentInfo right_arm_info;
  right_arm_info.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "right_arm", moveit_cpp_);
  right_arm_info.joint_group = moveit_cpp_->getRobotModel()->getJointModelGroup("right_arm");
  right_arm_info.num_joints = right_arm_info.joint_group->getJointModelNames().size();
  right_arm_info.trajectory_publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/r/joint_trajectory_controller/joint_trajectory", 1);

  PlanningComponentInfo left_arm_info;
  left_arm.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "right_arm", moveit_cpp_);
  left_arm_info.joint_group = moveit_cpp_->getRobotModel()->getJointModelGroup("left_arm");
  left_arm_info.num_joints = left_arm_info.joint_group->getJointModelNames().size();
  left_arm_info.trajectory_publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/l/joint_trajectory_controller/joint_trajectory", 1);

  planning_component_hash_.at("right_arm") = right_arm_info;
  planning_component_hash_.at("left_arm") = left_arm_info;
}


void Moveit2Wrapper::construct_planning_scene()
{
  // adding tall vertical box
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "yumi_body"; //reference frame, cannot be yumi_base_link
  collision_object.id = "box";
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.10, 0.10, 1.0 };
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.30;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.0+0.50-0.20;
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // adding "desk" - box
  moveit_msgs::msg::CollisionObject col_obj;
  col_obj.header.frame_id = "yumi_body"; //reference frame, cannot be yumi_base_link
  col_obj.id = "desk";
  shape_msgs::msg::SolidPrimitive desk;
  desk.type = desk.BOX;
  desk.dimensions = { 1.0, 0.5, 0.7 };
  geometry_msgs::msg::Pose desk_pose;
  desk_pose.position.x = 0.15+0.5;
  desk_pose.position.y = 0.45+0.25;
  desk_pose.position.z = 0.0+0.35-0.20;
  col_obj.primitives.push_back(desk);
  col_obj.primitive_poses.push_back(desk_pose);
  col_obj.operation = col_obj.ADD;

  // adding "desk" - box
  moveit_msgs::msg::CollisionObject col_obj2;
  col_obj2.header.frame_id = "yumi_body"; //reference frame, cannot be yumi_base_link
  col_obj2.id = "desk2";
  shape_msgs::msg::SolidPrimitive desk2;
  desk2.type = desk2.BOX;
  desk2.dimensions = { 1.0, 0.5, 0.7 };
  geometry_msgs::msg::Pose desk2_pose;
  desk2_pose.position.x = -(0.0+0.5);
  desk2_pose.position.y = -(0.80+0.25);
  desk2_pose.position.z = 0.0+0.35-0.20;
  col_obj2.primitives.push_back(desk2);
  col_obj2.primitive_poses.push_back(desk2_pose);
  col_obj2.operation = col_obj2.ADD;

  // adding "carboard box" - box
  moveit_msgs::msg::CollisionObject col_obj3;
  col_obj3.header.frame_id = "yumi_body"; //reference frame, cannot be yumi_base_link
  col_obj3.id = "cardboard";
  shape_msgs::msg::SolidPrimitive cardboard;
  cardboard.type = cardboard.BOX;
  cardboard.dimensions = { 0.8, 0.8, 0.5 };
  geometry_msgs::msg::Pose cardboard_pose;
  cardboard_pose.position.x = -(0.15+0.40);
  cardboard_pose.position.y = 0.45+0.40;
  cardboard_pose.position.z = 0.0+0.25-0.20;
  col_obj3.primitives.push_back(cardboard);
  col_obj3.primitive_poses.push_back(cardboard_pose);
  col_obj3.operation = col_obj3.ADD;

  // adding "wooden pallet" - box
  moveit_msgs::msg::CollisionObject col_obj4;
  col_obj4.header.frame_id = "yumi_body"; //reference frame, cannot be yumi_base_link
  col_obj4.id = "pallet";
  shape_msgs::msg::SolidPrimitive pallet;
  pallet.type = pallet.BOX;
  pallet.dimensions = { 1.0, 0.70, 0.10 };
  geometry_msgs::msg::Pose pallet_pose;
  pallet_pose.position.x = -0.40;
  pallet_pose.position.y = 0.0;
  pallet_pose.position.z = -(0.10+0.05);
  col_obj4.primitives.push_back(pallet);
  col_obj4.primitive_poses.push_back(pallet_pose);
  col_obj4.operation = col_obj4.ADD;

  // Adding objects to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(collision_object);
    scene->processCollisionObjectMsg(col_obj);
    scene->processCollisionObjectMsg(col_obj2);
    scene->processCollisionObjectMsg(col_obj3);
    scene->processCollisionObjectMsg(col_obj4);
  }  // Unlock PlanningScene
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

