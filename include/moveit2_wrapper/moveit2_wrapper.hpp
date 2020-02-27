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

#pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


class Moveit2Wrapper
{
public:

  Moveit2Wrapper(std::string node_name);

  /** 
   * Initializes the wrapper. Initializes the palnning interface, KdlWrapper and sets up the necessary 
   * publishers and subscriptions.
   * 
   * @return true if the initialization is succesfull.
   */
  bool init();

  /**
   * Blocking end-effector pose-to-pose motion using KDL NR inverse solver with joint limits to find 
   * the joint configuration giving the desired pose, and planning in state space using OMPL via Moveit2.
   * 
   * @param pose 6D-vector, desired end-effector pose.
   * @return true if KDL could find a solution to the inverse kinematics, OMPL was able to plan to the goal
   *         and the robot was able to reach the desired configuration.
   */
  bool pose_to_pose_motion(std::vector<double>& pose, std::string planning_component, bool visualize);

  /**
   * Blocking joint state-to-state motion, planning in state-space using OMPL via Moveit2
   * 
   * @param state 7D-vector, dsired joint configurations
   * @return true if OMPL was able to plan to the goal, and the was able robot to reach desired configuration.
   */
  bool state_to_state_motion(std::vector<double>& state, std::string planning_component, bool visualize);
  
private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;

  KdlWrapper kdl;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  struct PlanningComponentInfo
  {
    std::shared_ptr<moveit::planning_interface::PlanningComponent> planning_component;
    std::shared_ptr<moveit::core::JointModelGroup> joint_group;
    int num_joints;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
  };
  std::map<std::string, PlanningComponentInfo> planning_component_hash_;


  // Helper functions
  void populate_hash();
  void construct_planning_scene();
  void visualize_trajectory(const robot_trajectory::RobotTrajectory& trajectory);
};