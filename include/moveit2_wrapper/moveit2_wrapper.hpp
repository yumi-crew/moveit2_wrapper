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
#include <angles/angles.h>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/moveit_cpp/planning_component.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/kinematic_constraints/utils.h>

namespace moveit2_wrapper
{

class Moveit2Wrapper
{
public:
  Moveit2Wrapper(const std::string node_name);

  /** 
   * Initializes the wrapper: initializes the planning interface and sets up the necessary publishers and subscriptions.
   * 
   * @return true if the initialization is succesfull. False indicate failure.
   */
  bool init();

  /**
   * End-effector pose-to-pose motion of a joint group.
   * 
   * @param pose 6D-vector, desired end-effector pose. [ZYX-Euler angles or quaternions]
   * @param retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization only available for blocking motion.
   * @param quat flag indicating if quaternions are used to represent orientation.
   * @param blocking flag indicating if the function call should be blocking.
   * @return true if the planner was able to plan to the goal, and the joint group was able to reach the desired pose.
   */
  bool pose_to_pose_motion(std::string planning_component, std::vector<double> pose, int retries=0, bool visualize=true, 
                           bool quat=false, bool blocking=true);

  /**
   * State-to-state motion of a joint group.
   * 
   * @param state 7D-vector, desired joint configuration. [radians]
   * @param retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization only available for blocking motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @return true if the planner was able to plan to the goal, and the joint group was able to reach the desired 
   *         configuration.
   */
  bool state_to_state_motion(std::string planning_component, std::vector<double> state, int retries=0, 
                            bool visualize=true, bool blocking=true);


  /** 
   * Hardcoded dual arm state-to-state motion for the ABB YuMi.
   * 
   * @param state_left 7D-vector, desired joint configuration of the left arm. [radians]
   * @param state_right 7D-vector, desired joint configuration of the riht arm. [radians]
   * @param retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   * @param blocking flag indicating if the function call should be blocking.
   * @return true if the planner was able to plan to the goal, and the joint group was able to reach the desired 
   *         configuration.
   */
  bool dual_arm_state_to_state_motion(std::vector<double> state_left, std::vector<double> state_right, int retries=0, 
                                      bool visualize=true, bool blocking=true);


  void launch_planning_scene();
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }
  
protected:
  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  struct PlanningComponentInfo
  {
    std::shared_ptr<moveit::planning_interface::PlanningComponent> planning_component;
    std::shared_ptr<moveit::core::JointModelGroup> joint_group;
    uint num_joints;
    std::vector<std::string> joint_names;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    bool goal_reached;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_publisher;
  };

  std::unordered_map<std::string, PlanningComponentInfo> planning_components_hash_;
  std::unordered_map<std::string, double> joint_states_hash_;
  
  double safety_margin_ = 0.04; //4cm
  bool robot_ready_ = false;

  void populate_hashs();
  void construct_planning_scene();
  void visualize_trajectory(const robot_trajectory::RobotTrajectory& trajectory);
  void update_joint_state_hash(std::string& planning_component);

  double sum_error(std::vector<double>& goal, std::string& planning_component);
  std::vector<double> sum_error(std::vector<double>& goal_pose, std::vector<double>& curr_pose);

  void block_until_reached(std::vector<double>& goal, std::string planning_component);
  void block_until_reached(std::vector<double>& pose, std::string planning_component, std::string link_name);

  /* Converts a 6D pose vector {x,y,z, E(Z), E(Y), E(X)} into a Frame object. 
     Position given in meters, euler angles in degrees */
  KDL::Frame pose_to_frame(std::vector<double>& pose);

  /* Gives the pose of a desired link as a 7D pose vector {x_pos, y_pos, z_pos, x_quat, y_quat, z_quat, w_quat} 
     Position given in meters */
  std::vector<double> find_pose(std::string link_name);

  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg);
  void stop_planning_component(std::string planning_component);
};

} // namespace moveit2_wrapper