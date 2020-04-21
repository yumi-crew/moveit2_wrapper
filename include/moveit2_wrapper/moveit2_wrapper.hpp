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
  Moveit2Wrapper(std::shared_ptr<rclcpp::Node> node);

  /** 
   * Initializes the wrapper: initializes the planning interface and sets up the necessary publishers and subscriptions.
   * 
   * @return true if the initialization is succesfull. False indicate failure.
   */
  bool init();

  /**
   *  Pose-to-pose motion of a joint group. The last link of the joint group is moved to the pose.
   * 
   * @param pose desired pose. [quaternions]
   * @param eulerzyx flag indicating if ZYX-Euler angles are used instead for quaternions.
   * @param retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * 
   * @return true if the planner was able to plan to the goal.
   */
  bool pose_to_pose_motion(std::string planning_component, std::vector<double> pose, bool eulerzyx=false, 
                           int retries=0, bool visualize=true, bool blocking=true);

  /**
   * State-to-state motion of a joint group.
   * 
   * @param state 7D-vector, desired joint configuration. [radians]
   * @param retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * 
   * @return true if the planner was able to plan to the goal.
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
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * 
   * @return true if the planner was able to plan to the goal.
   */
  bool dual_arm_state_to_state_motion(std::vector<double> state_left, std::vector<double> state_right, int retries=0, 
                                      bool visualize=true, bool blocking=true);

  /* Launches the planning scene. */
  void launch_planning_scene();

  /* Determines if a given pose is reached by the planning_component. */
  bool is_pose_reached(std::string planning_component, std::vector<double> goal_pose, bool eulerzyx);

  /* Moves an existing collision object to a new position. */
  void move_collison_object(std::string object_id, std::vector<double> new_pos);

  /* Updates the planning_scene. */
  void update_scene();
  
  struct PlanningComponentInfo
  {
    std::shared_ptr<moveit::planning_interface::PlanningComponent> planning_component;
    std::shared_ptr<moveit::core::JointModelGroup> joint_group;
    uint num_joints;
    std::vector<std::string> joint_names;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_publisher;
    bool in_motion;
    bool should_replan;
  };
  
  std::unordered_map<std::string, PlanningComponentInfo>* get_planning_components_hash()
    { return &planning_components_hash_; };


private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  std::unordered_map<std::string, PlanningComponentInfo> planning_components_hash_;
  std::unordered_map<std::string, double> joint_states_hash_;
  
  double safety_margin_ = 0.04; // 4 cm
  double allowed_pos_error_ = 0.002; // 2 mm
  double allowed_or_errror_ = 0.02; // summed quaternion error
  double allowed_state_error_ = 0.001; // summed joint state error

  void populate_hashs();
  void construct_planning_scene();
  void visualize_trajectory(const robot_trajectory::RobotTrajectory& trajectory);
  void update_joint_state_hash(std::string& planning_component);
  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg);

  double sum_error(std::vector<double>& goal_state, std::string& planning_component);
  std::vector<double> sum_error(std::vector<double>& goal_pose, std::vector<double>& curr_pose);

  void block_until_reached(std::vector<double>& goal_state, std::string planning_component);
  void block_until_reached(std::vector<double>& goal_pose, std::string planning_component, std::string link_name);

  /* Gives the pose of a desired link. [position given in meters, orientation in quaternions]
     {x_pos, y_pos, z_pos, x_quat, y_quat, z_quat, w_quat} */
  std::vector<double> find_pose(std::string link_name);

  /* Converts an orientation given in ZYX-Euler angles [degrees] to one given by quaternions.*/
  std::vector<double> eulerzyx_to_quat(std::vector<double> orientation);
};

} // namespace moveit2_wrapper