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
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/moveit_cpp/planning_component.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/kinematic_constraints/utils.h>



class Moveit2Wrapper
{
public:

  Moveit2Wrapper(const std::string node_name);

  /** 
   * Initializes the wrapper: initializes the planning interface, KdlWrapper and sets up the necessary 
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
   * @param retries number of allowed attempts at planning a trajectory.
   * @return true if KDL could find a solution to the inverse kinematics, OMPL was able to plan to the goal
   *         and the robot was able to reach the desired configuration.
   */
  bool pose_to_pose_motion(std::string planning_component, std::vector<double> pose, int retries=0, bool visualize=true, bool quat=false);

   /** 
   * (ABB YuMi SPECIFIC)
   * Blocking, concurrent dual arm state-to-state motion of both arms of the YuMi.
   * 
   * @param retries number of allowed attempts at planning a trajectory.
   * 
   */
  bool dual_arm_pose_to_pose_motion(std::vector<double> pose_left, std::vector<double> pose_right, int retries=0, 
                                    bool visualize=true);

  /**
   * Blocking joint state-to-state motion, planning in state-space using OMPL via Moveit2
   * 
   * @param state 7D-vector, desired joint configuration.
   * @param retries number of allowed attempts at planning a trajectory.
   * @return true if OMPL was able to plan to the goal, and the was able robot to reach desired configuration.
   */
  bool state_to_state_motion(std::string planning_component, std::vector<double> state, int retries=0, 
                            bool visualize=true);

   /** 
   * (ABB YuMi SPECIFIC)
   * Blocking, concurrent dual arm state-to-state motion of both arms of the YuMi.
   * 
   * @param state_left 7D-vector, desired joint configuration of the left arm.
   * @param state_right 7D-vector, desired joint configuration of the right arm.
   * @param retries number of allowed attempts at planning a trajectory.
   */
  bool dual_arm_state_to_state_motion(std::vector<double> state_left, std::vector<double> state_right, int retries=0, 
                                      bool visualize=true);

  /**
   * Launches the predfined planning scene representing the enviroment of the robot.
   */
  void launch_planning_scene();


  std::shared_ptr<rclcpp::Node> get_node() { return node_; }
  
private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  struct PlanningComponentInfo
  {
    std::shared_ptr<moveit::planning_interface::PlanningComponent> planning_component;
    std::shared_ptr<moveit::core::JointModelGroup> joint_group;
    uint num_joints;
    std::vector<std::string> joint_names;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    bool goal_reached;
  };

  std::unordered_map<std::string, PlanningComponentInfo> planning_components_hash_;
  std::unordered_map<std::string, double> joint_states_hash_;
  
  double safety_margin_;
  std::mutex planning_mutex;

  // Helper function
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

};