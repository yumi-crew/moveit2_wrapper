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
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/moveit_cpp/planning_component.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>

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
   * Pose-to-pose motion of a planning component. 
   * 
   * @param link the link to be moved to the desired pose.
   * @param pose desired pose. [quaternions]
   * @param eulerzyx flag indicating whether ZYX-Euler angles [degrees] are used instead of quaternions.
   * @param retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @param speed_scale scaling factor used to scale the velocity of the trajectory.
   * @param acc_scale scaling factor used to scale the acceleration of the trajectory.
   * 
   * @return true if the planner was able to plan to the goal.
   */
  bool pose_to_pose_motion(std::string planning_component, std::string link, std::vector<double> pose, 
                           bool eulerzyx=false, int retries=0, bool visualize=true, bool vis_abortable=false, 
                           bool blocking=true, double speed_scale=1, double acc_scale=1);

  /**
   * Cartesian straight-line pose-to-pose motion of a planning component. 
   * 
   * @param link the link to be moved to the desired pose. Link must be the last link of a registered joint group.
   * @param pose desired pose. [quaternions]
   * @param eulerzyx flag indicating whether ZYX-Euler angles [degrees] are used instead of quaternions.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @param min_percentage the required 'linearity' or 'straightness' of the motion.
   * @param speed_scale scaling factor used to scale the velocity of the trajectory.
   * @param acc_scale scaling factor used to scale the acceleration of the trajectory.
   * 
   * @return true if the planner was able to plan to the goal.
   */
  bool cartesian_pose_to_pose_motion(std::string planning_component, std::string link, std::vector<double> pose, 
                                     bool eulerzyx=false, bool visualize=true, bool vis_abortable=false,  
                                     bool blocking=true, bool collision_checking=true, double min_percentage=1, 
                                     double speed_scale=1, double acc_scale=1);
                                    
  /**
   * State-to-state motion of a planning component. 
   * 
   * @param state 7D-vector, desired joint configuration. [radians]
   * @param retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @param speed_scale scaling factor used to scale the velocity of the trajectory.
   * @param acc_scale scaling factor used to scale the acceleration of the trajectory.
   * 
   * @return true if the planner was able to plan to the goal.
   */
  bool state_to_state_motion(std::string planning_component, std::vector<double> state, int retries=0, 
                            bool visualize=true, bool vis_abortable=false, bool blocking=true, double speed_scale=1, 
                            double acc_scale=1);


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

  /* Launches the initial planning scene representing the enviroment around the robot.  */
  void launch_planning_scene();

  /** 
   * Determines if a given pose is reached by a link of the planning_component. 
   * 
   * @param link link to reach pose.
   * @param eulerzyx flag indicating if goal_pose is given in ZYX-Euler angles [degrees] instead of quaternions.
   */
  bool pose_reached(std::string planning_component, std::string link, std::vector<double> goal_pose, bool eulerzyx);

  /* Determines if a given state is reached by the planning component. */
  bool state_reached(std::string planning_component, std::vector<double> goal_state);

  /** 
   * Returns the pose given by a vector as a pose msg.
   * 
   * @param eulerzyx flag indicating if the vector represent orientation using ZYX-Euler angles [degrees] 
   *                 instead of quaternions.
   */
  geometry_msgs::msg::PoseStamped pose_vec_to_msg(std::vector<double> pose, bool eulerzyx);

  /* Returns the current configuration of the joints og the given planning component. */ 
  std::vector<double> get_current_state(std::string planning_component);

  /* Gives the pose of a desired link. [position given in meters, orientation in quaternions]
     {x_pos, y_pos, z_pos, x_quat, y_quat, z_quat, w_quat} */
  std::vector<double> find_pose(std::string link_name);

  Eigen::Matrix4d find_pose_matrix(std::string link_name);

  /* Converts an orientation given in ZYX-Euler angles [degrees] to one given by quaternions.*/
  std::vector<double> eulerzyx_to_quat(std::vector<double> orientation);

  std::vector<double> quat_to_eulerzyx(std::vector<double> orientation);

  /* Disables collisions between a link and an object. */
  void disable_collision(std::string link, std::string object_id);

  bool gripper_closed(std::string planning_component);
  bool gripper_open(std::string planning_component);

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
    std::string ee_link;
    std::string ee_joint;
    std::vector<double> home_configuration;
    std::unordered_map<std::string, std::shared_ptr<moveit::core::JointModelGroup>> secondary_joint_groups;
  };
  
  std::unordered_map<std::string, PlanningComponentInfo>* get_planning_components_hash()
    { return &planning_components_hash_; };

  moveit::planning_interface::MoveItCppPtr get_moveit_cpp(){ return moveit_cpp_; };

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  std::unordered_map<std::string, PlanningComponentInfo> planning_components_hash_;
  std::unordered_map<std::string, double> joint_states_hash_;
  
  double safety_margin_ = 0.04; // 4 cm
  double allowed_pos_error_ = 0.002; // 2 mm
  double allowed_or_errror_ = 0.015; // summed quaternion error
  double allowed_state_error_ = 0.001; // summed joint state error
  double maximum_planning_time_ = 2.0;
  double cartesian_max_step_ = 0.004;
  double joint_threshold_factor_ = 2;
  double joint_threshold_factor_limit_ = 6;
  std::string planning_pipeline_ = "ompl";

  void populate_hash_tables();
  void construct_planning_scene();
  void update_joint_state_hash(std::string& planning_component);
  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg);

  double sum_error(std::vector<double>& goal_state, std::string& planning_component);
  std::vector<double> sum_error(std::vector<double>& goal_pose, std::vector<double>& curr_pose);

  void block_until_reached(std::vector<double>& goal_state, std::string planning_component);
  
  /* Expects pose using quaterions. */
  void block_until_reached(std::vector<double>& goal_pose, std::string planning_component, std::string link_name);

  /**
   * Visualizes a computed trajectory. Return true if the visualization was allowed to compelete, false if not.
   * 
   * @param abortable flag indicating if the visualization is allowed to be aborted.  
   */
  bool visualize_trajectory(const robot_trajectory::RobotTrajectory& trajectory, std::string planning_component, 
                            bool abortable);

  /* Returns the pose given by a pose message as a 7D vector. [position given in meters, orientation in quaternions] */
  std::vector<double> pose_msg_to_vec(geometry_msgs::msg::PoseStamped msg);
  std::vector<double> pose_msg_to_vec(geometry_msgs::msg::Pose msg);

  /* Timeparameterizes a path by computing timestamps using iterative spline interpolation. Returns the trajectory. */
  robot_trajectory::RobotTrajectory time_parameterize_path(std::vector<moveit::core::RobotStatePtr> path, 
                                                           std::string planning_component, double speed_scale,
                                                           double acc_scale);
};

} // namespace moveit2_wrapper