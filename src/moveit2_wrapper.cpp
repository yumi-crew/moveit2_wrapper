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

Moveit2Wrapper::Moveit2Wrapper(std::shared_ptr<rclcpp::Node> node) : node_{node}
{  
}


bool Moveit2Wrapper::init()
{ 
  robot_state_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);
  
  // Using the moveit_cpp API
  moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
  moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);
  populate_hash_tables();
  return true;
}


bool Moveit2Wrapper::state_to_state_motion(std::string planning_component, std::vector<double> state, int retries, 
                                           bool visualize, bool vis_abortable, bool blocking, double speed_scale, 
                                           double acc_scale)
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

  // Set planning parameters.
  moveit::planning_interface::PlanningComponent::PlanRequestParameters plannning_params;
  plannning_params.planning_attempts = retries;
  plannning_params.planning_time = maximum_planning_time_;
  plannning_params.max_velocity_scaling_factor = speed_scale;
  plannning_params.max_acceleration_scaling_factor = acc_scale;
  plannning_params.planning_pipeline = planning_pipeline_;

  // Plan solution
  int retries_left = retries;
  auto planned_solution = planning_components_hash_.at(planning_component).planning_component->plan(plannning_params);
  while(!planned_solution)
  {
    if(retries_left) 
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning for planning component '" << planning_component << 
        "' failed, " << retries_left  << " retries left.");
      planned_solution = planning_components_hash_.at(planning_component).planning_component->plan(plannning_params);
      retries_left--;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Planning for planning component '" << planning_component 
        << "' failed after " << retries << ". Aborting.");
      return false;
    } 
  } 

  // Visualize only if no other planning_components are in motion.
  if(visualize) 
  { 
    auto it = std::find_if(planning_components_hash_.begin(), planning_components_hash_.end(),
                          [](auto hash)->bool{ return hash.second.in_motion; });
    if(it == planning_components_hash_.end())
    { 
      // Abort motion if visualization is aborted.
      if(!visualize_trajectory(*planned_solution.trajectory, planning_component, vis_abortable)) return false;
    }
    else 
    { 
      RCLCPP_WARN(node_->get_logger(),"Visualization is only available when no other planning_component is in motion"); 
    }
  }

  // Generate msg and publish trajectory
  moveit_msgs::msg::RobotTrajectory traj_msg;
  planned_solution.trajectory->getRobotTrajectoryMsg(traj_msg);
  planning_components_hash_.at(planning_component).trajectory_publisher->publish(traj_msg.joint_trajectory);
  planning_components_hash_.at(planning_component).in_motion = true;
  
  if(blocking) 
  {
    block_until_reached(state, planning_component);
    planning_components_hash_.at(planning_component).in_motion = false;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal state of planning component '" << planning_component 
      << "' considered reached.");
  }
  return true;
}


bool Moveit2Wrapper::pose_to_pose_motion(std::string planning_component, std::string link, 
                                         std::vector<double> pose, bool eulerzyx, int retries, bool visualize, 
                                         bool vis_abortable, bool blocking, double speed_scale, double acc_scale)
{
  //std::cout << "pose_to_pose_motion() called for planning_component '" << planning_component << "'" << std::endl;

  if(!planning_components_hash_.at(planning_component).joint_group->hasLinkModel(link) && 
      link!=planning_components_hash_.at(planning_component).ee_link)
  {
    RCLCPP_ERROR(node_->get_logger(),"Link must be a member of the joint group or be the registered end-effector link.");
    return false;
  }
  
  // Create pose message. 
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)->now();
  msg.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  msg.pose.position.x = pose[0];
  msg.pose.position.y = pose[1];
  msg.pose.position.z = pose[2];

  // Within this class, orientation will always be represented using quaternions.
  if(eulerzyx)
  {
    std::vector<double> q_orien = eulerzyx_to_quat({pose[3], pose[4], pose[5]});
    msg.pose.orientation.x = q_orien[0];
    msg.pose.orientation.y = q_orien[1];
    msg.pose.orientation.z = q_orien[2];
    msg.pose.orientation.w = q_orien[3];
    std::copy_n(q_orien.begin(), 4, pose.begin()+3);
  }
  else
  {
    msg.pose.orientation.x = pose[3];
    msg.pose.orientation.y = pose[4];
    msg.pose.orientation.z = pose[5];
    msg.pose.orientation.w = pose[6];
  }

  // Set goal
  if(!planning_components_hash_.at(planning_component).planning_component->setGoal(msg, 
     planning_components_hash_.at(planning_component).ee_link))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"Unable to set goal for planning component '" << planning_component << "'");
    return false;
  }

  // Set planning parameteres.
  moveit::planning_interface::PlanningComponent::PlanRequestParameters plannning_params;
  plannning_params.planning_attempts = retries;
  plannning_params.planning_time = maximum_planning_time_;
  plannning_params.max_velocity_scaling_factor = speed_scale;
  plannning_params.max_acceleration_scaling_factor = acc_scale;
  plannning_params.planning_pipeline = planning_pipeline_;

  // Plan solution
  int retries_left = retries;
  auto planned_solution = planning_components_hash_.at(planning_component).planning_component->plan(plannning_params);
  while(!planned_solution)
  {
    if(retries_left) 
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Planning for planning component '" << planning_component << 
        "' failed, " << retries_left  << " retries left.");
      planned_solution = planning_components_hash_.at(planning_component).planning_component->plan(plannning_params);
      retries_left--;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Planning for planning component '" << planning_component 
        << "' failed after " << retries << ". Aborting.");
      return false;
    } 
  } 

  // Visualize only if no other planning_components are in motion.
  if(visualize) 
  { 
    auto it = std::find_if(planning_components_hash_.begin(), planning_components_hash_.end(),
                          [](auto hash)->bool{ return hash.second.in_motion; });
    if(it == planning_components_hash_.end())
    { 
      // Abort motion if visualization is aborted.
      if(!visualize_trajectory(*planned_solution.trajectory, planning_component, vis_abortable)) return false; 
    }
    else 
    { 
      RCLCPP_WARN(node_->get_logger(),"Visualization is only available when no other planning_component is in motion"); 
    }
  }

  // Generate msg and publish trajectory
  moveit_msgs::msg::RobotTrajectory traj_msg;
  planned_solution.trajectory->getRobotTrajectoryMsg(traj_msg);
  planning_components_hash_.at(planning_component).trajectory_publisher->publish(traj_msg.joint_trajectory);
  planning_components_hash_.at(planning_component).in_motion = true;

  if(blocking) 
  {
    block_until_reached(pose, planning_component, planning_components_hash_.at(planning_component).ee_link);
    planning_components_hash_.at(planning_component).in_motion = false;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal pose of planning component '" << planning_component 
      << "' considered reached.");
  }
  return true;
}


bool Moveit2Wrapper::cartesian_pose_to_pose_motion(std::string planning_component, std::string link, 
                                                   std::vector<double> pose, bool eulerzyx, bool visualize,
                                                   bool vis_abortable, bool blocking, double speed_scale, 
                                                   double acc_scale, double min_percentage)
{
  //std::cout << "cartesian_pose_to_pose_motion() called for link '" << link << "'" << std::endl;
  
  std::shared_ptr<moveit::core::JointModelGroup> joint_group;
  const moveit::core::LinkModel* link_model;

  // Find the correct joint_group.
  std::string last_link = planning_components_hash_.at(planning_component).joint_group->getLinkModelNames().back();
  bool found = false;
  if(last_link != link)
  {
    for(auto sec_joint_groups : planning_components_hash_.at(planning_component).secondary_joint_groups)
    {    
      last_link = sec_joint_groups.second->getLinkModelNames().back();
      if(last_link == link)
      {
        found = true;
        joint_group = sec_joint_groups.second;
        link_model = joint_group->getLinkModel(link);
        break;
      }
    }
    if(!found)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Link '" << link << "' is not the last link of any of the joint_groups "
         << "of planning component '" << planning_component << "'");
    }
  }
  else
  {
    joint_group = planning_components_hash_.at(planning_component).joint_group;
    link_model = joint_group->getLinkModel(link);
  }

  Eigen::Isometry3d target_frame; 
  geometry_msgs::msg::PoseStamped msg = pose_vec_to_msg(pose, eulerzyx);
  tf2::convert(msg.pose, target_frame);
  std::vector<moveit::core::RobotStatePtr> states;

  double factor = joint_threshold_factor_;
  double percentage = moveit_cpp_->getCurrentState()->computeCartesianPath(joint_group.get(), states, link_model,
                                                                           target_frame, true, cartesian_max_step_, factor);
  while(percentage < min_percentage)
  {
    factor += 0.5;
    percentage = moveit_cpp_->getCurrentState()->computeCartesianPath(joint_group.get(), states, link_model, 
                                                                      target_frame, true, cartesian_max_step_, factor);
  }
  //std::cout << "percentage: " << percentage << std::endl;
  //std::cout << "number of waypoints: " << states.size() << std::endl;

  robot_trajectory::RobotTrajectory robot_traj = time_parameterize_path(states,planning_component,speed_scale,acc_scale);

  // Visualize only if no other planning_components are in motion.
  if(visualize) 
  { 
    auto it = std::find_if(planning_components_hash_.begin(), planning_components_hash_.end(),
                          [](auto hash)->bool{ return hash.second.in_motion; });
    if(it == planning_components_hash_.end())
    { 
      // Abort motion if visualization is aborted.
      if(!visualize_trajectory(robot_traj, planning_component, vis_abortable)) return false;
    }
    else 
    { 
      RCLCPP_WARN(node_->get_logger(),"Visualization is only available when no other planning_component is in motion"); 
    }
  }

  moveit_msgs::msg::RobotTrajectory robot_traj_msg;
  robot_traj.getRobotTrajectoryMsg(robot_traj_msg);
  planning_components_hash_.at(planning_component).trajectory_publisher->publish(robot_traj_msg.joint_trajectory);
  planning_components_hash_.at(planning_component).in_motion = true;

  if(blocking) 
  {
    block_until_reached(pose, planning_component, link);
    planning_components_hash_.at(planning_component).in_motion = false;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal pose of planning component '" << planning_component 
      << "' considered reached.");
  }
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

  if(visualize) { visualize_trajectory(*planned_solution.trajectory, "both_arms", false); }

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
  planning_components_hash_.at("both_arms").in_motion = true;
  
  if(blocking)
  {
    block_until_reached(state, "both_arms");
    planning_components_hash_.at("both_arms").in_motion = false;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal state of planning component 'both_arms' considered reached.");
  }
  return true;
}


void Moveit2Wrapper::launch_planning_scene()
{
  RCLCPP_INFO_STREAM(node_->get_logger(),  "Launching the planning scene.");
  construct_planning_scene();
  moveit_cpp_->getPlanningSceneMonitor()->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_GEOMETRY);
  sleep(1);
}


void Moveit2Wrapper::populate_hash_tables()
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
  right_arm_info.stop_signal_publisher = node_->create_publisher<std_msgs::msg::Bool>(
    "/r/joint_trajectory_controller/arm_stop", 1);
  right_arm_info.in_motion = false;
  right_arm_info.should_replan = false;
  right_arm_info.ee_link = "gripper_r_base";
  right_arm_info.home_configuration = {0.0, -2.26, -2.35, 0.52, 0.0, 0.52, 0.0};
  right_arm_info.secondary_joint_groups["with_gripper"] = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("right_arm_with_gripper"));

  PlanningComponentInfo left_arm_info;
  left_arm_info.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "left_arm", moveit_cpp_);
  left_arm_info.joint_group = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("left_arm"));
  left_arm_info.joint_names = left_arm_info.joint_group->getJointModelNames();
  left_arm_info.num_joints = left_arm_info.joint_names.size();
  left_arm_info.trajectory_publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/l/joint_trajectory_controller/joint_trajectory", 1);
  left_arm_info.stop_signal_publisher =  node_->create_publisher<std_msgs::msg::Bool>(
    "/l/joint_trajectory_controller/arm_stop", 1);
  left_arm_info.in_motion = false;
  left_arm_info.should_replan = false;
  left_arm_info.ee_link = "gripper_l_base";
  left_arm_info.home_configuration = {0.0, -2.26, 2.35, 0.52, 0.0, 0.52, 0.0};
  left_arm_info.secondary_joint_groups["with_gripper"] = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("left_arm_with_gripper"));

  PlanningComponentInfo both_arms_info;
  both_arms_info.planning_component = std::make_shared<moveit::planning_interface::PlanningComponent>(
    "both_arms", moveit_cpp_);
  both_arms_info.joint_group = std::make_shared<moveit::core::JointModelGroup>(
    *moveit_cpp_->getRobotModel()->getJointModelGroup("both_arms"));
  both_arms_info.joint_names = both_arms_info.joint_group->getJointModelNames();
  both_arms_info.num_joints = both_arms_info.joint_names.size();
  both_arms_info.trajectory_publisher = NULL;
  both_arms_info.stop_signal_publisher = NULL;
  both_arms_info.in_motion = false;
  both_arms_info.should_replan = false;

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
  /* Manual loading of model of the robots enviroment. */

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
  camera.type = camera.BOX;
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
  }  // Unlock PlanningScene (scopekill)
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


void Moveit2Wrapper::block_until_reached(std::vector<double>& goal_state, std::string planning_component) 
{
  double summed_error = 1;
  update_joint_state_hash(planning_component);
  while(summed_error > allowed_state_error_)
  {
    summed_error = sum_error(goal_state, planning_component);
    // Calculate error more frequently when closer to the target.
    if(summed_error > 1.22) rclcpp::sleep_for(std::chrono::seconds(1)); 
    else rclcpp::sleep_for(std::chrono::milliseconds(50));
    update_joint_state_hash(planning_component);
    //std::cout << "summed error: " << summed_error << std::endl;
  }
}


void Moveit2Wrapper::block_until_reached(std::vector<double>& goal_pose, std::string planning_component, 
                                         std::string link_name) 
{
  std::vector<double> errors{1, 1};
  std::vector<double> curr_pose = find_pose(link_name);
  while((errors[0] > allowed_pos_error_) && (errors[1] > allowed_or_errror_))
  {
    errors = sum_error(goal_pose, curr_pose);
    // Calculate error more frequently when closer to the target.
    if(errors[0] > 0.10) rclcpp::sleep_for(std::chrono::seconds(1)); 
    else rclcpp::sleep_for(std::chrono::milliseconds(50));
    curr_pose = find_pose(link_name);
    //std::cout << "summed error, position: " << errors[0] << " summed error, orientation: " << errors[1]  << std::endl;
  }
}


std::vector<double> Moveit2Wrapper::find_pose(std::string link_name)
{
  const Eigen::Isometry3d& pose_eigen = moveit_cpp_->getCurrentState()->getGlobalLinkTransform(
      moveit_cpp_->getCurrentState()->getLinkModel(link_name));  
  
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  msg.pose = tf2::toMsg(pose_eigen);
  return pose_msg_to_vec(msg);
}


bool Moveit2Wrapper::visualize_trajectory(const robot_trajectory::RobotTrajectory& trajectory, 
                                          std::string planning_component, bool abortable)
{
  moveit_msgs::msg::DisplayRobotState waypoint;
  const auto start_time = node_->now();

  // for all waypoints in trajectory ...
  for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
  {
    moveit::core::robotStateToRobotStateMsg(trajectory.getWayPoint(i), waypoint.state);
    const auto waypoint_time = start_time + rclcpp::Duration::from_seconds(trajectory.getWayPointDurationFromStart(i));
    const auto now = node_->now();

    // to ensure a time-appropriate visualization of the waypoints
    if (waypoint_time > now)
    {
      rclcpp::sleep_for(std::chrono::nanoseconds((waypoint_time - now).nanoseconds()));
    }
    // publish the DisplayRobotState msg
    robot_state_publisher_->publish(waypoint);

    if(abortable && planning_components_hash_.at(planning_component).should_replan)
    {
      return false;
    }
  }
  return true;
}


bool Moveit2Wrapper::pose_reached(std::string planning_component, std::string link, std::vector<double> goal_pose, 
                                  bool eulerzyx)
{
  if(eulerzyx)
  {
    std::vector<double> q_orien = eulerzyx_to_quat({goal_pose[3], goal_pose[4], goal_pose[5]});
    goal_pose[3] = q_orien[0];
    goal_pose[4] = q_orien[1];
    goal_pose[5] = q_orien[2];
    goal_pose.push_back(q_orien[3]);
  }
  std::vector<double> errors{1, 1};
  std::vector<double> curr_pose = find_pose(link);  
  errors = sum_error(goal_pose, curr_pose);
  //std::cout << "position error: " << errors[0] << ", orientation error: " << errors[1] << std::endl;
  if((errors[0] < allowed_pos_error_) && (errors[1] < allowed_or_errror_)) return true;
  else return false;
}


bool Moveit2Wrapper::state_reached(std::string planning_component, std::vector<double> goal_state)
{
  update_joint_state_hash(planning_component);
  double summed_error = sum_error(goal_state, planning_component);
    
  if(summed_error < allowed_state_error_) return true; 
  else return false;
}


std::vector<double> Moveit2Wrapper::eulerzyx_to_quat(std::vector<double> orien)
{
  std::vector<double> q_orien; q_orien.resize(4);
  KDL::Rotation rot = KDL::Rotation().EulerZYX(angles::from_degrees(orien[0]), angles::from_degrees(orien[1]), 
                                               angles::from_degrees(orien[2]));
  rot.GetQuaternion(q_orien[0], q_orien[1], q_orien[2], q_orien[3]);
  return q_orien;
}


geometry_msgs::msg::PoseStamped Moveit2Wrapper::pose_vec_to_msg(std::vector<double> pose, bool eulerzyx)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  msg.pose.position.x = pose[0];
  msg.pose.position.y = pose[1];
  msg.pose.position.z = pose[2];

  if(eulerzyx)
  {
    std::vector<double> q_orien = eulerzyx_to_quat({pose[3], pose[4], pose[5]});
    msg.pose.orientation.x = q_orien[0];
    msg.pose.orientation.y = q_orien[1];
    msg.pose.orientation.z = q_orien[2];
    msg.pose.orientation.w = q_orien[3];
  }
  else
  {
    msg.pose.orientation.x = pose[3];
    msg.pose.orientation.y = pose[4];
    msg.pose.orientation.z = pose[5];
    msg.pose.orientation.w = pose[6];
  }
  return msg;
}


std::vector<double> Moveit2Wrapper::pose_msg_to_vec(geometry_msgs::msg::PoseStamped msg)
{
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


robot_trajectory::RobotTrajectory 
Moveit2Wrapper::time_parameterize_path(std::vector<moveit::core::RobotStatePtr> robot_states, 
                                       std::string planning_component, double speed_scale, double acc_scale)
{
  std::vector<std::string> joint_names = planning_components_hash_.at(planning_component).joint_names;
  moveit_msgs::msg::RobotTrajectory robot_traj_msg;
  robot_traj_msg.joint_trajectory.joint_names = joint_names;

  // Create trajectory lacking time stamps.
  for(auto robot_state : robot_states)
  {
    trajectory_msgs::msg::JointTrajectoryPoint traj_point;
    for(auto joint : joint_names)
    {
      traj_point.positions.push_back(robot_state->getVariablePosition(joint));
      traj_point.velocities.push_back(robot_state->getVariableVelocity(joint));
      traj_point.accelerations.push_back(robot_state->getVariableAcceleration(joint));
    }
    robot_traj_msg.joint_trajectory.points.push_back(traj_point);
  }

  // Initial and final velocity and acceleration should be zero.
  std::vector<double> zeros; zeros.assign(7, 0);
  robot_traj_msg.joint_trajectory.points.front().velocities = zeros;
  robot_traj_msg.joint_trajectory.points.front().accelerations = zeros;
  robot_traj_msg.joint_trajectory.points.back().velocities = zeros;
  robot_traj_msg.joint_trajectory.points.back().accelerations = zeros;

  robot_trajectory::RobotTrajectory robot_traj(moveit_cpp_->getRobotModel(),
    planning_components_hash_.at(planning_component).joint_group.get());
  robot_traj.setRobotTrajectoryMsg(*moveit_cpp_->getCurrentState(), robot_traj_msg);

  // Adding timestamps.
  trajectory_processing::IterativeSplineParameterization parameterization(true);
  parameterization.computeTimeStamps(robot_traj, speed_scale, acc_scale);
  return robot_traj;
}

} // namepsace moveit2_wrapper

