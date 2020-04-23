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

#include <moveit2_wrapper/table_monitor.hpp>

namespace moveit2_wrapper
{

TableMonitor::TableMonitor(moveit::planning_interface::MoveItCppPtr moveit_cpp) 
{
  moveit_cpp_ = moveit_cpp;
}


bool TableMonitor::init()
{
  std::vector<double> bin_pose = find_collision_object("bin");

  std::cout << "bin position: " << std::endl;
  for(int i = 0; i < 3; i++)
  {
    std::cout << bin_pose[i] << std::endl;
  }
  return true;
}


std::vector<double> TableMonitor::find_object(std::string object_id)
{
  if(objects_hash_.at(object_id).type == ObjectType::COLLISION_OBJECT)
  { 
    return find_collision_object(object_id); 
  }   
}


std::vector<double> TableMonitor::find_collision_object(std::string object_id)
{
  Eigen::Affine3d object_frame_transform;
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    object_frame_transform = scene->getFrameTransform(object_id);
  }  // Unlock PlanningScene (scopekill)

  geometry_msgs::msg::Pose pose_msg;
  tf2::convert(object_frame_transform, pose_msg);
  std::vector<double> pose; pose.resize(7);
  pose[0] = pose_msg.position.x;
  pose[1] = pose_msg.position.y;
  pose[2] = pose_msg.position.z;
  pose[3] = pose_msg.orientation.x;
  pose[4] = pose_msg.orientation.y;
  pose[5] = pose_msg.orientation.z;
  pose[6] = pose_msg.orientation.w;
  return pose;
}

} // namespace moveit2_wrapper