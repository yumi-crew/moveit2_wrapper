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
  populate_hash_tables();
  return true;
}


bool TableMonitor::activate()
{
  // Add all objects with known location to the planning scene. 
  for(auto object : objects_hash_)
  {
    if(!object.second.last_observed_pose.empty())
    {
      if(object.second.type == ObjectType::SOLID_PRIMITIVE)
      { 
        add_primitive_object_to_scene(object.second.id, object.second.last_observed_pose, false); 
      }
      else if(object.second.type == ObjectType::MESH)
      {
        add_mesh_to_scene(object.second.id, object.second.last_observed_pose, false);
      }
    }
  }
  update_planning_scene();
  return true;
}


std::vector<double> TableMonitor::find_object(std::string object_id)
{
  if(objects_hash_.at(object_id).collision_object)
  { 
    return find_collision_object(object_id); 
  }
  else
  {
    std::cout << "not yet implemented" << std::endl;
    return {};
  }
}


void TableMonitor::move_object(std::string object_id, std::vector<double> pose)
{
  if(objects_hash_.at(object_id).collision_object)
  {
    move_collision_object(object_id, pose, true);
  }
  else
  {
    std::cout << "not yet implemented" << std::endl;
  }
}


std::vector<double> TableMonitor::find_collision_object(std::string object_id)
{
  Eigen::Affine3d object_frame_transform;
  std::vector<double> pose; 

  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    if(scene->knowsFrameTransform(object_id))
    {
      object_frame_transform = scene->getFrameTransform(object_id);
    }
    else 
    {
      std::cout << "Planning scene do not know the frame transform associated with the object." << std::endl;
      return {};
    }
  }  // Unlock PlanningScene (scopekill)

  pose.resize(7);
  geometry_msgs::msg::Pose pose_msg;
  tf2::convert(object_frame_transform, pose_msg);
  
  pose[0] = pose_msg.position.x;
  pose[1] = pose_msg.position.y;
  pose[2] = pose_msg.position.z;
  pose[3] = pose_msg.orientation.x;
  pose[4] = pose_msg.orientation.y;
  pose[5] = pose_msg.orientation.z;
  pose[6] = pose_msg.orientation.w;
  return pose;
}


void TableMonitor::move_collision_object(std::string object_id, std::vector<double> new_pose, bool update_scene)
{
  moveit_msgs::msg::CollisionObject obj_msg;
  obj_msg.header.frame_id = reference_frame_; 
  obj_msg.id = object_id;

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = new_pose[0];
  pose_msg.position.y = new_pose[1];
  pose_msg.position.z = new_pose[2];
  pose_msg.orientation.x = new_pose[3];
  pose_msg.orientation.y = new_pose[4];
  pose_msg.orientation.z = new_pose[5];
  pose_msg.orientation.w = new_pose[6];

  obj_msg.primitive_poses.push_back(pose_msg);
  obj_msg.operation = obj_msg.MOVE;

  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(obj_msg);
  }  // Unlock PlanningScene (scopekill)

  if(update_scene) update_planning_scene();
}


void TableMonitor::populate_hash_tables()
{
  /* Manual fillout, should be replaced by config file parseing. */

  ObjectData bin_data;
  bin_data.id = "bin";
  bin_data.collision_object = true;
  bin_data.type = ObjectType::SOLID_PRIMITIVE;
  bin_data.last_observed_pose = {};

  SolidPrimitiveData bin;
  bin.id = "bin";
  bin.dimensions = {0.20, 0.20, 0.20};
  bin.type = SolidPrimitiveType::BOX;

  ObjectData screwdriver_data;
  screwdriver_data.id = "screwdriver";
  screwdriver_data.collision_object = true;
  screwdriver_data.type = ObjectType::MESH;
  screwdriver_data.last_observed_pose = {};
  screwdriver_data.grip_transforms = {};

  MeshData screwdriver;
  screwdriver.id = "screwdriver";

  objects_hash_["bin"] = bin_data;
  objects_hash_["screwdriver"] = screwdriver_data;
  registered_solid_primitives_["bin"] = bin;
  registered_meshs_["screwdriver"] = screwdriver;
}


void TableMonitor::update_pose_of_all_objects()
{
  for(auto object : objects_hash_)
  {
    object.second.last_observed_pose = find_object(object.second.id);
  }
}


void TableMonitor::add_object_to_scene(std::string object_id, std::vector<double> pose)
{
  if(registered_solid_primitives_.find(object_id) != registered_solid_primitives_.end())
  {
    add_primitive_object_to_scene(object_id, pose, true);
  }
  else if(registered_meshs_.find(object_id) != registered_meshs_.end())
  {
    add_mesh_to_scene(object_id, pose, true);
  }
}


void TableMonitor::add_primitive_object_to_scene(std::string object_id, std::vector<double> pose, bool update_scene)
{
  // Add to registered collision_objects
  moveit_msgs::msg::CollisionObject col_obj;
  col_obj.header.frame_id = reference_frame_;
  col_obj.id = object_id;

  shape_msgs::msg::SolidPrimitive obj_msg;
  obj_msg.type = SolidPrimitiveType::BOX;
  obj_msg.dimensions.push_back(registered_solid_primitives_.at(object_id).dimensions[0]);
  obj_msg.dimensions.push_back(registered_solid_primitives_.at(object_id).dimensions[1]);
  obj_msg.dimensions.push_back(registered_solid_primitives_.at(object_id).dimensions[2]);

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = pose[0];
  obj_pose.position.y = pose[1];
  obj_pose.position.z = pose[2];
  obj_pose.orientation.x = pose[3];
  obj_pose.orientation.y = pose[4];
  obj_pose.orientation.z = pose[5];
  obj_pose.orientation.w = pose[6];

  col_obj.primitives.push_back(obj_msg);
  col_obj.primitive_poses.push_back(obj_pose);
  col_obj.operation = col_obj.ADD;
   
  // Adding object to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(col_obj);
  }  // Unlock PlanningScene (scopekill)

  if(update_scene) update_planning_scene();
}


void TableMonitor::add_mesh_to_scene(std::string object_id, std::vector<double> pose, bool update_scene)
{
  // Add to registered collision_objects
  moveit_msgs::msg::CollisionObject col_obj;
  col_obj.header.frame_id = reference_frame_;
  col_obj.id = object_id;

  std::string resource_path = "package://"+stl_location+"/"+object_id+".stl";
  shapes::Mesh* m = shapes::createMeshFromResource("package://object_files/stl/screwdriver.stl");

  // Add mesh extents to register.
  Eigen::Vector3d extents = shapes::computeShapeExtents(m);
  registered_meshs_.at(object_id).mesh_extents = {extents[0], extents[1], extents[2]};
  
  // Matrix representing first 
  Eigen::Matrix4d grip1 = Eigen::Matrix4d::Identity();
  grip1(0, 3) = extents[0]/2;
  grip1(1, 3) = 1.0*extents[1]/4.0;
  grip1(2, 3) = -extents[2]/2.2;
  objects_hash_.at(object_id).grip_transforms.push_back(grip1);

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  mesh = boost::get<shape_msgs::msg::Mesh>(shape_msg);
  col_obj.meshes.push_back(mesh);

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = pose[0];
  obj_pose.position.y = pose[1];
  obj_pose.position.z = pose[2];
  obj_pose.orientation.x = pose[3];
  obj_pose.orientation.y = pose[4];
  obj_pose.orientation.z = pose[5];
  obj_pose.orientation.w = pose[6];

  col_obj.mesh_poses.push_back(obj_pose);
  col_obj.operation = col_obj.ADD;

  // Adding object to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(col_obj);
  }  // Unlock PlanningScene (scopekill)

  if(update_scene) update_planning_scene();
}


void TableMonitor::update_planning_scene()
{
  moveit_cpp_->getPlanningSceneMonitor()->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_GEOMETRY);
  moveit_cpp_->getPlanningSceneMonitor()->updateFrameTransforms();
}


void TableMonitor::remove_object_from_scene(std::string object_id, bool update_scene)
{
  moveit_msgs::msg::CollisionObject obj_msg;
  obj_msg.header.frame_id = reference_frame_; 
  obj_msg.id = object_id;

  obj_msg.operation = obj_msg.REMOVE;

  {  // Locks PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(obj_msg);
  }  // Unlocks PlanningScene

  if(update_scene) update_planning_scene();
}


std::vector<double> TableMonitor::get_object_dimensions(std::string object_id)
{
  if(registered_solid_primitives_.find(object_id) != registered_solid_primitives_.end())
  {
    return registered_solid_primitives_.at(object_id).dimensions;
  }
  else if(registered_meshs_.find(object_id) != registered_meshs_.end())
  {
    return registered_meshs_.at(object_id).mesh_extents;
  }
}


Eigen::Matrix4d TableMonitor::get_object_global_transform(std::string object_id)
{
  Eigen::Affine3d object_frame_transform;
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    if(scene->knowsFrameTransform(object_id))
    {
      object_frame_transform = scene->getFrameTransform(object_id);
    }
  }
  return object_frame_transform.matrix();
}

} // namespace moveit2_wrapper