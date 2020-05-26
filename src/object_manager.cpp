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

#include <moveit2_wrapper/object_manager.hpp>

namespace moveit2_wrapper
{

ObjectManager::ObjectManager(moveit::planning_interface::MoveItCppPtr moveit_cpp) 
{
  moveit_cpp_ = moveit_cpp;
}


bool ObjectManager::init()
{
  reference_frame_ = moveit_cpp_->getRobotModel()->getRootLinkName();
  populate_hash_tables();
  return true;
}


bool ObjectManager::activate()
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


std::vector<double> ObjectManager::find_object(std::string object_id)
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


void ObjectManager::move_object(std::string object_id, std::vector<double> pose)
{
  if(objects_hash_.at(object_id).collision_object)
  {
    move_collision_object(object_id, pose, true);
  }
  else std::cout << "not yet implemented" << std::endl;
}


std::vector<double> ObjectManager::find_collision_object(std::string object_id)
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
  }  // Unlock PlanningScene 

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


void ObjectManager::move_collision_object(std::string object_id, std::vector<double> new_pose, bool update_scene)
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

  if(objects_hash_.at(object_id).type == ObjectType::SOLID_PRIMITIVE) obj_msg.primitive_poses.push_back(pose_msg);
  else if( objects_hash_.at(object_id).type == ObjectType::MESH)      obj_msg.mesh_poses.push_back(pose_msg);
  
  obj_msg.operation = obj_msg.MOVE;

  std_msgs::msg::ColorRGBA color_msg;
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    color_msg = scene->getObjectColor(object_id);
  }

  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(obj_msg);
    scene->setObjectColor(object_id, color_msg);
  }  // Unlock PlanningScene 

  if(update_scene) update_planning_scene();
}


void ObjectManager::populate_hash_tables()
{
  // Testing dirname
  char result[100];
  ssize_t count = readlink("/proc/self/exe", result, 100);
  const char *path;
  if (count != -1) 
  {
    path = dirname(result);
  }
  sd::cout << "\n\n\n dirname: " << path << std::cout;

  //Testing ament_index package share path
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("my_package_name");
  std::cout << "\n\n\n package shared directory : " << package_share_directory << std::endl;

  char *buf = getlogin();
  std::string u_name = buf;
  std::string path = "/home/" + u_name + "/abb_ws/src/object_files/stl/";
  register_models(path);
}


void ObjectManager::register_models(std::string path_to_models_dir)
{
  std::string extension;
  std::string model_path;
  for (const auto &entry : std::experimental::filesystem::directory_iterator(path_to_models_dir))
  {
    extension = entry.path().string().substr(entry.path().string().find(".") + 1, 3);
    if (extension.compare("stl") == 0)
    {
      model_path = entry.path().string();
      std::cout << "register_model: "  << model_path << ", ";
      
      std::string name = model_path.substr(model_path.find_last_of("/") + 1);
      name = name.substr(0, name.find("."));
      std::cout << "as: " << name << std::endl;
  
      ObjectData object_data;
      object_data.id = name;
      object_data.collision_object = true;
      object_data.type = ObjectType::MESH;
      
      MeshData mesh_data;
      mesh_data.id = name;

      objects_hash_[name] = object_data;
      registered_meshs_[name] = mesh_data;
    }
  }
}


void ObjectManager::update_pose_of_all_objects()
{
  for(auto object : objects_hash_)
  {
    object.second.last_observed_pose = find_object(object.second.id);
  }
}


void ObjectManager::add_object_to_scene(std::string object_id, std::vector<double> pose, bool update_scene)
{
  if(registered_solid_primitives_.find(object_id) != registered_solid_primitives_.end())
  {
    add_primitive_object_to_scene(object_id, pose, update_scene);
  }
  else if(registered_meshs_.find(object_id) != registered_meshs_.end())
  {
    add_mesh_to_scene(object_id, pose, update_scene);
  }
}


void ObjectManager::add_primitive_object_to_scene(std::string object_id, std::vector<double> pose, bool update_scene)
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
  }  // Unlock PlanningScene 

  if(update_scene) update_planning_scene();
}


void ObjectManager::add_mesh_to_scene(std::string object_id, std::vector<double> pose, bool update_scene)
{
  // Add to registered collision_objects
  moveit_msgs::msg::CollisionObject col_obj;
  col_obj.header.frame_id = reference_frame_;
  col_obj.id = object_id;

  std::string resource_path = "package://"+stl_location+"/"+object_id+".stl";
  shapes::Mesh* m = shapes::createMeshFromResource(resource_path);

  // Add mesh extents to register.
  Eigen::Vector3d extents = shapes::computeShapeExtents(m);
  registered_meshs_.at(object_id).mesh_extents = {extents[0], extents[1], extents[2]};
  
  // Matrix representing first 
  Eigen::Matrix4d grip1 = Eigen::Matrix4d::Identity();
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
  }  // Unlock PlanningScene 

  if(update_scene) update_planning_scene();
}


void ObjectManager::update_planning_scene()
{
  moveit_cpp_->getPlanningSceneMonitor()->updateFrameTransforms();
  moveit_cpp_->getPlanningSceneMonitor()->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE);
  moveit_cpp_->getPlanningSceneMonitor()->updateFrameTransforms();
}


void ObjectManager::remove_object_from_scene(std::string object_id, bool update_scene)
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


std::vector<double> ObjectManager::get_object_dimensions(std::string object_id)
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


Eigen::Matrix4d ObjectManager::get_object_global_transform(std::string object_id)
{
  Eigen::Affine3d object_frame_transform;
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    if(scene->knowsFrameTransform(object_id))
    {
      object_frame_transform = scene->getFrameTransform(object_id);
    }
  } //unlock PlanningScene
  return object_frame_transform.matrix();
}


void ObjectManager::set_object_color(std::string object_id, std::vector<float> rgba)
{
  std_msgs::msg::ColorRGBA color;
  color.r = rgba[0];
  color.g = rgba[1];
  color.b = rgba[2];
  color.a = rgba[3];
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->setObjectColor(object_id, color);
  } //unlock PlanningScene
  update_planning_scene();
}


void ObjectManager::attach_object(std::string object_id, std::string link)
{
  moveit_msgs::msg::CollisionObject obj_msg;

  // Get CollisionObject msg
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->getCollisionObjectMsg(obj_msg, object_id);
  }  // Unlock PlanningScene 

  // Attach msg
  moveit_msgs::msg::AttachedCollisionObject a_obj_msg;
  a_obj_msg.object = obj_msg;
  a_obj_msg.link_name = link;

  // Remove old msg
  moveit_msgs::msg::CollisionObject r_obj_msg;
  r_obj_msg.header.frame_id = moveit_cpp_->getRobotModel()->getRootLinkName();
  r_obj_msg.id = object_id;
  r_obj_msg.operation = obj_msg.REMOVE;

  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(r_obj_msg);
    scene->processAttachedCollisionObjectMsg(a_obj_msg);
  }  

  update_planning_scene();
} 


void ObjectManager::detatch_object(std::string object_id)
{
  moveit_msgs::msg::AttachedCollisionObject a_obj_msg;

  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->getAttachedCollisionObjectMsg(a_obj_msg, object_id);
  }  // Unlock PlanningScene 

  a_obj_msg.object.operation =  a_obj_msg.object.REMOVE;

  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(a_obj_msg);
  }  // Unlock PlanningScene

  remove_object_from_scene(object_id, false);
  update_planning_scene();
}


std::string ObjectManager::object_held(std::string link)
{
  std::vector<moveit_msgs::msg::AttachedCollisionObject> msgs;
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->getAttachedCollisionObjectMsgs(msgs);
  }  // Unlock PlanningScene

  for(auto msg : msgs)
  {
    if(msg.link_name == link) return msg.object.id;
  }
}

} // namespace moveit2_wrapper