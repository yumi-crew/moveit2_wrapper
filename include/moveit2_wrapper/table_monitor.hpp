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

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_msgs/msg/collision_object.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_messages.h>

namespace moveit2_wrapper
{

class TableMonitor
{
public:
  TableMonitor(moveit::planning_interface::MoveItCppPtr moveit_cpp);

  /* Fills the hash table of registered objects. */ 
  bool init();

  /* Adds registered objects with known pose to the planning scene. */
  bool activate();

  /* Returns the pose of a registered object. [quaternions] */
  std::vector<double> find_object(std::string object_id);

  /* Adds a registered object to the planning scene. Pose given using quaternions. */
  void add_object_to_scene(std::string object_id, std::vector<double> pose);

  /* Moves a registered object to the given pose. Pose given using quaternions.*/
  void move_object(std::string object_id, std::vector<double> pose);

  /* Removes a registered object from the planning scene. */
  void remove_object_from_scene(std::string object_id, bool update_scene);

  /* Returns the dimensions of a registered object. */
  std::vector<double> get_object_dimensions(std::string object_id);

  Eigen::Matrix4d get_object_global_transform(std::string object_id);

  std::vector<Eigen::Matrix4d> get_grip_transforms(std::string object_id)
    { return objects_hash_.at(object_id).grip_transforms; }

private:
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  std::string reference_frame_ = "yumi_base_link";
  std::string stl_location = "object_files/stl";

  enum SolidPrimitiveType
  {
    BOX=1,
    SPHERE=2,
    CYLINDER=3,
    CONE=4
  };

  enum ObjectType
  {
    SOLID_PRIMITIVE = 1,
    MESH = 2
  };

  struct SolidPrimitiveData
  {
    std::string id;
    std::vector<double> dimensions;
    SolidPrimitiveType type;
  };

  struct MeshData
  {
    std::string id;
    std::vector<double> mesh_extents;
  };

  struct ObjectData
  {
    std::string id;
    bool collision_object;
    ObjectType type;
    std::vector<double> last_observed_pose;
    std::vector<Eigen::Matrix4d> grip_transforms;
  };

  std::unordered_map<std::string, ObjectData> objects_hash_;
  std::unordered_map<std::string, SolidPrimitiveData> registered_solid_primitives_;
  std::unordered_map<std::string, MeshData> registered_meshs_;

  void update_planning_scene();
  void populate_hash_tables();

  /* Iterates through the registered objects, finds the objects in the scene and updates the last_observed_pose field.*/
  void update_pose_of_all_objects();

  /* Adds a registered solid primitive to the planning scene. */
  void add_primitive_object_to_scene(std::string object_id, std::vector<double> pose, bool update_scene);

  /* Adds a registered mesh to the planning scene. */
  void add_mesh_to_scene(std::string object_id, std::vector<double> pose, bool update_scene);

  /* Returns the pose of a registered collision object. */
  std::vector<double> find_collision_object(std::string object_id);

  /* Moves a registered collision object to a new pose. */
  void move_collision_object(std::string object_id, std::vector<double> new_pose, bool update_scene);
};

} // namespace moveit2_wrapper