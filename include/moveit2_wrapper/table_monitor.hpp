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

namespace moveit2_wrapper
{

class TableMonitor
{
public:
  TableMonitor(moveit::planning_interface::MoveItCppPtr moveit_cpp);

  /* Fills the hash table. */ 
  bool init();

  /* Adds the registered objects to the planning scene. */
  bool activate();

  /* Returns the pose of a registered object. */
  std::vector<double> find_object(std::string object_id);

  /* Moves a registered object to the given pose. */
  void move_object(std::string object_id, std::vector<double> pose);

  /* Removes a object from the planning scene. */
  void remove_object_from_scene(std::string object_id, bool update_scene);

  /* Returns the dimensions of a registered solid primitive. */
  std::vector<double> get_object_dimensions(std::string object_id);

private:
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  std::string reference_frame_ = "yumi_base_link";

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

  struct Mesh
  {
    std::string id;
  };

  struct ObjectData
  {
    std::string id;
    bool collision_object;
    ObjectType type;
    std::vector<double> last_observed_pose;
  };

  std::unordered_map<std::string, ObjectData> objects_hash_;
  std::unordered_map<std::string, SolidPrimitiveData> registered_solid_primitives_;
  std::unordered_map<std::string, Mesh> registered_meshs_;

  void populate_hash_tables();
  void update_pose_of_all_objects();
  void update_planning_scene();

  /* Adds a collision object to the hash table and the planning scene. */
  void add_primitive_object_to_scene(std::string object_id, std::vector<double> pose, bool update_scene);

  /* Returns the pose of a registered collision object. */
  std::vector<double> find_collision_object(std::string object_id);

  /* Moves a registered collision object to a new position. */
  void move_collision_object(std::string object_id, std::vector<double> new_pose, bool update_scene);
};

} // namespace moveit2_wrapper