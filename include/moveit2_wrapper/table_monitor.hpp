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

  bool init();

  std::vector<double> find_object(std::string object_id);

private:
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  enum ObjectType
  {
    COLLISION_OBJECT = 1
  };

  struct ObjectData
  {
    std::string id;
    ObjectType type;
    std::vector<double> last_observed_pose;
    double hover_height;
  };
  std::unordered_map<std::string, ObjectData> objects_hash_;

  std::vector<double> find_collision_object(std::string object_id);
};

} // namespace moveit2_wrapper