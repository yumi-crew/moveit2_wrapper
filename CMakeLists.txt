cmake_minimum_required(VERSION 3.5)
project(moveit2_wrapper)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(angles REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(object_files REQUIRED)

# moveit2_wrapper
add_library(moveit2_wrapper SHARED src/moveit2_wrapper.cpp)
target_link_libraries(moveit2_wrapper PRIVATE)
target_include_directories(moveit2_wrapper PRIVATE include)
ament_target_dependencies(moveit2_wrapper
                          moveit_ros_planning_interface
                          moveit_msgs
                          Boost
                          angles
                          tf2_eigen                                                 
)
# table_monitor
add_library(table_monitor SHARED src/table_monitor.cpp)
target_link_libraries(table_monitor libstdc++fs.a)
target_include_directories(table_monitor PRIVATE include)
ament_target_dependencies(table_monitor
                          moveit_ros_planning_interface
                          moveit_msgs
                          Boost
                          angles
                          tf2_eigen
                          geometric_shapes
                          object_files                                                 
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS moveit2_wrapper 
                table_monitor
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(moveit2_wrapper table_monitor)
ament_export_include_directories(include)
ament_export_dependencies(moveit_ros_planning_interface moveit_msgs)
ament_package()
