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
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


# moveit2_demo_yumi
add_library(moveit2_wrapper SHARED src/moveit2_wrapper.cpp)
target_include_directories(moveit2_wrapper PRIVATE include)
ament_target_dependencies(moveit2_wrapper
                          moveit_ros_planning_interface
                          Boost
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS moveit2_wrapper 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS test_demo
        DESTINATION lib/${PROJECT_NAME}
)


ament_export_libraries(moveit2_wrapper)
ament_export_include_directories(include)
ament_export_dependencies(moveit_ros_planning_interface)
ament_package()
