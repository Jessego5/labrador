# This builds the cell's skills: a ROS-free library with the grasp geometry and the insertion
# search helpers, the skill server as a component, and the standalone skill_server executable
# that cell.launch.py runs. The geometry is kept in its own library so its gtests build and run
# without MoveIt or a running robot. It also installs run_pick_place_trials.py, the pick and
# place acceptance script.
cmake_minimum_required(VERSION 3.16)
project(cell_skills)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cell_interfaces REQUIRED)
find_package(cell_workspace REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# the ROS-free grasp geometry and insertion search helpers
add_library(grasp_geometry SHARED src/grasp_geometry.cpp src/insertion.cpp)
target_include_directories(grasp_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(grasp_geometry PUBLIC Eigen3::Eigen)

add_library(skill_server_component SHARED src/skill_server.cpp)
target_link_libraries(skill_server_component
  grasp_geometry
  cell_workspace::cell_workspace_core
  ament_index_cpp::ament_index_cpp
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_ros_planning_interface::moveit_planning_scene_interface
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${cell_interfaces_TARGETS}
  ${control_msgs_TARGETS}
  ${controller_manager_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
)
rclcpp_components_register_nodes(skill_server_component "cell_skills::SkillServer")

add_executable(skill_server src/skill_server_main.cpp)
target_link_libraries(skill_server skill_server_component)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS grasp_geometry skill_server_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS skill_server DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/run_pick_place_trials.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grasp_geometry test/test_grasp_geometry.cpp)
  target_link_libraries(test_grasp_geometry grasp_geometry)
  ament_add_gtest(test_insertion test/test_insertion.cpp)
  target_link_libraries(test_insertion grasp_geometry)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
