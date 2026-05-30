# This builds cell_workspace in two layers. cell_workspace_core is plain C++ over Eigen and
# yaml-cpp, the workspace model with no ROS in it, so it can be unit tested on its own;
# workspace_component wraps it in a node that publishes the TF frames and markers and serves
# /workspace/resolve_target. The tests are test_workspace for the model and test_mjcf_alignment,
# which checks the generated MuJoCo geometry against workspace.yaml. Build it with colcon build
# --packages-select cell_workspace.

cmake_minimum_required(VERSION 3.16)
project(cell_workspace)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cell_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# the ROS-free core: slot arithmetic, YAML loading and validation, which the tests and other
# packages link against without pulling in a node
add_library(cell_workspace_core SHARED
  src/workspace.cpp
  src/workspace_yaml.cpp
)
target_include_directories(cell_workspace_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(cell_workspace_core
  PUBLIC Eigen3::Eigen
  PRIVATE yaml-cpp::yaml-cpp
)

# the ROS node, as a composable component plus a standalone `workspace_node` executable
add_library(workspace_component SHARED src/workspace_node.cpp)
target_link_libraries(workspace_component
  cell_workspace_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${cell_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)
rclcpp_components_register_node(workspace_component
  PLUGIN "cell_workspace::WorkspaceNode"
  EXECUTABLE workspace_node
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS cell_workspace_core workspace_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)

  ament_add_gtest(test_workspace test/test_workspace.cpp)
  target_link_libraries(test_workspace cell_workspace_core)

  ament_add_gtest(test_mjcf_alignment test/test_mjcf_alignment.cpp)
  target_link_libraries(test_mjcf_alignment
    cell_workspace_core ament_index_cpp::ament_index_cpp tinyxml2::tinyxml2)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
