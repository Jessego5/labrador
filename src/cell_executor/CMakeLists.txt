# This builds cell_executor: the ROS-free core library (task parsing, the run context and the
# control nodes), the executor_node that adds the skill nodes and the ROS interfaces, the two
# gtest suites, and the install rules for the task files, the tree files and tree_status.py. The
# core is a library of its own so the tests can link it without a ROS node.
cmake_minimum_required(VERSION 3.16)
project(cell_executor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cell_interfaces REQUIRED)
find_package(cell_workspace REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# task parsing, the world model and the control nodes, with no ROS node, unit tested
add_library(cell_executor_core SHARED
  src/task.cpp
  src/run_context.cpp
  src/control_nodes.cpp
)
target_include_directories(cell_executor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(cell_executor_core
  PUBLIC behaviortree_cpp::behaviortree_cpp cell_workspace::cell_workspace_core
    ${cell_interfaces_TARGETS}
  PRIVATE yaml-cpp::yaml-cpp)

add_executable(executor_node src/executor_node.cpp src/skill_nodes.cpp)
target_link_libraries(executor_node
  cell_executor_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS cell_executor_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS executor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY tasks trees DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/tree_status.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task test/test_task.cpp)
  target_link_libraries(test_task cell_executor_core)
  target_compile_definitions(test_task PRIVATE TASKS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tasks")
  ament_add_gtest(test_tree test/test_tree.cpp)
  target_link_libraries(test_tree cell_executor_core)
  target_compile_definitions(test_tree PRIVATE TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp cell_interfaces cell_workspace)
ament_package()
