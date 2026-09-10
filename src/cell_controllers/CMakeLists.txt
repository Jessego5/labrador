# This builds cell_controllers, the Cartesian admittance controller the skills switch in for
# compliant placing and insertion. It is split in two: the admittance library holds the math and
# the KDL kinematics with no ROS in it, so test_admittance can check them on their own, and the
# cartesian_compliance_controller plugin wraps that library for ros2_control and is exported
# through cell_controllers.xml. Build and test it with colcon build and colcon test.
cmake_minimum_required(VERSION 3.16)
project(cell_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# the admittance math and KDL kinematics, without ROS
add_library(admittance SHARED src/admittance.cpp)
target_include_directories(admittance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(admittance PUBLIC Eigen3::Eigen orocos-kdl kdl_parser::kdl_parser)

add_library(cartesian_compliance_controller SHARED src/cartesian_compliance_controller.cpp)
target_link_libraries(cartesian_compliance_controller
  admittance
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${geometry_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(controller_interface cell_controllers.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS admittance cartesian_compliance_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_admittance test/test_admittance.cpp)
  target_link_libraries(test_admittance admittance)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface Eigen3 kdl_parser orocos_kdl rclcpp_lifecycle realtime_tools)
ament_package()
