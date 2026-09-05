# This builds cell_perception, the colour and depth object detector behind /perception/detect.
# The detector itself is a library with no ROS in it, so test_detector can run it on synthetic
# images; perception_node wraps it with the camera trigger, the service, TF and the debug image.
# config/ is installed so the node finds perception.yaml in the package share directory. Build and
# test it with colcon build and colcon test.
cmake_minimum_required(VERSION 3.16)
project(cell_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cell_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# the detector, without ROS
add_library(detector SHARED src/detector.cpp)
target_include_directories(detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(detector PUBLIC Eigen3::Eigen ${OpenCV_LIBS} PRIVATE yaml-cpp::yaml-cpp)
target_include_directories(detector SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node
  detector
  ament_index_cpp::ament_index_cpp
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${cell_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS detector EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector detector)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV)
ament_package()
