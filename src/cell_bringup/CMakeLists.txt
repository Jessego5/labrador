# This builds cell_bringup, which has no code: it installs the launch files and the RViz
# config into the package share, where ros2 launch cell_bringup sim.launch.py finds them.
cmake_minimum_required(VERSION 3.16)
project(cell_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
