# This builds cell_moveit_config, which has no code: it installs config/ (the SRDF and the
# MoveIt YAML files) into the package share, where MoveItConfigsBuilder in cell.launch.py
# reads them.
cmake_minimum_required(VERSION 3.16)
project(cell_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
