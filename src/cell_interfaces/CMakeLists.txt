# This builds cell_interfaces, the interface definitions shared by the cell's packages. So far
# that is one service, ResolveTarget, which the workspace node serves. It holds definitions
# only, no code, and C++ and Python nodes alike build against it.
cmake_minimum_required(VERSION 3.16)
project(cell_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResolveTarget.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
