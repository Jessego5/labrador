# This builds cell_interfaces, the messages, services and actions shared by every package in the
# cell: the skill actions and the workspace's ResolveTarget service. It holds definitions only,
# no code, and both the C++ and the Python nodes build against it.
cmake_minimum_required(VERSION 3.16)
project(cell_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SkillError.msg"
  "srv/ResolveTarget.srv"
  "action/MoveToNamedPose.action"
  "action/Pick.action"
  "action/Place.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
