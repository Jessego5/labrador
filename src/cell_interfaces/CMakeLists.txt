# This builds cell_interfaces, the messages, services and actions shared by every package in the
# cell: the skill actions, the perception and workspace services, and the executor's
# ExecuteTask. It holds definitions only, no code, and both the C++ and the Python nodes build
# against it.
cmake_minimum_required(VERSION 3.16)
project(cell_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/SkillError.msg"
  "srv/DetectObjects.srv"
  "srv/LabReset.srv"
  "srv/ResolveTarget.srv"
  "srv/StartRun.srv"
  "action/DetectObjects.action"
  "action/ExecuteTask.action"
  "action/Insert.action"
  "action/MoveToNamedPose.action"
  "action/Pick.action"
  "action/Place.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
