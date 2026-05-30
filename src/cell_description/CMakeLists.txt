# This builds cell_description, which has nothing to compile: it installs the config, mjcf and
# urdf directories into the package share so the launch files, the workspace node and the tests
# can find them with get_package_share_directory. The scripts are not installed; they are run
# from the source tree to regenerate the MJCF.

cmake_minimum_required(VERSION 3.16)
project(cell_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config mjcf urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
