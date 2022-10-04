cmake_minimum_required(VERSION 3.0.2)
project(tool_description)

find_package(catkin REQUIRED)

catkin_package()

# Install files
install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


