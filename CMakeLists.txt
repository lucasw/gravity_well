cmake_minimum_required(VERSION 2.8.3)
project(gravity_well)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  ddynamic_reconfigure
  message_generation
  nodelet
  roscpp
  roslint
  std_msgs
)

roslint_python()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS ddynamic_reconfigure
)

install(PROGRAMS
  scripts/gravity_well.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
