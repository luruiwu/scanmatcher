cmake_minimum_required(VERSION 2.8.3)
project(scanmatcher)
find_package(catkin REQUIRED COMPONENTS
  rospy
  pymrpt
)
catkin_python_setup()
catkin_package(
  CATKIN_DEPENDS rospy pymrpt
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
install(PROGRAMS
  scripts/scanmatcher_odom_node
  scripts/scanmatcher_localizer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)