cmake_minimum_required(VERSION 2.8.3)
project(vision)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS roslint)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
   scripts/image_stitcher
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
