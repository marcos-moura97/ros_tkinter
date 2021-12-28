cmake_minimum_required(VERSION 2.8.3)
project(ros_tkinter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  nodes/ros_tkinter_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
