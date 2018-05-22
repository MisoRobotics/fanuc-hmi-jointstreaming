cmake_minimum_required(VERSION 2.8.3)
project(fanuc_driver)

find_package(catkin REQUIRED
  control_msgs
  roslint
  rospy
  trajectory_msgs)
catkin_package(CATKIN_DEPENDS
  control_msgs
  rospy
  trajectory_msgs)
catkin_python_setup()
roslint_python()
