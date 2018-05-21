cmake_minimum_required(VERSION 2.8.3)
project(miso_fanuc)

find_package(catkin REQUIRED
  control_msgs
  rospy
  trajectory_msgs)
catkin_package(CATKIN_DEPENDS
  control_msgs
  rospy
  trajectory_msgs)
catkin_python_setup()

