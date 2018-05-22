cmake_minimum_required(VERSION 2.8.3)
project(fanuc_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  uuid_msgs
  )

add_message_files(FILES
  FanucStatus.msg
)

add_service_files(FILES
  SetJointSetpoint.srv
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  uuid_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  uuid_msgs
  )
