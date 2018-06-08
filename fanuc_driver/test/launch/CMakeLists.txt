cmake_minimum_required(VERSION 2.8.3)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    roslaunch
  )
  set(LAUNCH_FILES
    launch-driver-stack.test
  )
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE} USE_TEST_DEPENDENCIES)
  endforeach()
endif()
