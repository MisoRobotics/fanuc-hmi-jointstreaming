cmake_minimum_required(VERSION 2.8.3)
project(fanuc_driver)

find_package(catkin REQUIRED roslint)
catkin_package()
catkin_python_setup()
roslint_python()
