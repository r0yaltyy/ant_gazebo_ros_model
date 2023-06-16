cmake_minimum_required(VERSION 3.0.4)

project(ant_model_control)

find_package(catkin REQUIRED 
 rospy
 ant_model_description)

find_package(OpenCV REQUIRED) 
 
catkin_python_setup() 
 
catkin_package()

catkin_install_python(PROGRAMS scripts/ant_motion_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

