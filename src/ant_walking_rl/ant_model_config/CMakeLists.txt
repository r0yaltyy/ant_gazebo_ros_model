cmake_minimum_required(VERSION 3.0.4)

project(ant_model_config)

find_package(catkin 
  REQUIRED
  ant_model_description
  ant_model_control)

catkin_package()

find_package(roslaunch)

