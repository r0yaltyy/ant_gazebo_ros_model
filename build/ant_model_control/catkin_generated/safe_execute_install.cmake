execute_process(COMMAND "/home/alex/ant_ws/build/ant_model_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/alex/ant_ws/build/ant_model_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()