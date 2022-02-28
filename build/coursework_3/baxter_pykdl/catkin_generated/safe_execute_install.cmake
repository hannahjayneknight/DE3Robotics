execute_process(COMMAND "/home/de3robotics/Desktop/DE3Robotics/build/coursework_3/baxter_pykdl/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/de3robotics/Desktop/DE3Robotics/build/coursework_3/baxter_pykdl/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
