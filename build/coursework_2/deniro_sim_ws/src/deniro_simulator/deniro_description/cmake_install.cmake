# Install script for directory: /home/de3robotics/Desktop/DE3Robotics/src/coursework_2/deniro_sim_ws/src/deniro_simulator/deniro_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/de3robotics/Desktop/DE3Robotics/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/de3robotics/Desktop/DE3Robotics/build/coursework_2/deniro_sim_ws/src/deniro_simulator/deniro_description/catkin_generated/installspace/deniro_description.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deniro_description/cmake" TYPE FILE FILES
    "/home/de3robotics/Desktop/DE3Robotics/build/coursework_2/deniro_sim_ws/src/deniro_simulator/deniro_description/catkin_generated/installspace/deniro_descriptionConfig.cmake"
    "/home/de3robotics/Desktop/DE3Robotics/build/coursework_2/deniro_sim_ws/src/deniro_simulator/deniro_description/catkin_generated/installspace/deniro_descriptionConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deniro_description" TYPE FILE FILES "/home/de3robotics/Desktop/DE3Robotics/src/coursework_2/deniro_sim_ws/src/deniro_simulator/deniro_description/package.xml")
endif()

