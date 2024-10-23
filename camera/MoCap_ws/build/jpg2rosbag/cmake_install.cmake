# Install script for directory: /lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/jpg2rosbag

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/build/jpg2rosbag/catkin_generated/installspace/jpg2rosbag.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jpg2rosbag/cmake" TYPE FILE FILES
    "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/build/jpg2rosbag/catkin_generated/installspace/jpg2rosbagConfig.cmake"
    "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/build/jpg2rosbag/catkin_generated/installspace/jpg2rosbagConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jpg2rosbag" TYPE FILE FILES "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/jpg2rosbag/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/jpg2rosbag" TYPE PROGRAM FILES "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/build/jpg2rosbag/catkin_generated/installspace/jpg2rosbag.py")
endif()

