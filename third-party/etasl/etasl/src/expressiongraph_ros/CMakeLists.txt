# USE THE CATKIN CMAKELISTS FOR BUILDING WITH CATKIN
# ELSE PROCEED WITH ROSBUILD BELOW
if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()


cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/outputs_ros.cpp 
                                     src/outputs_ros_lines.cpp 
                                     src/urdfexpressions.cpp
                                     src/urdfexpressions2.cpp
                                     src/urdfexpressions3.cpp
                                     src/fileutils.cpp
                                     )
target_link_libraries(${PROJECT_NAME} expressiongraph)

ADD_SUBDIRECTORY(examples)

