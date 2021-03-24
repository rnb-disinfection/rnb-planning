if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()

cmake_minimum_required(VERSION 2.4.6)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-overloaded-virtual")
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

# external libraries:


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/variables.cpp 
                                     src/constraints.cpp
                                     src/groupingobserver.cpp
                                     src/monitors.cpp    
                                     src/outputs.cpp 
                                     src/outputs_matlab.cpp 
                                     src/outputs_csv.cpp 
                                     src/context.cpp      
                                     src/fileutils.cpp
                                     src/defaultobserver.cpp
                                     )
target_link_libraries(${PROJECT_NAME} expressiongraph)

# unit testing:
rosbuild_add_gtest(tag test/tag.cpp)
target_link_libraries(tag ${PROJECT_NAME})


ADD_SUBDIRECTORY(examples)



