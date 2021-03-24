

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
rosbuild_add_library(${PROJECT_NAME} src/scripting.cpp      
                                     src/luabind_util.cpp
                                     src/init.cpp 
                                     src/breakobserver.cpp
                                     src/rfsm_observer.cpp
                                     )
rosbuild_add_executable(script_solver src/script_solver.cpp)
target_link_libraries(script_solver ${PROJECT_NAME})

rosbuild_add_executable(script_solver_grouping src/script_solver_grouping.cpp)
target_link_libraries(script_solver_grouping ${PROJECT_NAME})

rosbuild_add_executable(script_solver_rfsm src/script_solver_rfsm.cpp)
target_link_libraries(script_solver_rfsm ${PROJECT_NAME})

rosbuild_add_executable(script_ros src/script_ros.cpp)
target_link_libraries(script_ros ${PROJECT_NAME})

rosbuild_add_executable(script_ros_grouping src/script_ros_grouping.cpp)
target_link_libraries(script_ros_grouping ${PROJECT_NAME})

rosbuild_add_executable(script_ros_rfsm src/script_ros_rfsm.cpp)
target_link_libraries(script_ros_rfsm ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} expressiongraph_context_lua luabind)


