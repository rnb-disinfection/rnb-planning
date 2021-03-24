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
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

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
rosbuild_add_library(${PROJECT_NAME} src/cubicspline.cpp)
rosbuild_add_library(${PROJECT_NAME} src/init.cpp)
rosbuild_add_library(${PROJECT_NAME} src/scripting.cpp)
rosbuild_add_library(${PROJECT_NAME} src/splines.cpp)
rosbuild_add_library(${PROJECT_NAME} src/motionmodel.cpp)
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
PKG_CHECK_MODULES(LUA REQUIRED lua5.1)
PKG_CHECK_MODULES(LUABIND REQUIRED luabind)
INCLUDE_DIRECTORIES(${LUA_INCLUDE_DIRS} ${LUABIND_INCLUDE_DIRS})
LINK_DIRECTORIES(${LUA_LIBRARY_DIRS} ${LUABIND_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} ${LUA_LIBRARIES} ${LUABIND_LIBRARIES})

OPTION(ENABLE_EXAMPLES "enable compilation of a series of examples" ON) 
INCLUDE(${PROJECT_SOURCE_DIR}/examples/CMakeLists.txt)

