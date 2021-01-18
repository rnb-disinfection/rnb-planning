cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_lua)
catkin_add_env_hooks( 20.ilua SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
find_package(catkin REQUIRED COMPONENTS expressiongraph luabind)

include_directories( include ${catkin_INCLUDE_DIRS})

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#PKG_CHECK_MODULES(LUA REQUIRED lua5.1)

LINK_DIRECTORIES(${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} 
  LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES} rlcompleter_c luabind
  CATKIN_DEPENDS expressiongraph luabind
)


add_library(rlcompleter_c src/rlcompleter.c)
target_link_libraries(rlcompleter_c dl readline_c readline history readline history ${catkin_LIBRARY_DIRS})

add_library(readline_c src/readline.c)
target_link_libraries(readline_c dl readline history readline history)

add_library(${PROJECT_NAME} 
    src/bind_kdl.cpp 
    src/init.cpp 
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} rlcompleter_c readline_c
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)
   
install(DIRECTORY scripts/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)
 
install(PROGRAMS scripts/ilua.lua
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
 
