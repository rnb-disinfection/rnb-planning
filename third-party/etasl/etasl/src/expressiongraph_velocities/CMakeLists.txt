cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_velocities)

catkin_add_env_hooks( 27.egvel SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

find_package(catkin REQUIRED expressiongraph expressiongraph_lua)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# BUILDING AND LINKING LIBRARY
#FILE( GLOB SRCS src/[^.]*.cpp src/[^.]*.cxx)
set(SRCS src/init.cpp)


add_library(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${Bullet_LIBRARIES}
  DEPENDS orocos_kdl Eigen)


# POTENTIALLY, BUILDING AND LINKING EXAMPLES
# OPTION(ENABLE_EXAMPLES "enable compilation of a series of examples" ON) 
# INCLUDE(${PROJECT_SOURCE_DIR}/examples/CMakeLists.txt)

# INSTALLING LIBRARY AND HEADER-FILES
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(FILES readme.rst
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)


install(DIRECTORY src/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

