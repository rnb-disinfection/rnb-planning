cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_collision)

catkin_add_env_hooks( 25.expressiongraph_collision SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

find_package(catkin REQUIRED expressiongraph expressiongraph_lua)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system )
find_package( Bullet)
find_package( Eigen)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)

# BUILDING AND LINKING LIBRARY
#FILE( GLOB SRCS src/[^.]*.cpp src/[^.]*.cxx)
set(SRCS src/bullet_collision_lua.cpp  
         src/convex_object.cpp
         src/init.cpp)


add_library(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BULLET_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${Bullet_LIBRARIES}
  DEPENDS orocos_kdl Eigen)


# POTENTIALLY, BUILDING AND LINKING EXAMPLES
OPTION(ENABLE_EXAMPLES "enable compilation of a series of examples" ON) 
INCLUDE(${PROJECT_SOURCE_DIR}/examples/CMakeLists.txt)

# INSTALLING LIBRARY AND HEADER-FILES
install(TARGETS ${PROJECT_NAME} ${EXAMPLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
install(DIRECTORY scripts/lib/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/lib
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/utilities
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)



install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include
  PATTERN ".svn" EXCLUDE)


install(FILES example.lua readme.md scripts/test/derivatives.lua
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


