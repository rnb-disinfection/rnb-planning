cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_bspline)

catkin_add_env_hooks( 25.expressiongraph_bspline SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

find_package(catkin REQUIRED roscpp rospy expressiongraph expressiongraph_lua)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system )
find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



# BUILDING AND LINKING LIBRARY
#FILE( GLOB SRCS src/[^.]*.cpp src/[^.]*.cxx)
set(SRCS src/bspline.cpp src/bspline_scripting.cpp src/init.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Eigen)


add_library(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(bspline_test src/bspline_test.cpp)
target_link_libraries(bspline_test ${catkin_LIBRARIES} ${PROJECT_NAME})





# INSTALLING LIBRARY AND HEADER-FILES
install(TARGETS ${PROJECT_NAME} ${EXAMPLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
install(DIRECTORY scripts/lib/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/lib
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include
  PATTERN ".svn" EXCLUDE)


install(FILES readme.md
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


