cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_spline)

find_package(catkin REQUIRED COMPONENTS expressiongraph_context_lua)
find_package(Boost REQUIRED COMPONENTS system)

include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/cubicspline.cpp src/init.cpp src/scripting.cpp src/splines.cpp src/motionmodel.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES}
  CATKIN_DEPENDS expressiongraph_context_lua
)
INCLUDE(${PROJECT_SOURCE_DIR}/examples/catkin.cmake)


install(TARGETS ${PROJECT_NAME}
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
   
install(DIRECTORY vcg/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/vcg
  PATTERN ".svn" EXCLUDE)

