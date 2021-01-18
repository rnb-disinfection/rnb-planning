cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_ros)

find_package(catkin REQUIRED COMPONENTS rosconsole expressiongraph_context roscpp sensor_msgs visualization_msgs urdf)

include_directories( include ${catkin_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} )




add_library(${PROJECT_NAME}  src/outputs_ros.cpp 
                             src/outputs_ros_lines.cpp 
                             src/urdfexpressions.cpp
                             src/fileutils.cpp
                             )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


# currently disabled because it does not belong in this directory 
# and would create an unwanted dependency on expressiongraph_solver_qpoases
#ADD_SUBDIRECTORY(examples)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)
   
install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
   
install(DIRECTORY examples/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
  PATTERN ".svn" EXCLUDE)



