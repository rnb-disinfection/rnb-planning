cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_context)

find_package(catkin  REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS expressiongraph)
include_directories( include ${catkin_INCLUDE_DIRS})

# external libraries:
SET(qpOASES qpOASES-3.0beta)

add_definitions( -D__NO_COPYRIGHT__ )



add_library(${PROJECT_NAME}  src/variables.cpp 
                             src/constraints.cpp
                             src/controller.cpp
                             src/groupingobserver.cpp
                             src/monitors.cpp    
                             src/outputs.cpp 
                             src/outputs_matlab.cpp 
                             src/outputs_csv.cpp 
                             src/context.cpp      
                             src/fileutils.cpp
                             src/defaultobserver.cpp
                             src/controller.cpp
                             )

target_link_libraries(${PROJECT_NAME} expressiongraph ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} expressiongraph
  CATKIN_DEPENDS expressiongraph
  DEPENDS expressiongraph)




# BUILDING AND LINKING TESTS
 
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)
   
  
install(DIRECTORY examples/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
  PATTERN ".svn" EXCLUDE)

