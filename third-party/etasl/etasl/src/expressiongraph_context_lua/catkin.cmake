cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_context_lua)
catkin_add_env_hooks( 20.context_lua SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
find_package(catkin REQUIRED COMPONENTS expressiongraph_lua expressiongraph_ros expressiongraph_context roslib)

include_directories( include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES}
  CATKIN_DEPENDS expressiongraph_lua expressiongraph_ros expressiongraph_context roslib
)


add_library(${PROJECT_NAME} src/scripting.cpp      
                                     src/init.cpp 
                                     src/breakobserver.cpp
                                     src/rfsm_observer.cpp
                                     src/luabind_util.cpp
                                     )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


# Disabled because they introduce a dependency to expressiongraph_solver_qpoases
# should be in a seperate package.
#
#add_executable(script_solver src/script_solver.cpp)
#target_link_libraries(script_solver ${PROJECT_NAME} ${catkin_LIBRARIES})
#
#add_executable(script_solver_grouping src/script_solver_grouping.cpp)
#target_link_libraries(script_solver_grouping ${PROJECT_NAME} ${catkin_LIBRARIES})
#
#add_executable(script_solver_rfsm src/script_solver_rfsm.cpp)
#target_link_libraries(script_solver_rfsm ${PROJECT_NAME} ${catkin_LIBRARIES})
#
#add_executable(script_ros src/script_ros.cpp)
#target_link_libraries(script_ros ${PROJECT_NAME} ${catkin_LIBRARIES})
#
#add_executable(script_ros_grouping src/script_ros_grouping.cpp)
#target_link_libraries(script_ros_grouping ${PROJECT_NAME} ${catkin_LIBRARIES})
#
#add_executable(script_ros_rfsm src/script_ros_rfsm.cpp)
#target_link_libraries(script_ros_rfsm ${PROJECT_NAME} ${catkin_LIBRARIES})
#


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

