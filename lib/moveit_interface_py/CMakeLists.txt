cmake_minimum_required(VERSION 3.5)
project(moveit_interface_py)

set(CMAKE_CXX_STANDARD 14)

find_package(ompl 1.5.0 EXACT REQUIRED)
SET(OMPL_LIBRARIES /usr/local/lib/libompl.so)

MESSAGE("ompl version - ${OMPL_VERSION}")
MESSAGE("ompl include dir - ${OMPL_INCLUDE_DIRS}")
MESSAGE("ompl libs - ${OMPL_LIBRARIES}")
find_package(catkin REQUIRED
        COMPONENTS
        interactive_markers
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        chomp_motion_planner
        moveit_chomp_optimizer_adapter
        pluginlib
        moveit_visual_tools
        geometric_shapes
        pcl_ros
        pcl_conversions
        rosbag
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        stomp_core
        stomp_moveit
        )

MESSAGE("=====================================================================================================================================================================")
MESSAGE("catkin libs - ${catkin_LIBRARIES}")
MESSAGE("=====================================================================================================================================================================")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread python)

MESSAGE("boost libs - ${Boost_LIBRARIES}")

include_directories(include)
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})


set(CMAKE_SHARED_LIBRARY_PREFIX "") # output file name should match with BOOST_PYTHON_MODULE name

############################################################################
##############################  BUILD LIBRARY ##############################
add_library(${PROJECT_NAME} SHARED
        src/moveit_compact.cpp
        src/boost_python_interface.cpp
        src/ompl_interface/ompl_interface.cpp
        src/ompl_interface/planning_context_manager.cpp
        src/ompl_interface/model_based_planning_context.cpp
        src/ompl_interface/parameterization/model_based_state_space.cpp
        src/ompl_interface/parameterization/model_based_state_space_factory.cpp
        src/ompl_interface/parameterization/joint_space/joint_model_state_space.cpp
        src/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.cpp
        src/ompl_interface/parameterization/work_space/pose_model_state_space.cpp
        src/ompl_interface/parameterization/work_space/pose_model_state_space_factory.cpp
        src/ompl_interface/detail/threadsafe_state_storage.cpp
        src/ompl_interface/detail/state_validity_checker.cpp
        src/ompl_interface/detail/projection_evaluators.cpp
        src/ompl_interface/detail/goal_union.cpp
        src/ompl_interface/detail/constraints_library.cpp
        src/ompl_interface/detail/constrained_sampler.cpp
        src/ompl_interface/detail/constrained_valid_state_sampler.cpp
        src/ompl_interface/detail/constrained_goal_sampler.cpp
        src/logger.cpp
        src/typedef.cpp
        src/ompl_interface/planner/KPIECE_CUSTOM.cpp)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} yaml-cpp)
##############################  BUILD LIBRARY ##############################
############################################################################

############################################################################
#############################  TEST EXECUTION ##############################
add_executable(${PROJECT_NAME}-test
        src/test_mpc.cpp)
target_link_libraries(${PROJECT_NAME}-test
        ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${STOMP_PLUGIN_LIBRARIES} yaml-cpp)
#############################  TEST EXECUTION ##############################
############################################################################
