cmake_minimum_required(VERSION 3.5)
project(moveit_plan_compact)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
        COMPONENTS
        interactive_markers
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_planners_ompl
        moveit_ros_perception
        pluginlib
        geometric_shapes
        pcl_ros
        pcl_conversions
        rosbag
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        )

find_package(ompl REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread python)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_SHARED_LIBRARY_PREFIX "") # output file name should match with BOOST_PYTHON_MODULE name

#add_library(${PROJECT_NAME} SHARED src/moveit_compact.cpp include/moveit_compact.h
#        include/ros_load_yaml.h include/logger.h
#        src/boost_python_interface.cpp include/bp_container_interface.h)
add_executable(${PROJECT_NAME}
        src/moveit_compact.cpp
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
        include/moveit_compact.h
        include/ros_load_yaml.h
        include/logger.h
        include/ompl_custom_constraint.h
        include/typedef.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} yaml-cpp)
