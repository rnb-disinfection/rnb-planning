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
add_executable(${PROJECT_NAME} src/moveit_compact.cpp include/moveit_compact.h include/ros_load_yaml.h include/logger.h include/ompl_planner_manager_custom.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} yaml-cpp)
