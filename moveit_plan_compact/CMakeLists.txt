cmake_minimum_required(VERSION 3.17)
project(moveit_plan_compact)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
        COMPONENTS
        interactive_markers
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
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

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(${catkin_INCLUDE_DIRS})

add_library(moveit_plan_compact SHARED src/library.cpp)
#add_executable(moveit_plan_compact src/library.cpp include/library.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})