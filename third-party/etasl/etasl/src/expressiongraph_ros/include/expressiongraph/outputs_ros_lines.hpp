#ifndef EXPRESSIONGRAPH_OUTPUTS_ROS_LINES_HPP
#define EXPRESSIONGRAPH_OUTPUTS_ROS_LINES_HPP

#include <expressiongraph/context.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
namespace KDL {

/**
 * create an OutputGenerator to output outputs of type "type" to ROS for a robot given by "robotparam"
 * \param [in] type       outputs of this type will be passed to ros (only frames will be passed on)
 * \param [in] base_link  base w.r.t. which the points are expressed.
 * \param [in] next (optional) another OutputGenerator to handle different types of output.
 */
extern OutputGenerator::Ptr create_ros_lines_output( 
    const std::string& type, 
    const std::string& base_link, 
    ros::NodeHandle&   n,
    OutputGenerator::Ptr next
);

/**
 * create an OutputGenerator to output outputs of type "type" to ROS for a robot given by "robotparam"
 * \param [in] type       outputs of this type will be passed to ros (only frames will be passed on)
 * \param [in] base_link  base w.r.t. which the points are expressed.
 */
extern OutputGenerator::Ptr create_ros_lines_output( 
    const std::string& type,
    const std::string& base_link,
    ros::NodeHandle&   n
);




};//namespace
#endif
