#ifndef EXPRESSIONGRAPH_OUTPUTS_ROS_HPP
#define EXPRESSIONGRAPH_OUTPUTS_ROS_HPP
#include <ros/ros.h>
#include <expressiongraph/context.hpp>
#include <iostream>
#include <string>

namespace KDL {

/**
 * create an OutputGenerator to output outputs of type "type" to ROS for a robot given by "robotparam"
 * \param [in] robotparam contains urdf description of te robot.
 * \param [in] type       outputs of this type will be passed to ros (only frames will be passed on)
 * \param [in] next (optional) another OutputGenerator to handle different types of output.
 */
extern OutputGenerator::Ptr create_ros_output( 
    const std::string& robotparam,
    const std::string& type, 
    const std::string& base_link, 
    Context::Ptr ctx,
    ros::NodeHandle&   n,
    OutputGenerator::Ptr next
);

/**
 * create an OutputGenerator to output outputs of type "type" to ROS for a robot given by "robotparam"
 * \param [in] robotparam contains urdf description of te robot.
 * \param [in] type       outputs of this type will be passed to ros (only frames will be passed on)
 */
extern OutputGenerator::Ptr create_ros_output( 
    const std::string& robotparam,
    const std::string& type,
    const std::string& base_link,
    Context::Ptr ctx,
    ros::NodeHandle&   n
);




};//namespace
#endif
