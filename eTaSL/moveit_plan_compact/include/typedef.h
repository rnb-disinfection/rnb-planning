//
// Created by tamp on 20. 12. 27..
//

#ifndef MOVEIT_PLAN_COMPACT_TYPEDEF_H
#define MOVEIT_PLAN_COMPACT_TYPEDEF_H

#include <ros/ros.h>
#include <vector>
#include <Eigen/Core>

//ADD = 0,
//REMOVE = 1,
//APPEND = 2,
//MOVE = 3,
namespace RNB {
    namespace MoveitCompact {
        typedef std::vector<std::string> NameList;
        typedef Eigen::Matrix<double, 7, 1> CartPose;
        typedef Eigen::Vector3d Vec3;
        typedef Eigen::VectorXd JointState;
        typedef std::vector<JointState> Trajectory;

        /**
         * @brief Planning result holder
         * @author Junsu Kang
         */
        struct PlanResult {
            Trajectory trajectory;
            bool success;
        };

        enum Shape{
            BOX=shape_msgs::SolidPrimitive::BOX,
            SPHERE=shape_msgs::SolidPrimitive::SPHERE,
            CYLINDER=shape_msgs::SolidPrimitive::CYLINDER,
            PLANE=104u
        };

        struct Geometry{
            Shape type;
            CartPose pose;
            Eigen::Affine3d tf;
            Vec3 dims;
            Geometry(Shape type, CartPose pose, Vec3 dims){
                this->type = type;
                this->pose = pose;
                this->dims = dims;
                Eigen::Quaterniond q(pose[6], pose[3],pose[4],pose[5]);
                this->tf = Eigen::Translation3d(pose.block(0,0,3,1))*q;
            }
        };

        /** \class GeometryList
         */
        typedef std::vector<Geometry> GeometryList;
    }
}
#endif //MOVEIT_PLAN_COMPACT_TYPEDEF_H
