//
// Created by tamp on 20. 12. 27..
//

#ifndef MOVEIT_PLAN_COMPACT_TYPEDEF_H
#define MOVEIT_PLAN_COMPACT_TYPEDEF_H

#include <ros/ros.h>
#include <geometric_shapes/shape_messages.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

//ADD = 0,
//REMOVE = 1,
//APPEND = 2,
//MOVE = 3,

/**
 * @namespace RNB
 * @brief classes and functions developed by RNB
 * @author Junsu Kang
 */
namespace RNB {
    /**
     * @namespace RNB::MoveitCompact
     * @brief classes and functions designed for easy use of Moveit
     * @author Junsu Kang
     */
    namespace MoveitCompact {
        typedef std::vector<std::string> NameList;

        typedef Eigen::Vector3d Vec3;
        typedef Eigen::VectorXd JointState;
        typedef std::vector<JointState> Trajectory;

        typedef Eigen::Matrix<long double,-1, 1> VectorXld;
        typedef Eigen::Matrix<long double,-1, -1> MatrixXld;

        /** \class RNB::MoveitCompact::CartPose
         */
        typedef Eigen::Matrix<double, 7, 1> CartPose;

        /**
         * @brief Extract quaternion from RNB::MoveitCompact::CartPose
         * @author Junsu Kang
         */
        Eigen::Quaterniond getQuaternion(CartPose pose);

        /**
         * @brief Get Eigen::Affine3d from RNB::MoveitCompact::CartPose
         * @author Junsu Kang
         */
        Eigen::Affine3d getAffine3d(CartPose pose);

        Eigen::MatrixXd getQmat(Eigen::Quaterniond a);
        Eigen::MatrixXd getQhat(Eigen::Quaterniond a);

        Eigen::Matrix3d getHat(Eigen::Vector3d vec);

        /**
         * @brief Planning result holder
         * @author Junsu Kang
         */
        struct PlanResult {
            Trajectory trajectory;
            bool success;
        };

        enum ObjectType{
            BOX=shape_msgs::SolidPrimitive::BOX,
            SPHERE=shape_msgs::SolidPrimitive::SPHERE,
            CYLINDER=shape_msgs::SolidPrimitive::CYLINDER,
            PLANE=104u
        };

        struct Geometry{
            ObjectType type;
            CartPose pose;
            Eigen::Affine3d tf;
            Vec3 dims;
            Geometry(ObjectType type, CartPose pose, Vec3 dims);
        };

        /** \class RNB::MoveitCompact::GeometryList
         */
        typedef std::vector<Geometry> GeometryList;
    }
}
#endif //MOVEIT_PLAN_COMPACT_TYPEDEF_H
