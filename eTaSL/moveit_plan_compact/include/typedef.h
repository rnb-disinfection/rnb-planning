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
    }
}
#endif //MOVEIT_PLAN_COMPACT_TYPEDEF_H
