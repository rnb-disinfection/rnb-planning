//
// Created by tamp on 20. 12. 27..
//

#ifndef MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H
#define MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H

#include "typedef.h"
#include <ompl-1.5/ompl/base/Constraint.h>


namespace RNB {
    namespace MoveitCompact {
        namespace ob = ompl::base;
        /**
         * @brief custom ompl planning constraint, currently simple z-plane constraint is implemented
         * @author Junsu Kang
         */
        class CustomConstraint : public ob::Constraint
        {
        public:
            int dims;
            robot_state::RobotStatePtr kinematic_state;
            robot_state::JointModelGroup* joint_model_group;
            double plane_height;
            std::string tool_link;

            CustomConstraint(robot_model::RobotModelPtr robot_model_, std::string group_name, JointState init_state,
                             std::string tool_link, int dims) : ob::Constraint(dims, 1)
            {
                this->dims = dims;
                kinematic_state = std::make_shared<robot_state::RobotState>(robot_model_);
                joint_model_group = robot_model_->getJointModelGroup(group_name);
                this->tool_link = tool_link;
                kinematic_state->setToDefaultValues();

                this->plane_height = 0;
                Eigen::VectorXd height_vec(1);
                function(init_state, height_vec);
                plane_height = height_vec[0];
            }

            void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
            {
                kinematic_state->setJointGroupPositions(joint_model_group, x.data());
                const Eigen::Affine3d &end_effector_tf = kinematic_state->getGlobalLinkTransform(tool_link);
                out[0] = end_effector_tf.translation().z() - plane_height;
            }

            void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
            {
                Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
                Eigen::MatrixXd jacobian;
                kinematic_state->setJointGroupPositions(joint_model_group, x.data());
                kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(tool_link),
                                             reference_point_position,
                                             jacobian);
                out << jacobian.block(2,0, 1, dims);
            }
        };

        typedef std::shared_ptr<CustomConstraint> CustomConstraintPtr;
    }
}

#endif //MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H
