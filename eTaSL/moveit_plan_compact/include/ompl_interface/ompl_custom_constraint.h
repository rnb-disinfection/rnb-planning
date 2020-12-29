//
// Created by tamp on 20. 12. 27..
//

#ifndef MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H
#define MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H

#include "typedef.h"
#include <ompl/base/Constraint.h>
#include <fcl/distance.h>
#include "logger.h"

//#define PRINT_CONSTRAINT_VALUES

namespace RNB {
    namespace MoveitCompact {

        OMPL_CLASS_FORWARD(UnionManifold);

        /**
         * @brief manifold constraint, as a union of geometries
         * @author Junsu Kang
         */
        class UnionManifold : public ompl::base::Constraint {
        public:
            int dims;
            int num_const;
            robot_state::RobotStatePtr kinematic_state;
            robot_state::JointModelGroup *joint_model_group;
            std::string tool_link;
            GeometryList geometry_list;
            Vec3 offset;

            UnionManifold(robot_model::RobotModelPtr robot_model_,
                          std::string group_name, std::string tool_link, Vec3 offset,
                          GeometryList &geometry_list, double tol = ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE):
                    ompl::base::Constraint(robot_model_->getJointModelGroup(group_name)->getVariableCount(), 1,tol) {
                this->num_const = getCoDimension();
                this->dims = getAmbientDimension();
                kinematic_state = std::make_shared<robot_state::RobotState>(robot_model_);
                kinematic_state->setToDefaultValues();

                joint_model_group = robot_model_->getJointModelGroup(group_name);
                this->tool_link = tool_link;
                this->offset = offset;

                this->geometry_list.clear();
                this->geometry_list.assign(geometry_list.begin(), geometry_list.end());
            }

            double calc_surface_val(const Geometry& geo, Eigen::Affine3d& geo_tool_tf) const {
                double val;
                switch(geo.type){
                    case Shape::SPHERE:
                        val = abs(geo_tool_tf.translation().norm() - geo.dims[0]);
                        break;
                    case Shape::CYLINDER:
                        val = abs(geo_tool_tf.translation().block(0, 0, 2, 1).norm() - geo.dims[0]);
                        break;
                    case Shape::PLANE:
                        val = abs(geo_tool_tf.translation().z());
                        break;
                    case Shape::BOX:
                        std::cout << "ERROR: BOX is not supported" << std::endl;
                        throw;
                    default:
                        std::cout << "ERROR: UNDEFINED SHAPE" << std::endl;
                        throw;
                }
#ifdef PRINT_CONSTRAINT_VALUES
                std::cout << "surface value: " << val << std::endl;
#endif
                return val;
            }

            Eigen::VectorXd calc_surface_jac(const Geometry& geo, Eigen::Affine3d& geo_tool_tf, const Eigen::MatrixXd& jac_robot) const {
                Eigen::Vector3d vec;
                switch(geo.type){
                    case Shape::SPHERE:
                        vec = geo_tool_tf.translation().matrix();
                        vec.normalize();
                        break;
                    case Shape::CYLINDER:
                        vec = Eigen::Vector3d(geo_tool_tf.translation().x(), geo_tool_tf.translation().y(), 0);
                        vec.normalize();
                        break;
                    case Shape::PLANE:
                        vec = geo.tf.rotation().matrix().transpose()*Eigen::Vector3d(0,0,1);
#ifdef PRINT_CONSTRAINT_VALUES
                        std::cout << "surface jac: " << std::endl << vec.transpose() << std::endl;
#endif
                        vec.normalize();
                        break;
                    case Shape::BOX:
                        std::cout << "ERROR: BOX is not supported" << std::endl;
                        throw;
                    default:
                        std::cout << "ERROR: UNDEFINED SHAPE" << std::endl;
                        throw;
                }
#ifdef PRINT_CONSTRAINT_VALUES
                std::cout<<"vec"<<std::endl;
                std::cout<<vec.transpose()<<std::endl;
                std::cout<<"jac_robot"<<std::endl;
                std::cout<<jac_robot.block(0,0,3,dims)<<std::endl;
#endif
                return vec.transpose()*jac_robot.block(0,0,3,dims);
            }

            void value_(const Eigen::Ref<const Eigen::VectorXd> &x,
                        Eigen::VectorXd &out, Eigen::MatrixXd &jac, bool calc_jac) const{
                kinematic_state->setJointGroupPositions(joint_model_group, x.data());
                const Eigen::Affine3d &end_effector_tf = kinematic_state->getGlobalLinkTransform(tool_link);
                Eigen::MatrixXd jac_robot;
                if(calc_jac)
                {
                    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(tool_link),
                                                 offset, jac_robot,true);
                }

                Eigen::Translation3d end_off(offset);

                const Eigen::Affine3d tool_tf = end_effector_tf*end_off;
                double val_tmp;
                double surf_val = 1e5;
                Eigen::VectorXd surf_jac(dims);
                for(int idx=0;idx<geometry_list.size();idx++){
                    Geometry geo = geometry_list[idx];
                    Eigen::Affine3d geo_tool_tf = geo.tf.inverse() * tool_tf;
                    val_tmp = calc_surface_val(geo, geo_tool_tf);
                    if (val_tmp<surf_val){
                        surf_val = val_tmp;
                        if(calc_jac){
                            surf_jac = calc_surface_jac(geo, geo_tool_tf, jac_robot);
                        }
//                        std::cout<<TEXT_RED("WARN: Selection rule not implemented")<<std::endl;
                    }
                }
                if(calc_jac){
                    jac.block(0,0,1, dims) << surf_jac.transpose();
#ifdef PRINT_CONSTRAINT_VALUES
                    std::cout<<"surf_jac"<<std::endl;
                    std::cout<<surf_jac.transpose()<<std::endl;
                    std::cout<<"jac"<<std::endl;
                    std::cout<<jac<<std::endl;
#endif
                }
                else{
                    out[0] = surf_val;
                }
            }

            void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
            {
                Eigen::VectorXd val(num_const);
                Eigen::MatrixXd jac(num_const, dims);
                value_(x, val, jac, false);
                out << val;
            }

            void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
            {
                Eigen::VectorXd val(num_const);
                Eigen::MatrixXd jac(num_const, dims);
                value_(x, val, jac, true);
                out << jac;
#ifdef PRINT_CONSTRAINT_VALUES
                std::cout<<"jacout"<<std::endl;
                std::cout<<out<<std::endl;
#endif
            }
        };

        typedef std::vector<UnionManifold> UnionManifoldList;
    }
}

#endif //MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H
