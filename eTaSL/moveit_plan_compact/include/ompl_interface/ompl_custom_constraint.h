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
#define USE_ANALYTIC_JACOBIAN

namespace RNB {
    namespace MoveitCompact {
        const double DEFAULT_RADIUS = 1e-3;

        OMPL_CLASS_FORWARD(UnionManifold);

        /** \class UnionManifold
         * @brief Manifold constraint, as a union of geometries. The manifold should be differentiable.
         *        Union manifold is approximated as \f$ f_{union} = \prod^N{f_i}-r^N \f$
         *        and \f$ \nabla f_{union} = \sum^N ((\nabla f_j/f_j)  \prod^N{f_i}) \f$,
         *        where \f$ f_i = 0 \f$ defines the surface of \a i th geometry.
         * @author Junsu Kang
         */
        class UnionManifold : public ompl::base::Constraint {
        public:
            int dims; /*!< State dimension (copy of  Constraint::getAmbientDimension) */
            int num_const; /*!< Constraint dimension (copy of  Constraint::getCoDimension) */
            double radius; /*!< Surface interpolation radius */
            bool fix_surface; /*!< Indicates if surface contact is fixed or not */
            bool fix_normal; /*!< Indicates if normal direction is fixed or not */
            robot_state::RobotStatePtr kinematic_state;
            robot_state::JointModelGroup *joint_model_group;
            std::string tool_link;
            CartPose offset;
            Eigen::Affine3d end_affine;
            GeometryList geometry_list; /*!< Manifold geometry list*/
            Eigen::Vector3d zvec = Eigen::Vector3d(0,0,1);
            Eigen::Matrix4d I_star;

            /**
             * @brief Generate UnionManifold from GeometryList.
             * @author Junsu Kang
             * @param radius interpolation radius.
             * @param tol tolerance.
             */
            UnionManifold(robot_model::RobotModelPtr robot_model_,
                          std::string group_name, std::string tool_link, CartPose offset,
                          GeometryList &geometry_list, bool fix_surface, bool fix_normal, double radius = DEFAULT_RADIUS,
                          double tol = ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE):
                    ompl::base::Constraint(robot_model_->getJointModelGroup(group_name)->getVariableCount(),
                                           fix_surface? (fix_normal? 2: 1) : (fix_normal? 1: 0), tol)
            {
                this->num_const = getCoDimension();
                this->dims = getAmbientDimension();
                this->radius = radius;
                this->fix_normal = fix_normal;
                this->fix_surface = fix_surface;
                kinematic_state = std::make_shared<robot_state::RobotState>(robot_model_);
                kinematic_state->setToDefaultValues();

                joint_model_group = robot_model_->getJointModelGroup(group_name);
                this->tool_link = tool_link;
                this->offset = offset;
                this->end_affine = getAffine3d(offset);

                this->geometry_list.clear();
                this->geometry_list.assign(geometry_list.begin(), geometry_list.end());

                I_star <<   1,0,0,0,
                        0,-1,0,0,
                        0,0,-1,0,
                        0,0,0,-1;
            }

            /**
             * @brief Calculate surface value \f$ f(X) \f$ for a geometry.
             * @author Junsu Kang
             * @param geo geometry.
             * @param geo_tool_tf tool pose from geometry coordinate.
             */
            double calc_surface_val(const Geometry& geo, Eigen::Affine3d& geo_tool_tf) const
            {
                double val;
                switch(geo.type){
                    case Shape::SPHERE:
                        val = geo_tool_tf.translation().norm() - geo.dims[0];
                        break;
                    case Shape::CYLINDER:
                        val = geo_tool_tf.translation().block(0, 0, 2, 1).norm() - geo.dims[0];
                        break;
                    case Shape::PLANE:
                        val = geo_tool_tf.translation().z();
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

            /**
             * @brief Calculate surface jacobian \f$ V_n = \nabla f(X) \f$ for a geometry.
             * @author Junsu Kang
             * @param geo geometry.
             * @param geo_tool_tf tool pose from geometry coordinate.
             */
            Eigen::Vector3d calc_surface_normal(const Geometry& geo, Eigen::Affine3d& geo_tool_tf) const
            {
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
                        vec = zvec;
//                        vec.normalize();
                        break;
                    case Shape::BOX:
                        std::cout << "ERROR: BOX is not supported" << std::endl;
                        throw;
                    default:
                        std::cout << "ERROR: UNDEFINED SHAPE" << std::endl;
                        throw;
                }
                vec = geo.tf.rotation().matrix()*vec;
#ifdef PRINT_CONSTRAINT_VALUES
                std::cout<<"surface_normal: " << vec.transpose()<<std::endl;
#endif
                return vec;
            }

            /**
             * @brief Calculate normal value \f$ f(R) = V_n \cdot V(R) \f$ for a geometry surface.
             * @author Junsu Kang
             * @param vec normal vector V_n in base coordinate.
             * @param tool_vec tool vector V_e in base coordinate.
             * @return angular deviation between normal and tool vector.
             */
            double calc_normal_value(Eigen::Vector3d vec, Eigen::Vector3d tool_vec) const
            {
                double val = 1 - vec.dot(tool_vec);
#ifdef PRINT_CONSTRAINT_VALUES
                std::cout<<"normal_vec: " << vec.transpose() <<std::endl;
                std::cout<<"tool_vec: " << tool_vec.transpose() <<std::endl;
                std::cout<<"normal_value: " << val <<std::endl;
#endif
                return val;
            }

            /**
             * @brief Calculate normal value gradient \f$ \nabla_X f(R) = V_n \cdot V'(R) \cdot J \f$ for a geometry surface.
             * @author Junsu Kang
             * @param vec normal vector V_n in base coordinate.
             * @param end_effector_tf transformation of end effector.
             */
            Eigen::Vector4d calc_normal_gradient(Eigen::Vector3d vec, Eigen::Affine3d end_effector_tf) const
            {
                Eigen::Vector3d vec_tool = end_affine.rotation()*zvec;
                Eigen::Quaterniond quat_vec_tool(0, vec_tool.x(), vec_tool.y(), vec_tool.z());
                Eigen::Vector4d quat_vec_norm(0, vec.x(), vec.y(), vec.z());

                Eigen::Quaterniond quat(end_effector_tf.rotation());
                Eigen::Quaterniond quat_inv = quat.conjugate();

                Eigen::Vector4d normal_grad = - quat_vec_norm.transpose()*(getQmat(quat*quat_vec_tool)*I_star+getQhat(quat_vec_tool*quat_inv));

#ifdef PRINT_CONSTRAINT_VALUES
                std::cout << "normal gradient: " << normal_grad.transpose() << std::endl;
#endif
                return normal_grad;
            }

            /**
             * @brief Calculate union value and jacobian by \f$ f_{union} = \prod^N{f_i}-r^N \f$
             *        and \f$ \nabla f_{union} = \sum^N ((\nabla f_j/f_j)  \prod^N{f_i}) \f$,
             * @author Junsu Kang
             * @param x joint state.
             * @param out value output.
             * @param jac jacobian output.
             * @param calc_jac flag for calculating jacobian.
             */
            void value_(const Eigen::Ref<const Eigen::VectorXd> &x,
                        Eigen::VectorXd &out, Eigen::MatrixXd &jac, bool calc_jac) const
            {
                kinematic_state->setJointGroupPositions(joint_model_group, x.data());
                const Eigen::Affine3d &end_effector_tf = kinematic_state->getGlobalLinkTransform(tool_link);
                Eigen::MatrixXd jac_robot;
                if(calc_jac)
                {
                    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(tool_link),
                                                 end_affine.translation(), jac_robot,true);
#ifdef PRINT_CONSTRAINT_VALUES
                    std::cout<<"jac_robot"<<std::endl;
                    std::cout<<jac_robot<<std::endl;
#endif
                }

                const Eigen::Affine3d tool_tf = end_effector_tf*end_affine;
                Eigen::Vector3d tool_vec = (tool_tf.rotation().matrix()*zvec);
                double surf_val = 1;
                double surf_val_prod = 1;
                Eigen::Vector3d surf_normed;
                Eigen::Vector3d surf_normed_accum(0,0,0);
                Eigen::VectorXd surf_jac(dims);
                double normal_val = 1;
                double normal_val_prod = 1;
                Eigen::Vector4d normal_grad;
                Eigen::Vector4d normal_grad_accum(0,0,0, 0);
                Eigen::VectorXd normal_jac(dims);
                for(int idx=0;idx<geometry_list.size();idx++)
                {
                    Geometry geo = geometry_list[idx];
                    Eigen::Affine3d geo_tool_tf = geo.tf.inverse() * tool_tf;
                    if(fix_surface or calc_jac or fix_normal){
                        surf_val = calc_surface_val(geo, geo_tool_tf);
                        surf_val_prod *= abs(surf_val);
                    }
                    if(calc_jac or fix_normal){
                        surf_normed = calc_surface_normal(geo, geo_tool_tf);
                        surf_normed_accum += surf_normed/surf_val;
                    }
                    if(fix_normal) {
                        normal_val = calc_normal_value(surf_normed, tool_vec);
                        normal_val_prod *= abs(normal_val);
                        if(calc_jac){
                            normal_grad = calc_normal_gradient(surf_normed, end_effector_tf);
                            normal_grad_accum += normal_grad;
                        }
                    }
                }
                if(calc_jac)
                {
                    if(fix_surface){
                        surf_normed_accum *= surf_val_prod;
                        surf_jac = surf_normed_accum.transpose()*jac_robot.block(0,0,3,dims);
                        jac.block(0,0,1, dims) << surf_jac.transpose();
                    }
                    if(fix_normal) {
                        normal_grad_accum *= normal_val_prod;
                        normal_jac = normal_grad_accum.transpose()*jac_robot.block(3,0,4,dims);
                        jac.block(fix_surface?1:0,0,1, dims) << normal_jac.transpose();
                    }
#ifdef PRINT_CONSTRAINT_VALUES
                    std::cout<<"jac"<<std::endl;
                    std::cout<<jac<<std::endl;
#endif
                }
                else{
                    if(fix_surface){
                        out[0] = surf_val;
                    }
                    if(fix_normal) {
                        out[fix_surface?1:0] = normal_val;
                    }
                }
            }

            void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
            {
                Eigen::VectorXd val(num_const);
                Eigen::MatrixXd jac(num_const, dims);
                value_(x, val, jac, false);
                out << val;
            }

#ifdef USE_ANALYTIC_JACOBIAN
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
#endif
        };

        typedef std::vector<UnionManifold> UnionManifoldList;
    }
}

#endif //MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H
