//
// Created by tamp on 20. 12. 27..
//

#ifndef MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H
#define MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H

#include "typedef.h"
#include "constants.h"
#include <ompl/base/Constraint.h>
#include <fcl/distance.h>
#include "logger.h"


//#define DEBUG_CONSTRAINT_VALUES
#define USE_ANALYTIC_JACOBIAN
#define EXP_F_ALPHA 1000

namespace RNB {
    namespace MoveitCompact {
        OMPL_CLASS_FORWARD(UnionManifold);

        /*! \class UnionManifold
         * @brief Manifold constraint, as a union of geometries. The manifold should be differentiable.
         *        Union manifold is approximated as \f$ f_{union} = \prod^N{f_i}-r^N \f$
         *        and \f$ \nabla f_{union} = \sum^N ((\nabla f_j/f_j)  \prod^N{f_i}) \f$,
         *        where \f$ f_i = 0 \f$ defines the surface of \a i th geometry.
         * @author Junsu Kang
         */
        class UnionManifold : public ompl::base::Constraint {
        public:
            constexpr static const double DEFAULT_TOLERANCE = 1e-3;

            int dims; /*!< State dimension (copy of  Constraint::getAmbientDimension) */
            int num_const; /*!< Constraint dimension (copy of  Constraint::getCoDimension) */
            bool fix_surface; /*!< Indicates if surface contact is fixed or not */
            bool fix_normal; /*!< Indicates if normal direction is fixed or not */
            robot_state::RobotStatePtr kinematic_state;
            robot_state::JointModelGroup *joint_model_group;
            std::string tool_link;
            CartPose offset;
            Eigen::Affine3d end_affine;
            GeometryList geometry_list; /*!< Manifold geometry list*/
            Eigen::Vector3d xvec = Eigen::Vector3d(1,0,0);
            Eigen::Vector3d yvec = Eigen::Vector3d(0,1,0);
            Eigen::Vector3d zvec = Eigen::Vector3d(0,0,1);
            Eigen::Matrix4d I_star;

            /**
             * @brief Generate UnionManifold from GeometryList.
             * @author Junsu Kang
             * @param tol tolerance.
             */
            UnionManifold(robot_model::RobotModelPtr robot_model_,
                          std::string group_name, std::string tool_link, CartPose offset,
                          GeometryList &geometry_list, bool fix_surface, bool fix_normal,
                          double tolerance = DEFAULT_TOLERANCE):
                    ompl::base::Constraint(robot_model_->getJointModelGroup(group_name)->getVariableCount(),
                                           fix_surface? (fix_normal? 2: 1) : (fix_normal? 1: 0), tolerance)
            {
                this->num_const = getCoDimension();
                this->dims = getAmbientDimension();
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
                for (auto gtem = this->geometry_list.begin(); gtem!=this->geometry_list.end(); gtem++ ){
                    std::cout << "geometry(" << gtem->type << "): " << gtem->pose.transpose() << std::endl;
                }

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
                    case ObjectType::SPHERE:
                        val = geo_tool_tf.translation().norm() - geo.dims[0]/2;
                        break;
                    case ObjectType::CYLINDER:
                        val = geo_tool_tf.translation().block(0, 0, 2, 1).norm() - geo.dims[0]/2;
                        break;
                    case ObjectType::PLANE:
                        val = geo_tool_tf.translation().z();
                        break;
                    case ObjectType::BOX:
                        std::cout << TEXT_RED("ERROR: BOX it not tested") << std::endl;
                        throw std::runtime_error("ERROR: BOX it not tested");
                        val = (geo_tool_tf.translation().cwiseAbs()-geo.dims).minCoeff();
                        break;
                    default:
                        std::cout << "ERROR: UNDEFINED SHAPE" << std::endl;
                        throw;
                }
#ifdef PRINT_DEBUG
                std::cout << "surface value: " << val << std::endl;
#endif
                return val;
            }

            /**
             * @brief Calculate derivative of normal vector by
             *          \f$ \frac{d \overline{P} }{dP}
             *          = d (\frac{P}{|P|}) / dP
             *          = \frac{I}{|P|} - \frac{P}{|P|^2}\frac{d|P|}{P}
             *          = \frac{I}{|P|} - \frac{P\overline{P}^T}{|P|^2}
             *          = \frac{I - \overline{P}\overline{P}^T}{|P|} \f$ for a geometry surface.
             * @param vec normalized vector of P
             * @param vec_norm norm of P
             * @author Junsu Kang
             */
            Eigen::MatrixXd calc_normal_der(Eigen::VectorXd vec, double vec_norm) const {
                Eigen::MatrixXd I(vec.size(), vec.size());
                I.setIdentity();
                return (I-vec*vec.transpose())/vec_norm;
            }

            Eigen::VectorXd min_dir_(Eigen::VectorXd vec) const {
                if(vec[0]<vec[1]){ // not 1
                    if(vec[0]<vec[2]) return xvec;  // not 2
                    else return zvec;  // not 0
                }
                else{ // not 0
                    if(vec[1]<vec[2]) return yvec; // not 2
                    else return zvec; // not 1
                }
            }

            /**
             * @brief Calculate surface jacobian \f$ V_n = \nabla f(X) \f$ for a geometry.
             * @author Junsu Kang
             * @param geo geometry.
             * @param geo_tool_tf tool pose from geometry coordinate.
             * @param vec return normal vector
             * @param vec_der return space derivative of normal vector
             * @param flag for calculating jacobian
             */
            void calc_surface_normal(const Geometry& geo, Eigen::Affine3d& geo_tool_tf,
                                                Eigen::Vector3d& vec, Eigen::Matrix3d& vec_der, bool calc_jac=false) const
            {
                double norm;
                vec_der <<  0,0,0,
                            0,0,0,
                            0,0,0;
                switch(geo.type){
                    case ObjectType::SPHERE:
                        vec = geo_tool_tf.translation().matrix();
                        norm = vec.norm();
                        vec /= norm;
                        if (calc_jac){
                            vec_der << calc_normal_der(vec, norm);
                        }
                        break;
                    case ObjectType::CYLINDER:
                        vec = Eigen::Vector3d(geo_tool_tf.translation().x(), geo_tool_tf.translation().y(), 0);
                        norm = vec.norm();
                        vec /= norm;
                        if (calc_jac){
                            vec_der.block(0,0,2,2) << calc_normal_der(vec.block(0,0,2,1), norm);
                        }
                        break;
                    case ObjectType::PLANE:
                        vec = zvec;
                        norm = vec.norm();
                        vec /= norm;
                        break;
                    case ObjectType::BOX:
                        std::cout << TEXT_RED("ERROR: BOX it not tested") << std::endl;
                        throw std::runtime_error("ERROR: BOX it not tested");
                        vec = min_dir_(geo_tool_tf.translation().cwiseAbs()-geo.dims);
                        vec = geo_tool_tf.translation().cwiseProduct(vec);
                        norm = vec.norm();
                        vec /= norm;
                        break;
                    default:
                        std::cout << "ERROR: UNDEFINED SHAPE" << std::endl;
                        throw;
                }
                vec = geo.tf.linear()*vec;
                if (calc_jac){
                    vec_der = geo.tf.linear()*vec_der*geo.tf.linear().transpose();
                }
#ifdef PRINT_DEBUG
                std::cout<<"surface_normal: " << vec.transpose()<<std::endl;
#endif
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
#ifdef PRINT_DEBUG
                std::cout<<"normal_vec: " << vec.transpose() <<std::endl;
                std::cout<<"tool_vec: " << tool_vec.transpose() <<std::endl;
                std::cout<<"normal value: " << val <<std::endl;
#endif
                return val;
            }

            /**
             * @brief Calculate normal value gradient by
             *          \f$ \frac{d(v_1^T R v_2)}{dq_i}
             *          = \frac{d(v_1^T)}{dq_i} R v_2 + v_1 \frac{d(R v_2)}{dq_i}
             *          = (\frac{d(v_1)}{dP}\frac{dP}{dq_i})^T R v_2 + v_1 \cdot (w_i \times R v_2) \f$
             *          \f$ = (\frac{d(v_1)}{dP}\frac{dP}{dq_i})^T R v_2 + w_i \cdot (R v_2 \times v_1) \f$ for a geometry surface.
             * @author Junsu Kang
             * @param vec normal vector V_n in base coordinate.
             * @param vec_der derivative of normal vector V_n in base coordinate.
             * @param tool_tf tool transformation in base coordinate.
             * @param jac_robot jacobian vector for the robot, containts \f$ w_j \f$
             */
            Eigen::VectorXd calc_normal_gradient(Eigen::Vector3d vec, Eigen::Matrix3d vec_der,
                                                 Eigen::Affine3d tool_tf, Eigen::MatrixXd& jac_robot) const
            {
                Eigen::Vector3d vec_tool = tool_tf.linear()*zvec;
                Eigen::MatrixXd v_j(jac_robot.block(0,0,3,dims));
                Eigen::MatrixXd w_j(jac_robot.block(3,0,3,dims));
                Eigen::VectorXd normal_gradient;
                normal_gradient = -((vec_der*v_j).transpose()*vec_tool + w_j.transpose()*(vec_tool.cross(vec)));

#ifdef PRINT_DEBUG
                std::cout << "vec_der: \n" << vec_der << std::endl;
                std::cout << "vec_normal: " << vec.transpose() << std::endl;
                std::cout << "vec_tool: " << vec_tool.transpose() << std::endl;
                std::cout << "normal_gradient: " << normal_gradient.transpose() << std::endl;
#endif
                return normal_gradient;
            }

            /*! \fn void UnionManifold::soft_min_weight_gradient_
             * @brief soft-min style interplation (inverse-square-weighted-mean),
             *          \f$ f_{union} =  \frac{\sum_i e^{-\alpha f_i}p_i}{\sum_i e^{-\alpha f_i}} = P/G \f$,
             *          where \f$ P=\sum_i e^{-\alpha f_i}p_i \f$ and \f$ G=\sum_i e^{-\alpha f_i} \f$.
             *          The jacobian is \f$ \nabla f_{union} = (\nabla P G - P \nabla G)/G^2 \f$, where
             *          \f$ \nabla P = \sum{ \left( - \alpha e^{-\alpha f_i} \nabla f_i p_i + e^{-\alpha f_i} \nabla p_i \right)} \f$
             *          and \f$ \nabla G = - \alpha \sum{ e^{-\alpha f_i} \nabla f_i} \f$
             * @author Junsu Kang
             * @param f weight value vector (\f$ N_g \times 1 \f$).
             * @param df weight value gradients (\f$ N_g \times D \f$).
             * @param p value vector (\f$ N_g \times 1 \f$).
             * @param dp value gradients (\f$ N_g \times D \f$).
             * @param f_union returns f value for union geometry.
             * @param df_union returns f gradient for union geometry.
             * @param calc_jac flag for calculating jacobian.
             */
            void soft_min_weight_gradient_(Eigen::VectorXd &f, Eigen::MatrixXd &df, Eigen::VectorXd &p, Eigen::MatrixXd &dp,
                                    double& p_union, Eigen::VectorXd& dp_union, bool calc_jac=false) const
            {
                // !! - Cast to long double, exp() output gets too large
                double alpha = EXP_F_ALPHA;
                VectorXld  f_ = f.cast<long double>();
                MatrixXld  df_ = df.cast<long double>();
                VectorXld  p_ = p.cast<long double>();
                MatrixXld  dp_ = dp.cast<long double>();
                VectorXld  G = Eigen::exp(-alpha*f_.array()); // exp(-af)
                VectorXld P = G.cwiseProduct(p_); // exp(-af)*p
                long double sumP = P.sum();
                long double sumG = G.sum();
                p_union = sumP/sumG;
                if(calc_jac){
                    VectorXld dG = (- alpha * G.transpose()*df_).transpose();
                    VectorXld dP = (- alpha * G.cwiseProduct(p_).transpose()*df_ + G.transpose()*dp_).transpose();
                    dp_union = ((dP*sumG - sumP*dG)/pow(sumG,2)).cast<double>();
                }
#ifdef PRINT_DEBUG
                std::cout<< "f_: " << f_.transpose() <<std::endl;
                std::cout<< "G: " << G.transpose() <<std::endl;
#endif
            }

            /**
             * @brief Calculate union surface and normal values by UnionManifold::soft_min_weight_gradient_.
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
                const Eigen::Affine3d &base_tf = kinematic_state->getGlobalLinkTransform(joint_model_group->getJointModels()[0]->getParentLinkModel());
                const Eigen::Affine3d tool_tf = end_effector_tf*end_affine;

#ifdef PRINT_DEBUG
                std::cout<<"x: " << x.transpose() << std::endl;
                std::cout<<"end_effector_tf: \n" << end_effector_tf.matrix() << std::endl;
#endif

                Eigen::MatrixXd jac_robot(6, dims);
                if(calc_jac)
                {
                    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(tool_link),
                                                 end_affine.translation(), jac_robot,false);
                    /* [WARNING] getJacobian returns jacobian in the coordinate of joint group's root link.
                     * below is multipling root transformation to it! */
                    jac_robot.block(0,0,3,dims) << base_tf.linear()*jac_robot.block(0,0,3,dims);
                    jac_robot.block(3,0,3,dims) << base_tf.linear()*jac_robot.block(3,0,3,dims);
#ifdef PRINT_DEBUG
                    std::cout<<"root_joint: " <<joint_model_group->getJointModels()[0]->getName() << std::endl;
                    std::cout<<"root_link: " <<joint_model_group->getJointModels()[0]->getParentLinkModel()->getName() << std::endl;
                    std::cout<<"jac_robot"<<std::endl;
                    std::cout<<jac_robot<<std::endl;
#endif
                }

                // tool-related
                Eigen::Vector3d tool_vec = (tool_tf.linear()*zvec);

                // surface-related
                Eigen:: VectorXd surf_f(geometry_list.size());
                Eigen:: MatrixXd surf_df(geometry_list.size(), 3);
                double surf_val = 1;
                Eigen::Vector3d surf_normal; /* = surface gradient */
                Eigen::Matrix3d surf_normal_derivative;

                // normal-related
                double normal_val = 1;
                Eigen::VectorXd normal_f(geometry_list.size());
                Eigen::VectorXd normal_gradient(dims);
                Eigen::MatrixXd normal_df(geometry_list.size(), dims);

//                double surf_val_prod = 1;
//                Eigen::Vector3d surf_normed_accum(0,0,0);
//                Eigen::VectorXd surf_jac(dims);
//                double normal_val_prod = 1;
//                Eigen::VectorXd normal_grad;
//                Eigen::VectorXd normal_grad_accum=Eigen::VectorXd::Zero(dims);
//                Eigen::VectorXd normal_jac(dims);
                for(int idx=0;idx<geometry_list.size();idx++)
                {
                    Geometry geo = geometry_list[idx];
                    Eigen::Affine3d geo_tool_tf = geo.tf.inverse() * tool_tf;
                    if(fix_surface || calc_jac || fix_normal){
                        surf_val = calc_surface_val(geo, geo_tool_tf);
                        surf_f[idx] = surf_val;
                    }
                    if(calc_jac || fix_normal){
                        calc_surface_normal(geo, geo_tool_tf, surf_normal, surf_normal_derivative, calc_jac && fix_normal);
                        surf_df.block(idx,0,1,3) << surf_normal.transpose();
                    }
                    if(fix_normal) {
                        normal_val = calc_normal_value(surf_normal, tool_vec);
                        normal_f[idx] = normal_val;
                        if(calc_jac){
                            normal_gradient << calc_normal_gradient(surf_normal, surf_normal_derivative, tool_tf, jac_robot);
                            normal_df.block(idx,0,1,dims) << normal_gradient.transpose();
                        }
                    }
                }
                double surf_f_union;
                Eigen::VectorXd surf_df_union;
                Eigen::VectorXd surf_jac;

                double normal_f_union;
                Eigen::VectorXd normal_jac;

                if(fix_surface){
                    soft_min_weight_gradient_(surf_f, surf_df, surf_f, surf_df,
                                              surf_f_union, surf_df_union, calc_jac);
                }
                if(fix_normal) {
                    Eigen::MatrixXd surf_df_jac = surf_df*jac_robot.block(0,0,3,dims);
                    soft_min_weight_gradient_(surf_f, surf_df_jac, normal_f, normal_df,
                                              normal_f_union, normal_jac, calc_jac);
                }
                if(calc_jac)
                {
                    if(fix_surface){
                        surf_jac = surf_df_union.transpose()*jac_robot.block(0,0,3,dims);
                        jac.block(0,0,1, dims) << surf_jac.transpose();
                    }
                    if(fix_normal) {
                        jac.block(fix_surface?1:0,0,1, dims) << normal_jac.transpose();
                    }
#ifdef PRINT_DEBUG
                    std::cout<<"jac"<<std::endl;
                    std::cout<<jac<<std::endl;
#endif
                }
                else{
                    if(fix_surface){
                        out[0] = surf_f_union;
                    }
                    if(fix_normal) {
                        out[fix_surface?1:0] = normal_f_union;
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
#ifdef DEBUG_CONSTRAINT_VALUES
                Eigen::MatrixXd out_ref(out);
                ompl::base::Constraint::jacobian(x, out_ref);
                Eigen::VectorXd jac_diff = (out - out_ref).rowwise().norm();
                if (jac_diff.maxCoeff()>1e-5){
                    std::cout<<"jac_calc"<<std::endl;
                    std::cout<<out<<std::endl;
                    std::cout<<"jac_ref"<<std::endl;
                    std::cout<<out_ref<<std::endl;
                    std::cout<<"jac_diff: " << jac_diff.transpose() <<std::endl;
                    throw std::runtime_error("wrong jacobian calculation ");
                }
#endif
#ifdef PRINT_DEBUG
                std::cout<<"x"<<std::endl;
                std::cout<<x.transpose()<<std::endl;
                std::cout<<"jac_calc"<<std::endl;
                std::cout<<out<<std::endl;
                std::cout<<"jac_ref"<<std::endl;
                std::cout<<out_ref<<std::endl;
#endif
            }
#endif
        };

        typedef std::vector<UnionManifold> UnionManifoldList;
    }
}

#endif //MOVEIT_PLAN_COMPACT_OMPL_CUSTOM_CONSTRAINT_H
