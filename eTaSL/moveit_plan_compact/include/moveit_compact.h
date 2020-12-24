#ifndef MOVIT_PLAN_PY_LIBRARY_H
#define MOVIT_PLAN_PY_LIBRARY_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <ompl-1.5/ompl/base/Constraint.h>
#include "ompl_planner_manager_custom.h"

#define MAX_NAME_LEN 32
#define MAX_NAME_NUM 32
#define MAX_JOINT_NUM 32
#define MAX_TRAJ_LEN 64

//BOX = 1u,
//SPHERE = 2u,
//CYLINDER = 3u,

//ADD = 0,
//REMOVE = 1,
//APPEND = 2,
//MOVE = 3,
namespace RNB {
    namespace MoveitCompact {
        using namespace std;

        //Planner* planner_compact=NULL;
        ros::NodeHandlePtr init_ros(string name="moveit_plan_compact");

        typedef vector<string> NameList;
        typedef Eigen::Matrix<double, 7, 1> CartPose;
        typedef Eigen::Vector3d Vec3;
        typedef Eigen::VectorXd JointState;
        typedef vector<JointState> Trajectory;

        struct PlanResult {
            Trajectory trajectory;
            bool success;
        };

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
            string tool_link;

            CustomConstraint(robot_model::RobotModelPtr _robot_model, string group_name, JointState init_state,
                             string tool_link, int dims) : ob::Constraint(dims, 1)
            {
                this->dims = dims;
                kinematic_state = std::make_shared<robot_state::RobotState>(_robot_model);
                joint_model_group = _robot_model->getJointModelGroup(group_name);
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

        /**
         * @brief A imlplementation of ompl planner using moveit! interface
         * @author Junsu Kang
         */
        class Planner {
        public:
            std::shared_ptr<ompl_interface::OMPLPlannerManagerCustom> _planner_manager;
            robot_model_loader::RobotModelLoaderPtr _robot_model_loader;
            robot_model::RobotModelPtr _robot_model;
            planning_scene::PlanningScenePtr _planning_scene;
            std::shared_ptr<CustomConstraint> _custom_constraint;

            PlanResult plan_result;
            NameList joint_names;
            int joint_num;

            /**
             * @brief initialize planner from urdf and srdf files. redirects to init_planner
             * @author Junsu Kang
             */
            bool init_planner_from_file(string urdf_filepath, string srdf_filepath, NameList &group_names, string config_path);

            /**
             * @brief initialize planner with string contents of urdf and srdf files.
             * @param string config_path directory path where kinematics.yaml, ompl_plugin.yaml, planning_plugin.yaml is stored
             * @author Junsu Kang
             */
            bool init_planner(string &urdf_txt, string &srdf_txt, NameList &group_names, string config_path);

            /**
             * @brief initialize planner with string contents of urdf and srdf files.
             * @author Junsu Kang
             */
            PlanResult &plan(string group_name, string tool_link,
                             CartPose goal_pose, string goal_link,
                             JointState init_state, string planner_id="RRTConnectkConfigDefault",
                             double allowed_planning_time=0.1);

            /**
             * @brief initialize planner with string contents of urdf and srdf files.
             * @author Junsu Kang
             */
            PlanResult &plan_with_constraint(string group_name, string tool_link,
                             CartPose goal_pose, string goal_link,
                             JointState init_state, string planner_id="RRTConnectkConfigDefault",
                             double allowed_planning_time=0.1);

            /**
             * @brief generate and process ros object message
             * @author Junsu Kang
             */
            bool process_object(string name, const int type, CartPose pose, Vec3 dims,
                                string link_name, NameList touch_links, bool attach, const int action);

            /**
             * @brief simply add object
             * @author Junsu Kang
             */
            bool add_object(string name, const int type,
                            CartPose pose, Vec3 dims,
                            string link_name, NameList touch_links, bool attach);

            /**
             * @brief clear all objects in the scene
             * @author Junsu Kang
             */
            void clear_all_objects();

            /**
             * @brief set z-plane manifold for constrained planning
             * @author Junsu Kang
             */
            void set_zplane_manifold(string group_name, JointState init_state, string tool_link);

            void terminate();
        };

    }
}

//extern "C" {
//// testing hello
//c_string hello_buffer;
//c_string hello_cstr();
//char* hello_char();
//int get_max_str_len(){return MAX_STR_LEN;}
//
//// planner functions
//bool _ros_initialized = false;
//c_name_arr init_planner(string& urdf_buffer, string srdf_buffer, list<<string>> group_names, int group_count);
//void process_object(c_object_msg omsg);
//void clear_all_objects();
//c_trajectory plan_compact(c_plan_request goal);
//void terminate_ros();
//int get_max_name_len(){return MAX_NAME_LEN;}
//int get_max_joint_num(){return MAX_JOINT_NUM;}
//int get_max_traj_len(){return MAX_TRAJ_LEN;}
//}

#endif //MOVIT_PLAN_PY_LIBRARY_H
