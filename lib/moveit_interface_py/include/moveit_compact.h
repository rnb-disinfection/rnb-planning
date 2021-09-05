#ifndef MOVIT_PLAN_PY_LIBRARY_H
#define MOVIT_PLAN_PY_LIBRARY_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include "ompl_interface/ompl_planner_manager_custom.h"
#include "typedef.h"

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
        ros::NodeHandlePtr init_ros(string name="moveit_interface_py");

        /**
         * @brief A imlplementation of ompl planner using moveit! interface
         * @author Junsu Kang
         */
        class Planner {
        public:
            std::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
            std::string planner_plugin_name_;
            std::vector<std::string> adapter_plugin_names_;
            ompl_interface::OMPLPlannerManagerCustomPtr planner_instance_;
            std::shared_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter> > adapter_plugin_loader_;
            std::shared_ptr<planning_request_adapter::PlanningRequestAdapterChain> adapter_chain_;
            robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
            robot_model::RobotModelPtr robot_model_;
            planning_scene::PlanningScenePtr planning_scene_;
            std::vector<ompl::base::ConstraintPtr> manifolds;
            double tolerance_pose;
            double tolerance_angle;
            double tolerance_pose_const;
            double tolerance_angle_const;
            double tolerance_joint;
            JointState tol_vals;

            bool check_solution_paths_;
            PlanResult plan_result;
            NameList joint_names;
            int joint_num;

            JointState result_ik;
            JointState result_jac;

            /**
             * @brief initialize planner from urdf and srdf files. redirects to init_planner
             * @author Junsu Kang
             */
            bool init_planner_from_file(string urdf_filepath, string srdf_filepath, NameList &group_names, string config_path);

            /**
             * @brief initialize planner with string contents of urdf and srdf files.
             * @param config_path directory path where kinematics.yaml, ompl_plugin.yaml, planning_plugin.yaml is stored
             * @author Junsu Kang
             */
            bool init_planner(string &urdf_txt, string &srdf_txt, NameList &group_names, string config_path);

            /**
             * @brief load planner plugin
             * @author Junsu Kang
             */
            void configure();

            /**
             * @brief clear_context_cache - needed to call to reset planning context
             * @author Junsu Kang
             */
            void clear_context_cache();

            /**
             * @brief create union manifold
             * @author Junsu Kang
             */
            bool add_union_manifold(string group_name, string tool_link, CartPose tool_offset,
                                    GeometryList geometry_list, bool fix_surface, bool fix_normal,
                                    double tol=UnionManifold::DEFAULT_TOLERANCE);

            /**
             * @brief clear all manifolds
             * @author Junsu Kang
             */
            bool clear_manifolds();

            /**
             * @brief set flag for post-checking solution paths
             * @author Junsu Kang
             */
            void checkSolutionPaths(bool flag);

            /**
             * @brief search for plan.
             * @author Junsu Kang
             */
            PlanResult &plan(string group_name, string tool_link,
                             CartPose goal_pose, string goal_link,
                             JointState init_state, string planner_id="RRTConnectkConfigDefault",
                             double allowed_planning_time=0.1,
                             double vel_scale=0.1, double acc_scale=0.1, bool post_opt=false);

            /**
             * @brief search for joint motion plan.
             * @author Junsu Kang
             */
            PlanResult &plan_joint_motion(string group_name, JointState goal_state, JointState init_state,
                             string planner_id="RRTConnectkConfigDefault", double allowed_planning_time=0.1,
                                          double vel_scale=0.1, double acc_scale=0.1, bool post_opt=false);

            /**
             * @brief search for plan with constraints.
             * @author Junsu Kang
             */
            PlanResult &plan_with_constraints(string group_name, string tool_link,
                                             CartPose goal_pose, string goal_link,
                                             JointState init_state,
                                             string planner_id="RRTConnectkConfigDefault",
                                             double allowed_planning_time=0.1,
                                             double vel_scale=0.1, double acc_scale=0.1, bool post_opt=false,
                                             ompl_interface::ConstrainedSpaceType cs_type=
                                                     ompl_interface::ConstrainedSpaceType::ATLAS,
                                             bool allow_approximation=false,
                                             bool post_projection=false);

            /**
             * @brief set tolerance for planning
             */
            void set_tolerance(double pose=-1, double angle=-1, double pose_const=-1, double angle_const=-1, double joint=-1);

            /**
             * @brief get current tolerance for planning
             */
            JointState &get_tolerance();

            /**
             * @brief test jacobian
             * @author Junsu Kang
             */
            void test_jacobian(JointState init_state);

            /**
             * @brief validate joint moition trajectory
             * @author Junsu Kang
             */
            bool validate_trajectory(Trajectory trajectory);

            /**
             * @brief generate and process ros object message
             * @author Junsu Kang
             */
            bool process_object(string name, const ObjectType type, CartPose pose, Vec3 dims,
                                string link_name, NameList touch_links, bool attach, const int action);

            /**
             * @brief simply add object
             * @author Junsu Kang
             */
            bool add_object(string name, const ObjectType type,
                            CartPose pose, Vec3 dims,
                            string link_name, NameList touch_links, bool attach);

            /**
             * @brief clear all objects in the scene
             * @author Junsu Kang
             */
            void clear_all_objects();

            void terminate();

            /**
             * @brief solve inverse kinematics
             * @param timeout_single    timeout for single ik
             * @param timeout_sampling  timeout for sampling loop
             * @param self_collision    to check self-collision
             * @param fulll_collision   to check full collision with environment
             * @author Junsu Kang
             */
            JointState& solve_ik(string group_name, CartPose goal_pose,
                                double timeout_single, double timeout_sampling,
                                bool self_collision, bool fulll_collision);

            /**
             * @brief check current status of collision
             * @author Junsu Kang
             * @param only_self    to check only self-collision
             */
            bool check_collision(bool only_self);

            JointState& get_jacobian(string group_name, JointState Q);
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
