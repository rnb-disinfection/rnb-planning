#ifndef MOVIT_PLAN_PY_LIBRARY_H
#define MOVIT_PLAN_PY_LIBRARY_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include "ompl_interface/ompl_planner_manager_custom.h"

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

        /**
         * @brief A imlplementation of ompl planner using moveit! interface
         * @author Junsu Kang
         */
        class Planner {
        public:
            std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
            std::string planner_plugin_name_;
            ompl_interface::OMPLPlannerManagerCustomPtr planner_instance_;
            robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
            robot_model::RobotModelPtr robot_model_;
            planning_scene::PlanningScenePtr planning_scene_;

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
             * @brief load planner plugin
             * @author Junsu Kang
             */
            void configure();

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
                                             JointState init_state, RNB::MoveitCompact::UnionManifoldPtr& custom_constraint,
                                             string planner_id="RRTConnectkConfigDefault",
                                             double allowed_planning_time=0.1, bool allow_approximation=false);

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
