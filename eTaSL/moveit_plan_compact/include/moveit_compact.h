#ifndef MOVIT_PLAN_PY_LIBRARY_H
#define MOVIT_PLAN_PY_LIBRARY_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>

#include "bp_container_interface.h"
#define MAX_NAME_LEN 32
#define MAX_NAME_NUM 32
#define MAX_JOINT_NUM 32
#define MAX_TRAJ_LEN 64

#define LOG_FRAME_LINE "==================================================\n"
#define LOG_FRAME_LEN 50
#define TEXT_RED(STR) ("\x1B[31m" STR "\033[0m")
#define TEXT_GREEN(STR) ("\x1B[32m" STR "\033[0m")
#define TEXT_YELLOW(STR) ("\x1B[33m" STR "\033[0m")
#define TEXT_BLUE(STR) ("\x1B[34m" STR "\033[0m")
#define TEXT_CYAN(STR) ("\x1B[36m" STR "\033[0m")

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
        ros::NodeHandlePtr init_ros();

        typedef vector<string> NameList;
        typedef Eigen::Matrix<double, 7, 1> CartPose;
        typedef Eigen::Vector3d Vec3;
        typedef Eigen::VectorXd JointState;
        typedef vector<JointState> Trajectory;

        struct PlanResult {
            Trajectory trajectory;
            bool success = true;
        };

        class Planner {
        public:
            ros::NodeHandlePtr _node_handle;
            robot_model_loader::RobotModelLoaderPtr _robot_model_loader;
            robot_model::RobotModelPtr _robot_model;
            planning_scene::PlanningScenePtr _planning_scene;
            planning_pipeline::PlanningPipelinePtr _planning_pipeline;
            PlanResult plan_result;
            NameList joint_names;
            int joint_num;

            bool init_planner_from_file(string urdf_filepath, string srdf_filepath, NameList &group_names);

            bool init_planner(string &urdf_txt, string &srdf_txt, NameList &group_names);

            PlanResult &plan(string group_name, string tool_link,
                             CartPose goal_pose, string goal_link,
                             JointState init_state, double allowed_planning_time=0.1);

            bool process_object(string name, const int type,
                                CartPose pose, Vec3 dims,
                                string link_name, const int action);

            bool add_object(string name, const int type,
                            CartPose pose, Vec3 dims,
                            string link_name);

            void clear_all_objects();
        };

        string WRAP_LOG_FRAME(const char *msg);

        void PRINT_ERROR(const char *msg);

        void PRINT_FRAMED_LOG(const char *msg, bool endl = false);
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
