#ifndef MOVIT_PLAN_PY_LIBRARY_H
#define MOVIT_PLAN_PY_LIBRARY_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>

#define MAX_STR_LEN 50000
#define MAX_NAME_LEN 32
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

struct c_string {
    char buffer [MAX_STR_LEN];
    int len=0;
};

struct c_trajectory {
    char names_flt[MAX_JOINT_NUM*MAX_NAME_LEN]={','};
    double joints[MAX_JOINT_NUM*MAX_TRAJ_LEN];
    int name_len=MAX_NAME_LEN;
    int joint_count;
    int joint_max=MAX_JOINT_NUM;
    int traj_len;
    bool success=true;
};

struct c_plan_goal {
    char group_name[MAX_NAME_LEN];
    char tool_link[MAX_NAME_LEN];
    char goal_link[MAX_NAME_LEN];
    double goal_pose[7];
    double timeout=0.1;
};

struct c_object_msg {
    char name[MAX_NAME_LEN];
    char link_name[MAX_NAME_LEN];
    double pose[7];
    double dims[3];
    int type;
    int action;
};

class PlannerCompact{
public:
    ros::NodeHandlePtr _node_handle;
    robot_model_loader::RobotModelLoaderPtr _robot_model_loader;
    robot_model::RobotModelPtr _robot_model;
    planning_scene::PlanningScenePtr _planning_scene;
    planning_pipeline::PlanningPipelinePtr _planning_pipeline;

    void init_planner(c_string urdf, c_string srdf);
    c_trajectory plan_compact(const char* group_name, const char* link_name,
                              const double* goal_pose, const char* goal_link,
                              double allowed_planning_time);
    void process_object(const char* name, const int type,
                    double* pose, double *dims,
                    const char* link_name, const int action);
    void clear_all_objects();
};

PlannerCompact* planner_compact=NULL;
ros::NodeHandlePtr init_ros();

extern "C" {
// testing hello
c_string hello_buffer;
c_string hello_cstr();
char* hello_char();
int get_max_str_len(){return MAX_STR_LEN;}

// planner functions
bool _ros_initialized = false;
void init_planner(c_string urdf, c_string srdf);
void process_object(c_object_msg omsg);
void clear_all_objects();
c_trajectory plan_compact(c_plan_goal goal);
void terminate_ros();
int get_max_name_len(){return MAX_NAME_LEN;}
int get_max_joint_num(){return MAX_JOINT_NUM;}
int get_max_traj_len(){return MAX_TRAJ_LEN;}
}

std::string WRAP_LOG_FRAME(const char* msg);
void PRINT_ERROR(const char* msg);
void PRINT_FRAMED_LOG(const char* msg, bool endl=false);

#endif //MOVIT_PLAN_PY_LIBRARY_H
