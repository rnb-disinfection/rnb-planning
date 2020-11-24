#include "../include/library.h"

#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


std::string WRAP_LOG_FRAME(const char* msg){
    int msg_len = strlen(msg);
    float margin = fmax(2, LOG_FRAME_LEN-msg_len);
    std::string prefix(floor(margin/2)-1, '=');
    std::string postfix(ceil(margin/2)-1, '=');
    std::string msg_line = prefix + " " + msg + " " + postfix + "\n";
    return msg_line;
}

void PRINT_ERROR(const char* msg){
    std::string msg_line = "\x1B[31m" + WRAP_LOG_FRAME(msg) + "\033[0m";
    std::cout<<TEXT_RED(LOG_FRAME_LINE);
    std::cout<<msg_line.c_str();
    std::cout<<TEXT_RED(LOG_FRAME_LINE "\n");
}

void PRINT_FRAMED_LOG(const char* msg, bool endl){
    std::cout<<WRAP_LOG_FRAME(msg).c_str();
    if(endl) std::cout<<std::endl;
}

c_string hello_cstr() {
    std::string msg("Hello, World!");
    int idx=0;
    for(auto it=msg.begin(); it!=msg.end(); it++){
        hello_buffer.buffer[idx++]=*it;
    }
    hello_buffer.buffer[idx] = (char) NULL;
    hello_buffer.len = idx;
    return hello_buffer;
}

char* hello_char() {
    std::string msg("Hello, World!");
    int idx=0;
    for(auto it=msg.begin(); it!=msg.end(); it++){
        hello_buffer.buffer[idx++]=*it;
    }
    hello_buffer.buffer[idx] = (char) NULL;
    hello_buffer.len = idx;
    return hello_buffer.buffer;
}

void PlannerCompact::init_planner(c_string urdf, c_string srdf){
    if(!_ros_initialized){
        _node_handle = init_ros();
    }

    _node_handle->setParam("planning_plugin", "ompl_interface/OMPLPlanner");

    _node_handle->setParam("indy0/kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    _node_handle->setParam("indy0/kinematics_solver_search_resolution", 0.005);
    _node_handle->setParam("indy0/kinematics_solver_timeout", 0.005);

    _node_handle->setParam("panda1/kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    _node_handle->setParam("panda1/kinematics_solver_search_resolution", 0.005);
    _node_handle->setParam("panda1/kinematics_solver_timeout", 0.005);

    _node_handle->setParam("indy0_to_panda1/kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    _node_handle->setParam("indy0_to_panda1/kinematics_solver_search_resolution", 0.005);
    _node_handle->setParam("indy0_to_panda1/kinematics_solver_timeout", 0.005);

    _node_handle->setParam("panda1_to_indy0/kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    _node_handle->setParam("panda1_to_indy0/kinematics_solver_search_resolution", 0.005);
    _node_handle->setParam("panda1_to_indy0/kinematics_solver_timeout", 0.005);



    PRINT_FRAMED_LOG("load robot model");
    _robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
            robot_model_loader::RobotModelLoader::Options(urdf.buffer, srdf.buffer));
    _robot_model = _robot_model_loader->getModel();
    if(_robot_model.get() == NULL) {
        PRINT_ERROR("failed to load robot model");
        return;
    }
    PRINT_FRAMED_LOG("loaded robot model", true);

    PRINT_FRAMED_LOG("load scene");
    _planning_scene = std::make_shared<planning_scene::PlanningScene>(_robot_model);
    if(_planning_scene==NULL){
        PRINT_ERROR("failed to load scene");
        return;
    }
    PRINT_FRAMED_LOG("loaded scene", true);

    PRINT_FRAMED_LOG("load pipeline");
    _planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(_robot_model, *_node_handle,
                                                    "planning_plugin", "request_adapters");
    if(_planning_pipeline==NULL){
        PRINT_ERROR("failed to load pipeline");
        return;
    }
    PRINT_FRAMED_LOG("loaded pipeline", true);
}

c_trajectory PlannerCompact::plan_compact(const char* group_name, const char* tool_link,
                                          const double* goal_pose, const char* goal_link){
    PRINT_FRAMED_LOG("set goal", true);
    geometry_msgs::PoseStamped _goal_pose;
    _goal_pose.header.frame_id = goal_link;
    _goal_pose.pose.position.x = goal_pose[0];
    _goal_pose.pose.position.y = goal_pose[1];
    _goal_pose.pose.position.z = goal_pose[2];
    _goal_pose.pose.orientation.x = goal_pose[3];
    _goal_pose.pose.orientation.y = goal_pose[4];
    _goal_pose.pose.orientation.z = goal_pose[5];
    _goal_pose.pose.orientation.w = goal_pose[6];

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    PRINT_FRAMED_LOG("constructGoalConstraints", true);
    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the Panda
    // specifying the desired pose of the end-effector as input.
    // We will create the request as a constraint using a helper function available
    moveit_msgs::Constraints _constrinat_pose_goal;
    planning_interface::MotionPlanRequest _req;
    planning_interface::MotionPlanResponse _res;
    _req.group_name = group_name; //"indy0"; // "indy0_tcp"
    _constrinat_pose_goal = kinematic_constraints::constructGoalConstraints(tool_link, _goal_pose, tolerance_pose, tolerance_angle);
    _req.goal_constraints.push_back(_constrinat_pose_goal);

    PRINT_FRAMED_LOG("generatePlan", true);
    // Now, call the pipeline and check whether planning was successful.
    _planning_pipeline->generatePlan(_planning_scene, _req, _res);
    /* Check that the planning was successful */
    c_trajectory trajectory_out;
    std::fill_n(trajectory_out.names_flt, sizeof(trajectory_out.names_flt), ' ');
    if (_res.error_code_.val != _res.error_code_.SUCCESS)
    {
        trajectory_out.success = false;
        PRINT_ERROR(("failed to generatePlan ("+std::to_string(_res.error_code_.val)+")").c_str());
        return trajectory_out;
    }

    trajectory_out.traj_len = _res.trajectory_->getWayPointCount();
    PRINT_FRAMED_LOG((std::string("convert trajectory - ")+std::to_string(trajectory_out.traj_len)).c_str(), true);

    auto names = _res.trajectory_->getFirstWayPoint().getVariableNames();
    trajectory_out.joint_count = 0;
    for(auto name_p=names.begin(); name_p!=names.end(); name_p++){
        auto name = *name_p;
        memcpy(trajectory_out.names_flt+(trajectory_out.joint_count*trajectory_out.name_len),
               name.c_str(), name.size());
        trajectory_out.joint_count++;
    }
    for( int i=0; i<trajectory_out.traj_len; i++){
        auto wp_buff = _res.trajectory_->getWayPoint(i).getVariablePositions();
        memcpy(trajectory_out.joints+i*MAX_JOINT_NUM, wp_buff, sizeof(double)*trajectory_out.joint_count);
    }
    PRINT_FRAMED_LOG("done", true);
    return trajectory_out;
}

ros::NodeHandlePtr init_ros() {
    _ros_initialized = true;
    char **argv;
    int argc=0;
    ros::init(argc, argv, "moveit_plan_compact");
    ros::NodeHandlePtr _node_handle = boost::make_shared<ros::NodeHandle>("~");
    signal(SIGINT, mySigintHandler);
    PRINT_FRAMED_LOG(LOG_FRAME_LINE);
    PRINT_FRAMED_LOG("rosnode moveit_plan_compact initialized");
    PRINT_FRAMED_LOG(LOG_FRAME_LINE, true);
    return _node_handle;
}

void terminate_ros(){
    PRINT_FRAMED_LOG("DELETE PLANNER", true);
    delete planner_compact;
    PRINT_FRAMED_LOG("SHUTDOWN PLANNER", true);
    ros::shutdown();
    while (ros::ok())
    {
        sleep(1);
        PRINT_FRAMED_LOG("SHUTTING DOWN PLANNER");
    }
    PRINT_FRAMED_LOG("FINISHED", true);
}

void init_planner(c_string urdf, c_string srdf){
    planner_compact = new PlannerCompact();
    planner_compact->init_planner(urdf, srdf);
}
c_trajectory plan_compact(c_plan_goal goal){
    return planner_compact->plan_compact(goal.group_name, goal.tool_link, goal.goal_pose, goal.goal_link);
}

int main(int argc, char** argv) {
    std::string urdf_txt;
    std::string srdf_txt;
    std::string tmp_txt;

// Read from the text file
    std::ifstream urdf_file("../test_assets/custom_robots.urdf");
    std::ifstream srdf_file("../test_assets/custom_robots.srdf");

// Use a while loop together with the getline() function to read the file line by line
    while (getline (urdf_file, tmp_txt)) {
        // Output the text from the file
        urdf_txt += tmp_txt;
        std::cout << tmp_txt << std::endl;
    }
    while (getline (srdf_file, tmp_txt)) {
        srdf_txt += tmp_txt;
        // Output the text from the file
        std::cout << tmp_txt <<std::endl;
    }
    printf("urdf_len: %d \n",(int)urdf_txt.length());
    printf("srdf_len: %d \n",(int)srdf_txt.length());
    c_string urdf_cstr;
    c_string srdf_cstr;
    memcpy(urdf_cstr.buffer, urdf_txt.c_str(), urdf_txt.length());
    memcpy(srdf_cstr.buffer, srdf_txt.c_str(), srdf_txt.length());
    init_planner(urdf_cstr, srdf_cstr);
    double goal[7] = {-0.3,-0.2,0.4,0,0,0,1};
    planner_compact->plan_compact("indy0", "indy0_tcp", goal, "base_link");
    planner_compact->plan_compact("indy0", "indy0_tcp", goal, "base_link");
    terminate_ros();
    return 0;
}