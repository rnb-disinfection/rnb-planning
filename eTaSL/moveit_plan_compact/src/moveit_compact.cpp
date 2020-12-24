#include "moveit_compact.h"

#include <string>
#include <signal.h>
#include <logger.h>
#include <ros_load_yaml.h>
#include <moveit/ompl_interface/ompl_interface.h>
//#include "ompl_planner_manager_custom.h"

using namespace RNB::MoveitCompact;

void SigintHandlerJ(int sig)
{
    ros::shutdown();
}


bool _ros_initialized = false;
ros::NodeHandlePtr _node_handle;
string _node_name;

ros::NodeHandlePtr RNB::MoveitCompact::init_ros(string name) {
    _ros_initialized = true;
    char **argv;
    int argc=0;
    ros::init(argc, argv, name);
    _node_name = name;
    ros::NodeHandlePtr _node_handle = boost::make_shared<ros::NodeHandle>("~");
    signal(SIGINT, SigintHandlerJ);
    PRINT_FRAMED_LOG(LOG_FRAME_LINE);
    PRINT_FRAMED_LOG("rosnode moveit_plan_compact initialized");
    PRINT_FRAMED_LOG(LOG_FRAME_LINE, true);
    return _node_handle;
}

bool Planner::init_planner_from_file(string urdf_filepath, string srdf_filepath, NameList& group_names, string config_path){
    std::string urdf_txt;
    std::string srdf_txt;
    std::string tmp_txt;

// Read from the text file
    std::ifstream urdf_file(urdf_filepath);
    std::ifstream srdf_file(srdf_filepath);

// Use a while loop together with the getline() function to read the file line by line
    while (getline (urdf_file, tmp_txt)) {
        // Output the text from the file
        urdf_txt += tmp_txt;
//        std::cout << tmp_txt << std::endl;
    }
    PRINT_FRAMED_LOG("Load URDF from: "+urdf_filepath);

    while (getline (srdf_file, tmp_txt)) {
        srdf_txt += tmp_txt;
        // Output the text from the file
//        std::cout << tmp_txt <<std::endl;
    }
    PRINT_FRAMED_LOG("Loaded SRDF from: "+srdf_filepath);
    return init_planner(urdf_txt, srdf_txt, group_names, config_path);
}


bool Planner::init_planner(string& urdf_txt, string& srdf_txt, NameList& group_names, string config_path){
    if(!_ros_initialized){
        _node_handle = init_ros();
    }

    //------------------------parsing with Iterator
    rosparam_load_yaml(_node_handle, "", config_path+"kinematics.yaml");
    rosparam_load_yaml(_node_handle, "", config_path+"ompl_planning.yaml");
    rosparam_load_yaml(_node_handle, "", config_path+"planning_plugin.yaml");

    PRINT_FRAMED_LOG("load robot model");
    _robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
            robot_model_loader::RobotModelLoader::Options(urdf_txt, srdf_txt));
    _robot_model = _robot_model_loader->getModel();
    if(_robot_model.get() == NULL) {
        PRINT_ERROR("failed to load robot model");
        return false;
    }
    PRINT_FRAMED_LOG("loaded robot model", true);

    PRINT_FRAMED_LOG("load scene");
    _planning_scene = std::make_shared<planning_scene::PlanningScene>(_robot_model);
    if(_planning_scene==NULL){
        PRINT_ERROR("failed to load scene");
        return false;
    }
    PRINT_FRAMED_LOG("loaded scene", true);

    PRINT_FRAMED_LOG("load pipeline");
    _planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(_robot_model, *_node_handle,
                                                    "planning_plugin", "request_adapters");
    if(_planning_pipeline==NULL){
        PRINT_ERROR("failed to load pipeline");
        return false;
    }

    _planning_pipeline->checkSolutionPaths(false);
    _planning_pipeline->publishReceivedRequests(false);
    _planning_pipeline->displayComputedMotionPlans(false);

    PRINT_FRAMED_LOG("loaded pipeline", true);
    auto names = _planning_scene->getCurrentState().getVariableNames();
    joint_num = 0;
    joint_names.clear();
    for(auto name_p=names.begin(); name_p!=names.end(); name_p++){
        joint_names.push_back(*name_p);
        joint_num++;
    }
    return true;
}

typedef std::function<const ompl_interface::ModelBasedStateSpaceFactoryPtr&(const std::string&)> StateSpaceFactoryTypeSelectorMirror;


PlanResult& Planner::plan(string group_name, string tool_link,
                          CartPose goal_pose, string goal_link,
                          JointState init_state, string planner_id, double allowed_planning_time){
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
    std::vector<double> tolerance_pose(3, 0.001);
    std::vector<double> tolerance_angle(3, 0.001);

//    PRINT_FRAMED_LOG("constructGoalConstraints", true);
    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the Panda
    // specifying the desired pose of the end-effector as input.
    // We will create the request as a constraint using a helper function available
    moveit_msgs::Constraints _constrinat_pose_goal;
    planning_interface::MotionPlanRequest _req;
    planning_interface::MotionPlanResponse _res;
    _req.group_name = group_name; //"indy0"; // "indy0_tcp"
    _req.planner_id = planner_id;
    _req.allowed_planning_time = allowed_planning_time;
    _constrinat_pose_goal = kinematic_constraints::constructGoalConstraints(tool_link, _goal_pose, tolerance_pose, tolerance_angle);
    _req.goal_constraints.push_back(_constrinat_pose_goal);

    auto state_cur = _planning_scene->getCurrentState();
    state_cur.setVariablePositions(init_state.data());
    _planning_scene->setCurrentState(state_cur);

    plan_result.trajectory.clear();

    std::vector<std::size_t> dummy;
    _planning_pipeline->generatePlan(_planning_scene, _req, _res, dummy);

//    ompl_interface::OMPLPlannerManagerCustom _planner_manager;
//    planning_interface::PlanningContextPtr context =
//            _planner_manager.getPlanningContext(_planning_scene, _req, _res.error_code_);
//
//    context->solve(_res);

    /* Check that the planning was successful */
    if (_res.error_code_.val != _res.error_code_.SUCCESS)
    {
        plan_result.success = false;
        PRINT_ERROR(("failed to generatePlan ("+std::to_string(_res.error_code_.val)+")").c_str());
        return plan_result;
    }
    int traj_len = _res.trajectory_->getWayPointCount();
    for( int i=0; i<traj_len; i++){
        const double* wp_buff = _res.trajectory_->getWayPoint(i).getVariablePositions();
        JointState wp(joint_num);
        memcpy(wp.data(), wp_buff, sizeof(double)*joint_num);
        plan_result.trajectory.push_back(wp);
    }

    PRINT_FRAMED_LOG((std::string("got trajectory - ")+std::to_string(plan_result.trajectory.size())).c_str(), true);

//    printf(LOG_FRAME_LINE);
//    PRINT_FRAMED_LOG("last pose below");
//    cout << *(plan_result.trajectory.end()-1) << endl;
//    printf(LOG_FRAME_LINE "\n");
    plan_result.success = true;
    return plan_result;
}

PlanResult& Planner::plan_fixz(string group_name, string tool_link,
                         CartPose goal_pose, string goal_link,
                         JointState init_state, string planner_id, double allowed_planning_time){
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
    std::vector<double> tolerance_pose(3, 0.001);
    std::vector<double> tolerance_angle(3, 0.001);

//    PRINT_FRAMED_LOG("constructGoalConstraints", true);
    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the Panda
    // specifying the desired pose of the end-effector as input.
    // We will create the request as a constraint using a helper function available
    moveit_msgs::Constraints _constrinat_pose_goal;
    planning_interface::MotionPlanRequest _req;
    planning_interface::MotionPlanResponse _res;
    _req.group_name = group_name; //"indy0"; // "indy0_tcp"
    _req.planner_id = planner_id;
    _req.allowed_planning_time = allowed_planning_time;
    _constrinat_pose_goal = kinematic_constraints::constructGoalConstraints(tool_link, _goal_pose, tolerance_pose, tolerance_angle);
    _req.goal_constraints.push_back(_constrinat_pose_goal);

    auto state_cur = _planning_scene->getCurrentState();
    state_cur.setVariablePositions(init_state.data());
    _planning_scene->setCurrentState(state_cur);

    plan_result.trajectory.clear();

    std::vector<std::size_t> dummy;
    _planning_pipeline->generatePlan(_planning_scene, _req, _res, dummy);

//    ompl_interface::OMPLPlannerManagerCustom _planner_manager;
//    planning_interface::PlanningContextPtr context =
//            _planner_manager.getPlanningContext(_planning_scene, _req, _res.error_code_);
//
//    context->solve(_res);

    /* Check that the planning was successful */
    if (_res.error_code_.val != _res.error_code_.SUCCESS)
    {
        plan_result.success = false;
        PRINT_ERROR(("failed to generatePlan ("+std::to_string(_res.error_code_.val)+")").c_str());
        return plan_result;
    }
    int traj_len = _res.trajectory_->getWayPointCount();
    for( int i=0; i<traj_len; i++){
        const double* wp_buff = _res.trajectory_->getWayPoint(i).getVariablePositions();
        JointState wp(joint_num);
        memcpy(wp.data(), wp_buff, sizeof(double)*joint_num);
        plan_result.trajectory.push_back(wp);
    }

    PRINT_FRAMED_LOG((std::string("got trajectory - ")+std::to_string(plan_result.trajectory.size())).c_str(), true);

//    printf(LOG_FRAME_LINE);
//    PRINT_FRAMED_LOG("last pose below");
//    cout << *(plan_result.trajectory.end()-1) << endl;
//    printf(LOG_FRAME_LINE "\n");
    plan_result.success = true;
    return plan_result;
}
bool Planner::process_object(string name, const int type, CartPose pose, Vec3 dims,
                    string link_name, NameList touch_links, bool attach, const int action){
    bool res = false;

    moveit_msgs::PlanningScene scene_msg;
    moveit_msgs::AttachedCollisionObject att_object;
    moveit_msgs::CollisionObject object;
    geometry_msgs::Pose _pose;
    shape_msgs::SolidPrimitive primitive;

    /* A default pose */
    _pose.position.x = pose[0];
    _pose.position.y = pose[1];
    _pose.position.z = pose[2];
    _pose.orientation.x = pose[3];
    _pose.orientation.y = pose[4];
    _pose.orientation.z = pose[5];
    _pose.orientation.w = pose[6];

    /* Define a box to be attached */
    primitive.type = type;
    switch(type){
        case primitive.BOX:
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = dims[0];
            primitive.dimensions[1] = dims[1];
            primitive.dimensions[2] = dims[2];
            printf("BOX: %f, %f, %f \n", dims[0], dims[1], dims[2]);
            break;
        case primitive.SPHERE:
            primitive.dimensions.resize(1);
            primitive.dimensions[0] = dims[0];
            printf("SPHERE: %f \n", dims[0]);
            break;
        case primitive.CYLINDER:
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = dims[0];
            primitive.dimensions[1] = dims[1];
            printf("CYLINDER: %f, %f \n", dims[0], dims[1]);
            break;
    }

    if (attach) {
        att_object.link_name = link_name;
        /* The header must contain a valid TF frame*/
        att_object.object.header.frame_id = link_name;
        /* The id of the object */
        att_object.object.id = name;
        att_object.object.primitives.push_back(primitive);
        att_object.object.primitive_poses.push_back(_pose);
        att_object.object.operation = action;
        att_object.touch_links = touch_links;
        res = _planning_scene->processAttachedCollisionObjectMsg(att_object);
    }
    else {
        object.header.frame_id = link_name;
        /* The id of the object */
        object.id = name;
        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(_pose);
        object.operation = action;
        res = _planning_scene->processCollisionObjectMsg(object);
    }

    return res;
}

bool Planner::add_object(string name, const int type,
                             CartPose pose, Vec3 dims,
                             string link_name, NameList touch_links, bool attach){
    return process_object(name, type, pose, dims, link_name, touch_links, attach, moveit_msgs::CollisionObject::ADD);
}

void Planner::clear_all_objects(){
    _planning_scene->removeAllCollisionObjects();
}

void Planner::set_zplane_manifold(string group_name, JointState init_state, string tool_link){
    _custom_constraint = std::make_shared<CustomConstraint>(_robot_model, group_name, init_state, tool_link, joint_num);
}

//void terminate_ros(){
//    PRINT_FRAMED_LOG("DELETE PLANNER", true);
//    delete planner_compact;
//    PRINT_FRAMED_LOG("SHUTDOWN PLANNER", true);
//    ros::shutdown();
//    while (ros::ok())
//    {
//        sleep(1);
//        PRINT_FRAMED_LOG("SHUTTING DOWN PLANNER");
//    }
//    PRINT_FRAMED_LOG("FINISHED", true);
//}
//
//c_name_arr init_planner(c_string urdf, c_string srdf, c_name_arr group_names_cstr){
//    if(planner_compact==NULL) {
//        planner_compact = new Planner();
//    }
//
//
//    auto group_names = get_c_name_arr(group_names_cstr);
//    bool res = planner_compact->init_planner(urdf.buffer, srdf.buffer, group_names);
//
//    c_name_arr joint_names_cna;
//    set_c_name_arr(joint_names_cna, planner_compact->joint_names);
//    return joint_names_cna;
//}
//
//void process_object(c_object_msg omsg){
//    planner_compact->process_object(
//            omsg.name, omsg.type, omsg.pose, omsg.dims, omsg.link_name, omsg.action);
//}
//
//void clear_all_objects(){
//    planner_compact->clear_all_objects();
//}
//
//c_trajectory plan_compact(c_plan_request goal){
//    return planner_compact->plan_compact(
//            goal.group_name, goal.tool_link, goal.goal_pose, goal.goal_link,
//            goal.init_state, goal.timeout);
//}

int main(int argc, char** argv) {
    NameList group_names;
    group_names.push_back("indy0");
    group_names.push_back("panda1");

    Planner planner;
    planner.init_planner_from_file("../test_assets/custom_robots.urdf", "../test_assets/custom_robots.srdf",
                                   group_names, "../test_assets/");
    JointState init_state(13);
    init_state << 0, 0, -1.57, 0, -1.57, 0, 0, -0.4, 0, -1.57, 0, 1.57, 1.57;
    CartPose goal;
    goal << -0.3,-0.2,0.4,0,0,0,1;
    planner.plan("indy0", "indy0_tcp", goal, "base_link", init_state);

//
//    double goal_obs[7] = {-0.3,-0.2,0.0,0,0,0,1};
//    double dims[3] = {0.1,0.1,0.1};
//    planner_compact->process_object(
//            "box", shape_msgs::SolidPrimitive::BOX,
//            goal_obs, dims,"base_link", moveit_msgs::CollisionObject::ADD);
//
//    clear_all_objects();
//    planner_compact->plan_compact("indy0", "indy0_tcp", goal, "base_link", init_state, 0.1);
//
//    double goal_obs2[7] = {-0.3,-0.2,0.4,0,0,0,1};
//    planner_compact->process_object(
//            "box", shape_msgs::SolidPrimitive::BOX,
//            goal_obs2, dims,"base_link", moveit_msgs::CollisionObject::ADD);
//    planner_compact->plan_compact("indy0", "indy0_tcp", goal, "base_link", init_state, 0.1);
//
//    terminate_ros();
    return 0;
}