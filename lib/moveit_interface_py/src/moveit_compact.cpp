#include "moveit_compact.h"

#include <string>
#include <signal.h>
#include <logger.h>
#include <ros_load_yaml.h>
#include "constants.h"
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

using namespace RNB::MoveitCompact;

void SigintHandlerJ(int sig)
{
    ros::shutdown();
}


bool __ros_initialized = false;
ros::NodeHandlePtr __node_handle;
string __node_name;

ros::NodeHandlePtr RNB::MoveitCompact::init_ros(string name) {
    __ros_initialized = true;
    char **argv;
    int argc=0;
    ros::init(argc, argv, name);
    __node_name = name;
    ros::NodeHandlePtr __node_handle = boost::make_shared<ros::NodeHandle>("~");
    signal(SIGINT, SigintHandlerJ);
    PRINT_FRAMED_LOG(LOG_FRAME_LINE);
    PRINT_FRAMED_LOG("rosnode moveit_interface_py initialized");
    PRINT_FRAMED_LOG(LOG_FRAME_LINE, true);
    return __node_handle;
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

void Planner::set_tolerance(double pose, double angle, double pose_const, double angle_const, double joint){
    if(pose>=0) tolerance_pose = pose;
    if(angle>=0) tolerance_angle = angle;
    if(pose_const>=0) tolerance_pose_const = pose_const;
    if(angle_const>=0) tolerance_angle_const = angle_const;
    if(joint>=0) tolerance_joint = joint;
    tol_vals.setZero(5);
    tol_vals << tolerance_pose, tolerance_angle, tolerance_pose_const, tolerance_angle_const, tolerance_joint;
}

JointState& Planner::get_tolerance(){
    return tol_vals;
}


bool Planner::init_planner(string& urdf_txt, string& srdf_txt, NameList& group_names, string config_path){
    if(!__ros_initialized){
        __node_handle = init_ros();
    }

    set_tolerance(0.0002, 0.001, 0.001, 0.001, 0.01);

    //------------------------parsing with Iterator
    rosparam_load_yaml(__node_handle, "/"+__node_name, config_path+"kinematics.yaml");
    rosparam_load_yaml(__node_handle, "/"+__node_name, config_path+"ompl_planning.yaml");
    rosparam_load_yaml(__node_handle, "/"+__node_name, config_path+"planning_plugin.yaml");
//    rosparam_load_yaml(__node_handle, "/"+__node_name, config_path+"stomp_planning.yaml");
//    rosparam_load_yaml(__node_handle, "/"+__node_name, config_path+"chomp_planning.yaml");

    PRINT_FRAMED_LOG("load robot model");
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            robot_model_loader::RobotModelLoader::Options(urdf_txt, srdf_txt));
    robot_model_ = robot_model_loader_->getModel();
    if(robot_model_.get() == NULL) {
        PRINT_ERROR("failed to load robot model");
        return false;
    }
    PRINT_FRAMED_LOG("loaded robot model", true);

    PRINT_FRAMED_LOG("load scene");
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    if(planning_scene_==NULL){
        PRINT_ERROR("failed to load scene");
        return false;
    }
    PRINT_FRAMED_LOG("loaded scene", true);

    PRINT_FRAMED_LOG("load planner");
    configure();
    PRINT_FRAMED_LOG("loaded planner", true);

    auto names = planning_scene_->getCurrentState().getVariableNames();
    joint_num = 0;
    joint_names.clear();
    for(auto name_p=names.begin(); name_p!=names.end(); name_p++){
        joint_names.push_back(*name_p);
        joint_num++;
    }
    current_state.setZero(joint_num);
    return true;
}

void Planner::configure()
{
    check_solution_paths_ = false;  // this is set to true below

    std::string planner;
    if (__node_handle->getParam("planning_plugin", planner))
        planner_plugin_name_ = planner;

    std::string adapters;
    if (__node_handle->getParam("request_adapters", adapters))
    {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char> > tok(adapters, sep);
        for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
            adapter_plugin_names_.push_back(*beg);
    }

    // load the planning plugin
    try
    {
        planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    std::vector<std::string> classes;
    if (planner_plugin_loader_)
        classes = planner_plugin_loader_->getDeclaredClasses();
    if (planner_plugin_name_.empty() && classes.size() == 1)
    {
        planner_plugin_name_ = classes[0];
        ROS_INFO("No '~planning_plugin' parameter specified, but only '%s' planning plugin is available. Using that one.",
                 planner_plugin_name_.c_str());
    }
    if (planner_plugin_name_.empty() && classes.size() > 1)
    {
        planner_plugin_name_ = classes[0];
        ROS_INFO("Multiple planning plugins available. You should specify the '~planning_plugin' parameter. Using '%s' for "
                 "now.",
                 planner_plugin_name_.c_str());
    }

    if (planner_plugin_name_ == "ompl_interface/OMPLPlannerCustom")
    {
        try {
            planner_instance_ = std::make_shared<ompl_interface::OMPLPlannerManagerCustom>();
            if (!planner_instance_->initialize(robot_model_, __node_handle->getNamespace()))
                throw std::runtime_error("Unable to initialize planning plugin");
            ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
        }
        catch (pluginlib::PluginlibException& ex) {
            ROS_ERROR_STREAM("Exception while loading planner '"
                                     << planner_plugin_name_ << "': " << ex.what() << std::endl
                                     << "Available plugins: " << boost::algorithm::join(classes, ", "));
        }
    }
    else
    {
        ROS_ERROR_STREAM("Exception while loading planner: No available plugin");
    }
    // load the planner request adapters
    if (!adapter_plugin_names_.empty())
    {
        std::vector<planning_request_adapter::PlanningRequestAdapterPtr> ads;
        try
        {
            adapter_plugin_loader_.reset(new pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>(
                    "moveit_core", "planning_request_adapter::PlanningRequestAdapter"));
        }
        catch (pluginlib::PluginlibException& ex)
        {
            ROS_ERROR_STREAM("Exception while creating planning plugin loader " << ex.what());
        }

        if (adapter_plugin_loader_)
            for (const std::string& adapter_plugin_name : adapter_plugin_names_)
            {
                planning_request_adapter::PlanningRequestAdapterPtr ad;
                try
                {
                    ad = adapter_plugin_loader_->createUniqueInstance(adapter_plugin_name);
                }
                catch (pluginlib::PluginlibException& ex)
                {
                    ROS_ERROR_STREAM("Exception while loading planning adapter plugin '" << adapter_plugin_name
                                                                                         << "': " << ex.what());
                }
                if (ad)
                {
                    ads.push_back(std::move(ad));
                }
            }
        if (!ads.empty())
        {
            adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
            for (planning_request_adapter::PlanningRequestAdapterPtr& ad : ads)
            {
                ROS_INFO_STREAM("Using planning request adapter '" << ad->getDescription() << "'");
                adapter_chain_->addAdapter(ad);
            }
        }
    }
    checkSolutionPaths(true);
}

void Planner::checkSolutionPaths(bool flag)
{
    check_solution_paths_ = flag;
}

PlanResult &Planner::plan(string group_name, string tool_link,
                 CartPose goal_pose, string goal_link,
                 JointState init_state, string planner_id,
                 double allowed_planning_time,
                 double vel_scale, double acc_scale, bool post_opt){
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

    auto state_cur = planning_scene_->getCurrentState();
    state_cur.setVariablePositions(init_state.data());
    planning_scene_->setCurrentState(state_cur);

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
    _req.goal_constraints.clear();
    _req.goal_constraints.push_back(_constrinat_pose_goal);
    moveit::core::robotStateToRobotStateMsg(state_cur, _req.start_state);
    _req.max_acceleration_scaling_factor = acc_scale;
    _req.max_velocity_scaling_factor = vel_scale;

    plan_result.trajectory.clear();

    bool solved;
    std::vector<std::size_t> adapter_added_state_index;
    try
    {
        if (post_opt && adapter_chain_)
        {
            solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene_, _req, _res, adapter_added_state_index);
            if (!adapter_added_state_index.empty())
            {
                std::stringstream ss;
                for (std::size_t added_index : adapter_added_state_index)
                    ss << added_index << " ";
                ROS_INFO("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
            }
        }
        else
        {

            planning_interface::PlanningContextPtr context =
                    planner_instance_->getPlanningContext(planning_scene_, _req, _res.error_code_);
    solved = context->solve(_res);
        }
    }
    catch (std::exception& ex)
    {
        ROS_ERROR("Exception caught: '%s'", ex.what());
        solved = false;
    }

#ifdef PRINT_DEBUG
    std::cout <<"init_state: "<<init_state.transpose()<<std::endl;
    std::cout <<"goal_pose: "<<goal_pose.transpose()<<std::endl;
#endif

    bool valid = true;

    if (solved && _res.trajectory_)
    {
        std::size_t state_count = _res.trajectory_->getWayPointCount();
        ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
        if (check_solution_paths_)
        {
            std::vector<std::size_t> index;
            if (!planning_scene_->isPathValid(*_res.trajectory_, _req.path_constraints, _req.group_name, false, &index))
            {
                // check to see if there is any problem with the states that are found to be invalid
                // they are considered ok if they were added by a planning request adapter
                bool problem = false;
                for (std::size_t i = 0; i < index.size() && !problem; ++i)
                {
                    bool found = false;
                    for (std::size_t added_index : adapter_added_state_index)
                        if (index[i] == added_index)
                        {
                            found = true;
                            break;
                        }
                    if (!found)
                        problem = true;
                }
                if (problem)
                {
                    if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
                        ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
                    else
                    {
                        valid = false;
                        _res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;

                        // display error messages
                        std::stringstream ss;
                        for (std::size_t it : index)
                            ss << it << " ";
                        ROS_ERROR_STREAM("Computed path is not valid. Invalid states at index locations: [ "
                                                 << ss.str() << "] out of " << state_count
                                                 << ". Explanations follow in command line.");

                        // call validity checks in verbose mode for the problematic states
                        visualization_msgs::MarkerArray arr;
                        for (std::size_t it : index)
                        {
                            // check validity with verbose on
                            const moveit::core::RobotState& robot_state = _res.trajectory_->getWayPoint(it);
                            planning_scene_->isStateValid(robot_state, _req.path_constraints, _req.group_name, true);

                            // compute the contacts if any
                            collision_detection::CollisionRequest c_req;
                            collision_detection::CollisionResult c_res;
                            c_req.contacts = true;
                            c_req.max_contacts = 10;
                            c_req.max_contacts_per_pair = 3;
                            c_req.verbose = false;
                            planning_scene_->checkCollision(c_req, c_res, robot_state);
                            if (c_res.contact_count > 0)
                            {
                                visualization_msgs::MarkerArray arr_i;
                                collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene_->getPlanningFrame(),
                                                                                     c_res.contacts);
                                arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
                            }
                        }
                        ROS_ERROR_STREAM("Completed listing of explanations for invalid states.");
                    }
                }
                else
                    ROS_DEBUG("Planned path was found to be valid, except for states that were added by planning request "
                              "adapters, but that is ok.");
            }
            else
                ROS_DEBUG("Planned path was found to be valid when rechecked");
        }
    }

    /* Check that the planning was successful */
    if ((!(solved&&valid)) || (_res.error_code_.val != _res.error_code_.SUCCESS))
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


PlanResult& Planner::plan_joint_motion(string group_name, JointState goal_state, JointState init_state,
                                       string planner_id, double allowed_planning_time,
                                       double vel_scale, double acc_scale, bool post_opt){
    PRINT_FRAMED_LOG("set goal", true);
    robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
    robot_state::RobotState goal_state_moveit(robot_model_);
    goal_state_moveit.setJointGroupPositions(joint_model_group, goal_state);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(
            goal_state_moveit, joint_model_group, tolerance_joint);

    auto state_cur = planning_scene_->getCurrentState();
    state_cur.setVariablePositions(init_state.data());
    planning_scene_->setCurrentState(state_cur);

    planning_interface::MotionPlanRequest _req;
    planning_interface::MotionPlanResponse _res;
    _req.group_name = group_name; //"indy0"; // "indy0_tcp"
    _req.planner_id = planner_id;
    _req.allowed_planning_time = allowed_planning_time;
    _req.goal_constraints.clear();
    _req.goal_constraints.push_back(joint_goal);
    moveit::core::robotStateToRobotStateMsg(state_cur, _req.start_state);
    _req.max_acceleration_scaling_factor = acc_scale;
    _req.max_velocity_scaling_factor = vel_scale;

    plan_result.trajectory.clear();

    bool solved;
    std::vector<std::size_t> adapter_added_state_index;
    try
    {
        if (post_opt && adapter_chain_)
        {
            solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene_, _req, _res, adapter_added_state_index);
            if (!adapter_added_state_index.empty())
            {
                std::stringstream ss;
                for (std::size_t added_index : adapter_added_state_index)
                    ss << added_index << " ";
                ROS_INFO("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
            }
        }
        else
        {

            planning_interface::PlanningContextPtr context =
                    planner_instance_->getPlanningContext(planning_scene_, _req, _res.error_code_);
            solved = context->solve(_res);
        }
    }
    catch (std::exception& ex)
    {
        ROS_ERROR("Exception caught: '%s'", ex.what());
        solved = false;
    }

#ifdef PRINT_DEBUG
    std::cout <<"init_state: "<<init_state.transpose()<<std::endl;
    std::cout <<"goal_pose: "<<goal_pose.transpose()<<std::endl;
#endif

    bool valid = true;

    if (solved && _res.trajectory_)
    {
        std::vector<std::size_t> adapter_added_state_index;
        std::size_t state_count = _res.trajectory_->getWayPointCount();
        ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
        if (check_solution_paths_)
        {
            std::vector<std::size_t> index;
            if (!planning_scene_->isPathValid(*_res.trajectory_, _req.path_constraints, _req.group_name, false, &index))
            {
                // check to see if there is any problem with the states that are found to be invalid
                // they are considered ok if they were added by a planning request adapter
                bool problem = false;
                for (std::size_t i = 0; i < index.size() && !problem; ++i)
                {
                    bool found = false;
                    for (std::size_t added_index : adapter_added_state_index)
                        if (index[i] == added_index)
                        {
                            found = true;
                            break;
                        }
                    if (!found)
                        problem = true;
                }
                if (problem)
                {
                    if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
                        ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
                    else
                    {
                        valid = false;
                        _res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;

                        // display error messages
                        std::stringstream ss;
                        for (std::size_t it : index)
                            ss << it << " ";
                        ROS_ERROR_STREAM("Computed path is not valid. Invalid states at index locations: [ "
                                                 << ss.str() << "] out of " << state_count
                                                 << ". Explanations follow in command line.");

                        // call validity checks in verbose mode for the problematic states
                        visualization_msgs::MarkerArray arr;
                        for (std::size_t it : index)
                        {
                            // check validity with verbose on
                            const moveit::core::RobotState& robot_state = _res.trajectory_->getWayPoint(it);
                            planning_scene_->isStateValid(robot_state, _req.path_constraints, _req.group_name, true);

                            // compute the contacts if any
                            collision_detection::CollisionRequest c_req;
                            collision_detection::CollisionResult c_res;
                            c_req.contacts = true;
                            c_req.max_contacts = 10;
                            c_req.max_contacts_per_pair = 3;
                            c_req.verbose = false;
                            planning_scene_->checkCollision(c_req, c_res, robot_state);
                            if (c_res.contact_count > 0)
                            {
                                visualization_msgs::MarkerArray arr_i;
                                collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene_->getPlanningFrame(),
                                                                                     c_res.contacts);
                                arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
                            }
                        }
                        ROS_ERROR_STREAM("Completed listing of explanations for invalid states.");
                    }
                }
                else
                    ROS_DEBUG("Planned path was found to be valid, except for states that were added by planning request "
                              "adapters, but that is ok.");
            }
            else
                ROS_DEBUG("Planned path was found to be valid when rechecked");
        }
    }

    /* Check that the planning was successful */
    if ((!(solved&&valid)) || (_res.error_code_.val != _res.error_code_.SUCCESS))
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

void Planner::clear_context_cache(){
    planner_instance_->resetContextCache();
}

PlanResult& Planner::plan_with_constraints(string group_name, string tool_link,
                         CartPose goal_pose, string goal_link,
                         JointState init_state, string planner_id, double allowed_planning_time,
                         double vel_scale, double acc_scale, bool post_opt,
                         ompl_interface::ConstrainedSpaceType cs_type, bool allow_approximation,
                         bool post_projection){

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

    auto state_cur = planning_scene_->getCurrentState();
    state_cur.setVariablePositions(init_state.data());
    planning_scene_->setCurrentState(state_cur);

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
    _constrinat_pose_goal = kinematic_constraints::constructGoalConstraints(
            tool_link, _goal_pose, tolerance_pose_const, tolerance_angle_const);
    _req.goal_constraints.clear();
    _req.goal_constraints.push_back(_constrinat_pose_goal);
    moveit::core::robotStateToRobotStateMsg(state_cur, _req.start_state);
    _req.max_acceleration_scaling_factor = acc_scale;
    _req.max_velocity_scaling_factor = vel_scale;

    plan_result.trajectory.clear();

    if (manifolds.size()==0){
        plan_result.success = false;
        return plan_result;
    }
    auto manifold_ref = manifolds[0];
    ompl::base::ConstraintIntersectionPtr manifold_intersection = std::make_shared<ompl::base::ConstraintIntersection>(
            manifold_ref->getAmbientDimension(), manifolds);
    double tol = 0;
    for(auto _man = manifolds.begin(); _man!=manifolds.end(); _man++){
        tol += pow((*_man)->getTolerance(),2);
    }
    tol = sqrt(tol);
    manifold_intersection->setTolerance(tol);

    bool solved;
    std::vector<std::size_t> adapter_added_state_index;
    try
    {
        // No path-adaptation, as constrained motion is directly generated from OMPL, not available from ROS
        if(post_opt){
            ROS_WARN("Path optimization not available with constrained motion");
            post_opt = false;
        }
        if (post_opt && adapter_chain_)
        {
            solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene_, _req, _res, adapter_added_state_index);
            if (!adapter_added_state_index.empty())
            {
                std::stringstream ss;
                for (std::size_t added_index : adapter_added_state_index)
                    ss << added_index << " ";
                ROS_INFO("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
            }
        }
        else
        {
            planning_interface::PlanningContextPtr context =
                planner_instance_->getPlanningContextConstrained(planning_scene_, _req, _res.error_code_,
                                                                 manifold_intersection, cs_type,allow_approximation);
            solved = context->solve(_res);
        }
    }
    catch (std::exception& ex)
    {
        ROS_ERROR("Exception caught: '%s'", ex.what());
        solved = false;
    }

#ifdef PRINT_DEBUG
    std::cout <<"init_state: "<<init_state.transpose()<<std::endl;
    std::cout <<"goal_pose: "<<goal_pose.transpose()<<std::endl;
#endif

    bool valid = true;

    if (solved && _res.trajectory_)
    {
        std::vector<std::size_t> adapter_added_state_index;
        std::size_t state_count = _res.trajectory_->getWayPointCount();
        ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
        if (check_solution_paths_)
        {
            std::vector<std::size_t> index;
            if (!planning_scene_->isPathValid(*_res.trajectory_, _req.path_constraints, _req.group_name, false, &index))
            {
                // check to see if there is any problem with the states that are found to be invalid
                // they are considered ok if they were added by a planning request adapter
                bool problem = false;
                for (std::size_t i = 0; i < index.size() && !problem; ++i)
                {
                    bool found = false;
                    for (std::size_t added_index : adapter_added_state_index)
                        if (index[i] == added_index)
                        {
                            found = true;
                            break;
                        }
                    if (!found)
                        problem = true;
                }
                if (problem)
                {
                    if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
                        ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
                    else
                    {
                        valid = false;
                        _res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;

                        // display error messages
                        std::stringstream ss;
                        for (std::size_t it : index)
                            ss << it << " ";
                        ROS_ERROR_STREAM("Computed path is not valid. Invalid states at index locations: [ "
                                                 << ss.str() << "] out of " << state_count
                                                 << ". Explanations follow in command line.");

                        // call validity checks in verbose mode for the problematic states
                        visualization_msgs::MarkerArray arr;
                        for (std::size_t it : index)
                        {
                            // check validity with verbose on
                            const moveit::core::RobotState& robot_state = _res.trajectory_->getWayPoint(it);
                            planning_scene_->isStateValid(robot_state, _req.path_constraints, _req.group_name, true);

                            // compute the contacts if any
                            collision_detection::CollisionRequest c_req;
                            collision_detection::CollisionResult c_res;
                            c_req.contacts = true;
                            c_req.max_contacts = 10;
                            c_req.max_contacts_per_pair = 3;
                            c_req.verbose = false;
                            planning_scene_->checkCollision(c_req, c_res, robot_state);
                            if (c_res.contact_count > 0)
                            {
                                visualization_msgs::MarkerArray arr_i;
                                collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene_->getPlanningFrame(),
                                                                                     c_res.contacts);
                                arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
                            }
                        }
                        ROS_ERROR_STREAM("Completed listing of explanations for invalid states.");
                    }
                }
                else
                    ROS_DEBUG("Planned path was found to be valid, except for states that were added by planning request "
                              "adapters, but that is ok.");
            }
            else
                ROS_DEBUG("Planned path was found to be valid when rechecked");
        }
    }

    /* Check that the planning was successful */
    if ((!(solved&&valid)) || (_res.error_code_.val != _res.error_code_.SUCCESS))
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

    printf(LOG_FRAME_LINE);
    PRINT_FRAMED_LOG("last pose below");
    cout << (plan_result.trajectory.end()-1)->transpose() << endl;
    printf(LOG_FRAME_LINE "\n");
    plan_result.success = true;
    if(cs_type==ompl_interface::ConstrainedSpaceType::TANGENTBUNDLE){
        if(post_projection){
            for(auto itor=plan_result.trajectory.begin(); itor!=plan_result.trajectory.end(); itor++){
                manifold_intersection->project(*itor);
            }
        }
    }
    return plan_result;
}

void Planner::test_jacobian(JointState init_state){
    if (manifolds.size()==0){
        std::cout<<"manifold list is empty"<<std::endl;
        return;
    }
    auto manifold_ref = manifolds[0];
    ompl::base::ConstraintIntersectionPtr manifold_intersection = std::make_shared<ompl::base::ConstraintIntersection>(
            manifold_ref->getAmbientDimension(), manifolds);
    double tol = 0;
    for(auto _man = manifolds.begin(); _man!=manifolds.end(); _man++){
        tol += pow((*_man)->getTolerance(),2);
        std::cout<<"tolerance part: "<<(*_man)->getTolerance()<<std::endl;
    }
    tol = sqrt(tol);
    std::cout<<"tolerance original: "<<manifold_intersection->getTolerance()<<std::endl;
    manifold_intersection->setTolerance(tol);
    std::cout<<"tolerance changed: "<<manifold_intersection->getTolerance()<<std::endl;

    Eigen::MatrixXd jac(manifold_intersection->getCoDimension(), manifold_intersection->getAmbientDimension());
    manifold_intersection->jacobian(init_state, jac);

    Eigen::VectorXd f(manifold_intersection->getCoDimension());
    manifold_intersection->function(init_state, f);
    double tolerance_ = manifold_intersection->getTolerance();
    bool satisfied = manifold_intersection->isSatisfied(init_state);
    std::cout<<"f: "<< f.transpose() <<std::endl;
    std::cout<<"squaredNorm/squaredTol: "<<f.squaredNorm() << " / " << tolerance_*tolerance_ <<std::endl;
    std::cout<<"satisfied: "<< satisfied <<std::endl<<std::endl;
}

bool Planner::validate_trajectory(Trajectory trajectory){
    bool _valid = true;
    robot_state::RobotState robot_state = planning_scene_->getCurrentState();
    collision_detection::CollisionResult res;
    collision_detection::CollisionRequest collision_req;
    for(auto titor=trajectory.begin(); titor!=trajectory.end(); titor++){
        robot_state.setVariablePositions(titor->data());
        planning_scene_->checkCollision(collision_req, res, robot_state);
        _valid = !res.collision;
        if (!_valid)
            break;
    }
    return _valid;
}

bool Planner::process_object(string name, const ObjectType type, CartPose pose, Vec3 dims,
                    string link_name, NameList touch_links, bool attach, const int action){
    bool res = false;

    moveit_msgs::AttachedCollisionObject att_object;
    moveit_msgs::CollisionObject object;
    geometry_msgs::Pose _pose;

    /* A default pose */
    _pose.position.x = pose[0];
    _pose.position.y = pose[1];
    _pose.position.z = pose[2];
    _pose.orientation.x = pose[3];
    _pose.orientation.y = pose[4];
    _pose.orientation.z = pose[5];
    _pose.orientation.w = pose[6];

    shape_msgs::SolidPrimitive primitive;
    if(type != ObjectType::MESH){
        /* Define a box to be attached */
        primitive.type = type;
        switch(type){
            case ObjectType::BOX:
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = dims[0];
                primitive.dimensions[1] = dims[1];
                primitive.dimensions[2] = dims[2];
#ifdef PRINT_DEBUG
                printf("BOX: %f, %f, %f \n", dims[0], dims[1], dims[2]);
#endif
                break;
            case ObjectType::SPHERE:
                primitive.dimensions.resize(1);
                primitive.dimensions[0] = dims[0];
#ifdef PRINT_DEBUG
                printf("SPHERE: %f \n", dims[0]);
#endif
                break;
            case ObjectType::CYLINDER:
                primitive.dimensions.resize(2);
                primitive.dimensions[0] = dims[0];
                primitive.dimensions[1] = dims[1];
#ifdef PRINT_DEBUG
                printf("CYLINDER: %f, %f \n", dims[0], dims[1]);
#endif
                break;
        }
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
        res = planning_scene_->processAttachedCollisionObjectMsg(att_object);
    }
    else {
        object.header.frame_id = link_name;
        /* The id of the object */
        object.id = name;
        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(_pose);
        object.operation = action;
        res = planning_scene_->processCollisionObjectMsg(object);
    }

    return res;
}

bool Planner::add_object(string name, const ObjectType type,
                         CartPose pose, Vec3 dims,
                         string link_name, NameList touch_links, bool attach){
    return process_object(name, type, pose, dims, link_name, touch_links, attach, moveit_msgs::CollisionObject::ADD);
}

bool Planner::add_mesh(string name, const ObjectType type,
                       CartPose pose, Vec3List vertices, Vec3List triangles,
                       string link_name, NameList touch_links, bool attach){
    bool res = false;

    moveit_msgs::AttachedCollisionObject att_object;
    moveit_msgs::CollisionObject object;
    geometry_msgs::Pose _pose;

    /* A default pose */
    _pose.position.x = pose[0];
    _pose.position.y = pose[1];
    _pose.position.z = pose[2];
    _pose.orientation.x = pose[3];
    _pose.orientation.y = pose[4];
    _pose.orientation.z = pose[5];
    _pose.orientation.w = pose[6];

    shape_msgs::Mesh mesh;
    for(auto it_v=vertices.begin(); it_v<vertices.end(); it_v++){
        Vec3 vert = *it_v;
        geometry_msgs::Point pt;
        pt.x = vert[0];
        pt.y = vert[1];
        pt.z = vert[2];
        mesh.vertices.push_back(pt);
    }
    for(auto it_t=triangles.begin(); it_t<triangles.end(); it_t++){
        Vec3 tri = *it_t;
        shape_msgs::MeshTriangle mt;
        mt.vertex_indices[0] = tri[0];
        mt.vertex_indices[1] = tri[1];
        mt.vertex_indices[2] = tri[2];
        mesh.triangles.push_back(mt);
    }

    if (attach) {
        att_object.link_name = link_name;
        /* The header must contain a valid TF frame*/
        att_object.object.header.frame_id = link_name;
        /* The id of the object */
        att_object.object.id = name;
        att_object.object.meshes.push_back(mesh);
        att_object.object.mesh_poses.push_back(_pose);
        att_object.object.operation = moveit_msgs::CollisionObject::ADD;
        att_object.touch_links = touch_links;
        res = planning_scene_->processAttachedCollisionObjectMsg(att_object);
    }
    else {
        object.header.frame_id = link_name;
        /* The id of the object */
        object.id = name;
        object.meshes.push_back(mesh);
        object.mesh_poses.push_back(_pose);
        object.operation = moveit_msgs::CollisionObject::ADD;
        res = planning_scene_->processCollisionObjectMsg(object);
    }

    return res;
}

void Planner::clear_all_objects(){
    planning_scene_->removeAllCollisionObjects();
}

void Planner::terminate(){
    if(planner_instance_){
        planner_instance_->terminate();
    }
}

bool Planner::add_union_manifold(string group_name, string tool_link, CartPose tool_offset,
                                                GeometryList geometry_list, bool fix_surface, bool fix_normal,
                                                double tol){
    manifolds.push_back(std::make_shared<UnionManifold>(robot_model_, group_name,
                                                        tool_link, tool_offset, geometry_list,
                                                        fix_surface, fix_normal, tol));
    return true;
}

bool Planner::clear_manifolds(){
    manifolds.clear();
    return true;
}

JointState& Planner::solve_ik(string group_name, CartPose goal_pose,
                             double timeout_single,
                             bool self_collision, bool fulll_collision){
    auto state_cur = planning_scene_->getCurrentState();

    robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);

    robot_state::RobotStatePtr kinematic_state = std::make_shared<robot_state::RobotState>(robot_model_);
    Eigen::Isometry3d end_effector_goal = Eigen::Translation3d(goal_pose[0], goal_pose[1], goal_pose[2]) * Eigen::Quaterniond(goal_pose[6], goal_pose[3], goal_pose[4], goal_pose[5]);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    std::vector<double> joint_values;
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    kinematic_state->setToRandomPositions();
    result_ik.setZero(joint_names.size());
    bool found_ik;
    bool collision_ok;

    collision_ok = true;
    found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_goal, timeout_single);
    if (found_ik) {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            result_ik[i] = joint_values[i];
        }
        state_cur.setJointGroupPositions(joint_model_group, result_ik.data());
        planning_scene_->setCurrentState(state_cur);
        if(fulll_collision){
            planning_scene_->checkCollision(collision_request, collision_result);
            collision_ok = !collision_result.collision;
        }
        else if(self_collision){
            planning_scene_->checkSelfCollision(collision_request, collision_result);
            collision_ok = !collision_result.collision;
        }
    }

    if (!collision_ok || !found_ik){
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            result_ik[i] = 0;
        }
    }
    return result_ik;
}

void Planner::set_joint_state(JointState values){
    auto state_cur = planning_scene_->getCurrentState();
    state_cur.setVariablePositions(values.data());
    planning_scene_->setCurrentState(state_cur);
}

JointState& Planner::get_joint_state(){
    auto state_cur = planning_scene_->getCurrentState();
    auto joint_values = state_cur.getVariablePositions();
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        current_state[i] = joint_values[i];
    }
    return current_state;
}

bool Planner::check_collision(bool only_self){
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    if(only_self){
        planning_scene_->checkSelfCollision(collision_request, collision_result);
    }
    else{
        planning_scene_->checkCollision(collision_request, collision_result);
    }
    return collision_result.collision;
}

JointState& Planner::get_jacobian(string group_name, JointState Q){
    auto state_cur = planning_scene_->getCurrentState();
    robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
    state_cur.setVariablePositions(Q.data());
    planning_scene_->setCurrentState(state_cur);
    robot_state::RobotStatePtr kinematic_state = std::make_shared<robot_state::RobotState>(robot_model_);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);

    result_jac = JointState((Eigen::Map<Eigen::VectorXd>(jacobian.data(), jacobian.cols()*jacobian.rows())));
    return result_jac;
}