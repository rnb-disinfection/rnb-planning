#include "moveit_compact.h"

#include <string>
#include <signal.h>
#include <logger.h>
#include <ros_load_yaml.h>
#include "constants.h"

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
    PRINT_FRAMED_LOG("rosnode moveit_plan_compact initialized");
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


bool Planner::init_planner(string& urdf_txt, string& srdf_txt, NameList& group_names, string config_path){
    if(!__ros_initialized){
        __node_handle = init_ros();
    }

    //------------------------parsing with Iterator
    rosparam_load_yaml(__node_handle, "", config_path+"kinematics.yaml");
    rosparam_load_yaml(__node_handle, "", config_path+"ompl_planning.yaml");
    rosparam_load_yaml(__node_handle, "", config_path+"planning_plugin.yaml");

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
    return true;
}

void Planner::configure()
{
    std::string planner;
    if (__node_handle->getParam("planning_plugin", planner))
        planner_plugin_name_ = planner;

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
        ROS_ERROR_STREAM("Exception while loading planner: Only ompl_interface/OMPLPlannerCustom is available now");
//        try
//        {
//            planner_instance_ = planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
//            if (!planner_instance_->initialize(robot_model_, __node_handle->getNamespace()))
//                throw std::runtime_error("Unable to initialize planning plugin");
//            ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
//        }
//        catch (pluginlib::PluginlibException& ex)
//        {
//            ROS_ERROR_STREAM("Exception while loading planner '"
//                                     << planner_plugin_name_ << "': " << ex.what() << std::endl
//                                     << "Available plugins: " << boost::algorithm::join(classes, ", "));
//        }
    }
}

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

    auto state_cur = planning_scene_->getCurrentState();
    state_cur.setVariablePositions(init_state.data());
    planning_scene_->setCurrentState(state_cur);

    plan_result.trajectory.clear();

    planning_interface::PlanningContextPtr context =
            planner_instance_->getPlanningContext(planning_scene_, _req, _res.error_code_);

    context->solve(_res);

#ifdef PRINT_DEBUG
    std::cout <<"init_state: "<<init_state.transpose()<<std::endl;
    std::cout <<"goal_pose: "<<goal_pose.transpose()<<std::endl;
#endif

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
void Planner::clear_context_cache(){
    planner_instance_->resetContextCache();
}

PlanResult& Planner::plan_with_constraints(string group_name, string tool_link,
                         CartPose goal_pose, string goal_link,
                         JointState init_state, string planner_id, double allowed_planning_time, bool allow_approximation){
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

    auto state_cur = planning_scene_->getCurrentState();
    state_cur.setVariablePositions(init_state.data());
    planning_scene_->setCurrentState(state_cur);

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

    planning_interface::PlanningContextPtr context =
            planner_instance_->getPlanningContextConstrained(planning_scene_, _req, _res.error_code_,
                                                             manifold_intersection, allow_approximation);

    context->solve(_res);

#ifdef PRINT_DEBUG
    std::cout <<"init_state: "<<init_state.transpose()<<std::endl;
    std::cout <<"goal_pose: "<<goal_pose.transpose()<<std::endl;
#endif

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

    printf(LOG_FRAME_LINE);
    PRINT_FRAMED_LOG("last pose below");
    cout << (plan_result.trajectory.end()-1)->transpose() << endl;
    printf(LOG_FRAME_LINE "\n");
    plan_result.success = true;
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

bool Planner::process_object(string name, const ObjectType type, CartPose pose, Vec3 dims,
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