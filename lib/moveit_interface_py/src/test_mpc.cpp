//
// Created by tamp on 21. 1. 3..
//

#include "moveit_compact.h"

using namespace RNB::MoveitCompact;

int main(int argc, char** argv) {
    NameList group_names;
    group_names.push_back("indy0");
    group_names.push_back("panda1");

    string group_name("indy0");
    string tool_link("indy0_tcp");

    Planner planner;
    planner.init_planner_from_file("../test_assets/custom_robots.urdf", "../test_assets/custom_robots.srdf",
                                   group_names, "../test_assets/");
    JointState init_state(13);
    init_state << 0, 0, -1.57, 0, -1.57, 0, 0, -0.4, 0, -1.57, 0, 1.57, 1.57;

    robot_state::RobotStatePtr kinematic_state = std::make_shared<robot_state::RobotState>(planner.robot_model_);
    robot_state::JointModelGroup* joint_model_group = planner.robot_model_->getJointModelGroup(group_name);
    kinematic_state->setToDefaultValues();
    kinematic_state->setJointGroupPositions(joint_model_group, init_state.data());
    const Eigen::Affine3d &end_effector_tf = kinematic_state->getGlobalLinkTransform(tool_link);

    std::vector<double> joint_values;
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(tool_link);

    /* Set one joint in the right arm outside its joint limit */
    joint_values[0] = 1.57;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 0.1);
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planner.planning_scene_->checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is "
                            << (collision_result.collision ? "in" : "not in")
                            << " self collision");

    Eigen::Vector3d _vec(end_effector_tf.translation());
    Eigen::Quaterniond _rot(end_effector_tf.linear());

    auto goal_tf = end_effector_tf*Eigen::Translation3d(0,0.05,0);

    Eigen::Vector3d _vec_g(goal_tf.translation());
    Eigen::Quaterniond _rot_g(goal_tf.linear());

    CartPose inital_pose;
    CartPose goal_pose;
//    goal << -0.3,-0.2,0.4,0,0,0,1;
    inital_pose << _vec.x(), _vec.y(), _vec.z(), _rot.x(), _rot.y(), _rot.z(), _rot.w();
//    goal_pose << _vec.x(), _vec.y(), _vec.z(), _rot.x(), _rot.y(), _rot.z(), _rot.w();
    goal_pose << _vec_g.x(), _vec_g.y(), _vec_g.z(), _rot_g.x(), _rot_g.y(), _rot_g.z(), _rot_g.w();
//    goal_pose << _vec.x()+0.1, _vec.y()-0.1, _vec.z()-0.1, _rot.x(), _rot.y(), _rot.z(), _rot.w();

    std::cout<<"========== goal ========="<<std::endl;
    std::cout<<goal_pose<<std::endl;
    std::cout<<"========== goal ========="<<std::endl;
    GeometryList geometry_list;
    CartPose tool_offset;
    tool_offset<<0,0,0,0,0,0,1;
    CartPose plane_pose;
    plane_pose << _vec.x(),_vec.y(),_vec.z(), _rot.x(), _rot.y(), _rot.z(), _rot.w();
//    plane_pose << _vec.x(),_vec.y(),_vec.z(),0.70710678,0,0,0.70710678;
//    plane_pose << _vec.x(),_vec.y(),_vec.z(),0.38268343, 0.0, 0.0, 0.92387953;
    geometry_list.push_back(Geometry(ObjectType::PLANE, plane_pose, Vec3(0.5,0.5,0.000001)));
    planner.clear_manifolds();
    planner.add_union_manifold(group_name, tool_link, tool_offset, geometry_list,
                               true, false, 2e-3);

    PlanResult res = planner.plan_with_constraints(group_name, tool_link,
                                                   goal_pose, "base_link", init_state,
                                                   "KPIECE_CUSTOMkConfigDefault",
                                                   10,
                                                   ompl_interface::ConstrainedSpaceType::TANGENTBUNDLE,
                                                   false);

    std::cout<<std::endl;

//    std::cout<<"============== Initial state =============="<<std::endl;
//    std::cout<<init_state.transpose()<<std::endl;
//    std::cout<<"==========================================="<<std::endl<<std::endl;
//
//    std::cout<<"=============== Trajectory ================"<<std::endl;
//    for (auto it=res.trajectory.begin(); it!=res.trajectory.end(); it++){
//        std::cout<<(*it).transpose()<<std::endl;
//    }
//    std::cout<<"==========================================="<<std::endl<<std::endl;

    std::cout<<"============== End Effector ==============="<<std::endl;
//    if (res.trajectory.size()>0){
//        auto it=res.trajectory.begin();
//        kinematic_state->setJointGroupPositions(joint_model_group, it->data());
//        Eigen::Quaterniond quat(kinematic_state->getGlobalLinkTransform(tool_link).linear());
//        std::cout<<kinematic_state->getGlobalLinkTransform(tool_link).translation().transpose()
//                << " " << quat.x() << " "  << quat.y() << " "  << quat.z() << " "  << quat.w() <<std::endl;
//    }
//    if (res.trajectory.size()>1) {
//        auto it = res.trajectory.end() - 1;
//        kinematic_state->setJointGroupPositions(joint_model_group, it->data());
//        Eigen::Quaterniond quat(kinematic_state->getGlobalLinkTransform(tool_link).linear());
//        std::cout << kinematic_state->getGlobalLinkTransform(tool_link).translation().transpose()
//                  << " " << quat.x() << " "  << quat.y() << " "  << quat.z() << " "  << quat.w() <<std::endl;
//    }
    for (auto it=res.trajectory.begin(); it!=res.trajectory.end(); it++){
        kinematic_state->setJointGroupPositions(joint_model_group, it->data());
        Eigen::Quaterniond quat(kinematic_state->getGlobalLinkTransform(tool_link).linear());
        std::cout<<kinematic_state->getGlobalLinkTransform(tool_link).translation().transpose()
                 << " " << quat.x() << " "  << quat.y() << " "  << quat.z() << " "  << quat.w() <<std::endl;
    }
    std::cout<<"==========================================="<<std::endl<<std::endl;

    std::cout<<"============= Initial / Goal =============="<<std::endl;
    std::cout<<inital_pose.transpose()<<std::endl;
    std::cout<<goal_pose.transpose()<<std::endl;
    std::cout<<"==========================================="<<std::endl<<std::endl;
    std::cout<<"========== object_pose ========="<<std::endl;
    std::cout<<plane_pose.transpose()<<std::endl;
    std::cout<<"========== object_pose ========="<<std::endl;


    planner.terminate();

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