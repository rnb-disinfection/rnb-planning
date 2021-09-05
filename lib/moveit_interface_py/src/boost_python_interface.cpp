//
// Created by tamp on 20. 11. 25..
//



#include <boost/python.hpp>
#include "moveit_compact.h"
#include "bp_container_interface.h"

char const* greet()
{
    return "hello, world";
}

using namespace RNB::MoveitCompact;
Planner planner;

BOOST_PYTHON_MODULE(moveit_interface_py){
    using namespace boost::python;
    def("greet", greet);

    enum_<ObjectType>("ObjectType")
            .value("BOX", ObjectType::BOX)
            .value("SPHERE", ObjectType::SPHERE)
            .value("CYLINDER", ObjectType::CYLINDER)
            .value("PLANE", ObjectType::PLANE)
            .export_values()
            ;

    enum_<ompl_interface::ConstrainedSpaceType>("ConstrainedSpaceType")
            .value("PROJECTED", ompl_interface::ConstrainedSpaceType::PROJECTED)
            .value("ATLAS", ompl_interface::ConstrainedSpaceType::ATLAS)
            .value("TANGENTBUNDLE", ompl_interface::ConstrainedSpaceType::TANGENTBUNDLE)
            .export_values()
            ;

    class_<Geometry>("Geometry", init<ObjectType, CartPose, Vec3>())
            .def_readonly("type", &Geometry::type)
            .def_readonly("pose", &Geometry::pose)
            .def_readonly("dims", &Geometry::dims)
            ;

    class_<GeometryList>("GeometryList", init<>())
            .def("__len__", &GeometryList::size)
            .def("clear", &GeometryList::clear)
            .def("append", &std_item<GeometryList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<GeometryList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<GeometryList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<GeometryList>::del)
            ;

//    class_<UnionManifoldPtr>("UnionManifoldPtr")
//            ;
//      ! List of Ptr class is not available in boost-python?
//    class_<UnionManifoldList>("UnionManifoldList", init<>())
//            .def("__len__", &UnionManifoldList::size)
//            .def("clear", &UnionManifoldList::clear)
//            .def("append", &std_item<UnionManifoldList>::add,
//                 with_custodian_and_ward<1,2>()) // to let container keep value
//            .def("__getitem__", &std_item<UnionManifoldList>::get,
//                 return_value_policy<copy_non_const_reference>())
//            .def("__setitem__", &std_item<UnionManifoldList>::set,
//                 with_custodian_and_ward<1,2>()) // to let container keep value
//            .def("__delitem__", &std_item<UnionManifoldList>::del)
//            ;

    class_<NameList>("NameList", init<>())
            .def("__len__", &NameList::size)
            .def("clear", &NameList::clear)
            .def("append", &std_item<NameList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<NameList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<NameList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<NameList>::del)
            ;

    class_<CartPose>("CartPose", init<>())
            .def("__getitem__", &std_item<CartPose>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<CartPose>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            ;

    class_<Vec3>("Vec3", init<>())
            .def("__getitem__", &std_item<Vec3>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<Vec3>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            ;

    class_<JointState>("JointState", init<int>())
            .def("__getitem__", &std_item<JointState>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<JointState>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__len__", &JointState::size)
            ;

    class_<Trajectory>("Trajectory", init<>())
            .def("__len__", &Trajectory::size)
            .def("clear", &Trajectory::clear)
            .def("append", &std_item<Trajectory>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<Trajectory>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<Trajectory>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<Trajectory>::del)
            ;

    class_<PlanResult>("PlanResult")
            .def_readwrite("trajectory", &PlanResult::trajectory)
            .def_readwrite("success", &PlanResult::success)
            ;

    class_<Planner>("Planner")
            .def_readonly("joint_names", &Planner::joint_names)
            .def_readonly("joint_num", &Planner::joint_num)
            .def("init_planner", &Planner::init_planner)
            .def("init_planner_from_file", &Planner::init_planner_from_file)
            .def("plan", &Planner::plan,
                 return_value_policy<copy_non_const_reference>())
            .def("plan_joint_motion", &Planner::plan_joint_motion,
                 return_value_policy<copy_non_const_reference>())
            .def("plan_with_constraints", &Planner::plan_with_constraints,
                 return_value_policy<copy_non_const_reference>())
            .def("test_jacobian", &Planner::test_jacobian)
            .def("validate_trajectory", &Planner::validate_trajectory)
            .def("clear_context_cache", &Planner::clear_context_cache)
            .def("add_union_manifold", &Planner::add_union_manifold)
            .def("clear_manifolds", &Planner::clear_manifolds)
            .def("add_object", &Planner::add_object)
            .def("process_object", &Planner::process_object)
            .def("clear_all_objects", &Planner::clear_all_objects)
            .def("terminate", &Planner::terminate)
            .def("solve_ik", &Planner::solve_ik,
                 return_value_policy<copy_non_const_reference>())
            .def("check_collision", &Planner::check_collision)
            .def("get_jacobian", &Planner::get_jacobian,
                 return_value_policy<copy_non_const_reference>())
            .def("set_tolerance", &Planner::set_tolerance)
            .def("get_tolerance", &Planner::get_tolerance,
                 return_value_policy<copy_non_const_reference>())
            ;
}