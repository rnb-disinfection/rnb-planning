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

BOOST_PYTHON_MODULE(moveit_plan_compact){
    using namespace boost::python;
    def("greet", greet);

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
            .def("add_object", &Planner::add_object)
            .def("process_object", &Planner::process_object)
            .def("clear_all_objects", &Planner::clear_all_objects)
            .def("terminate", &Planner::terminate)
            ;
}