/**
 * @file bp_container_interface.h
 * @date Feb. 2021
 * @brief Example of boost python module.
 */



#include <boost/python.hpp>
#include "bp_container_interface.h"
#include "library.h"

BOOST_PYTHON_MODULE(boost_python_example){
    using namespace boost::python;
    def("hello", hello);


    class_<Vec3>("Vec3", init<>())
            .def("__getitem__", &std_item<Vec3>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<Vec3>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            ;

    class_<Vec3List>("Vec3List", init<>())
            .def("__len__", &Vec3List::size)
            .def("clear", &Vec3List::clear)
            .def("append", &std_item<Vec3List>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<Vec3List>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<Vec3List>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<Vec3List>::del)
            ;
}