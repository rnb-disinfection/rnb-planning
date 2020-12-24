//
// Created by tamp on 20. 11. 27..
//

//
// Created by tamp on 20. 11. 25..
//

#include <boost/python.hpp>
#include "openGJK_cpp_wrap.h"
#include "bp_container_interface.h"

char const* greet()
{
    return "hello, world";
}

BOOST_PYTHON_MODULE(openGJKlib){
    using namespace boost::python;
    def("greet", greet);

    class_<Point3>("Point3", init<>())
            .def(init<double, double, double>())
            .def("__len__", &Point3::size)
            .def("clear", &Point3::clear)
            .def("append", &std_item<Point3>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<Point3>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<Point3>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<Point3>::del)
            ;

    class_<PointList>("PointList", init<>())
            .def("__len__", &PointList::size)
            .def("clear", &PointList::clear)
            .def("append", &std_item<PointList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<PointList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<PointList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<PointList>::del)
            ;

    def("gjk_cpp", gjk_cpp);
}