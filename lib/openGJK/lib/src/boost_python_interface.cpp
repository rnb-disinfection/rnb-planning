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

    class_<PointListList>("PointListList", init<>())
            .def("__len__", &PointListList::size)
            .def("clear", &PointListList::clear)
            .def("append", &std_item<PointListList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<PointListList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<PointListList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<PointListList>::del)
            ;

    class_<DoubleList>("DoubleList", init<>())
            .def("__len__", &DoubleList::size)
            .def("clear", &DoubleList::clear)
            .def("append", &std_item<DoubleList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<DoubleList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<DoubleList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<DoubleList>::del)
            ;

    class_<DoubleListList>("DoubleListList", init<>())
            .def("__len__", &DoubleListList::size)
            .def("clear", &DoubleListList::clear)
            .def("append", &std_item<DoubleListList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<DoubleListList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<DoubleListList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<DoubleListList>::del)
            ;

    def("gjk_cpp", gjk_cpp);

    def("gjk_cpp_min", gjk_cpp_min);

    def("gjk_cpp_all", gjk_cpp_all);
}