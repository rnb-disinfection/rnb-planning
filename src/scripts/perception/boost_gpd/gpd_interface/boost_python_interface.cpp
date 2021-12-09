#include "grasp_detection_interface.h"
#include "bp_container_interface.h"
#include "typedef.h"
#include <boost/python.hpp>


char const* greet()
{
    return "hello, world";
}

//using namespace RNB::MoveitCompact;
//using namespace gpd::RNB;
//gpd::GraspDetector detector(config_filename);

BOOST_PYTHON_MODULE(grasp_interface_py){
    using namespace boost::python;
    def("greet", greet);

    def("getGPD", &getGPD);

    class_<Vec4>("Vec4", init<>())
            .def("__getitem__", &std_item<Vec4>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<Vec4>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            ;

    class_<Vec4List>("Vec4List", init<>())
        .def("__len__", &Vec4List::size)
        .def("clear", &Vec4List::clear)
        .def("append", &std_item<Vec4List>::add,
            with_custodian_and_ward<1,2>()) // to let container keep value
        .def("__getitem__", &std_item<Vec4List>::get,
            return_value_policy<copy_non_const_reference>())
        .def("__setitem__", &std_item<Vec4List>::set,
             with_custodian_and_ward<1,2>()) // to let container keep value
        .def("__delitem__", &std_item<Vec4List>::del)
        ;

    class_<Vec4ListList>("Vec4ListList", init<>())
        .def("__len__", &Vec4ListList::size)
        .def("clear", &Vec4ListList::clear)
        .def("append", &std_item<Vec4ListList>::add,
            with_custodian_and_ward<1,2>()) // to let container keep value
        .def("__getitem__", &std_item<Vec4ListList>::get,
            return_value_policy<copy_non_const_reference>())
        .def("__setitem__", &std_item<Vec4ListList>::set,
            with_custodian_and_ward<1,2>()) // to let container keep value
        .def("__delitem__", &std_item<Vec4ListList>::del)
        ;

    class_<Mat4List>("Mat4List", init<>())
        .def("__len__", &Mat4List::size)
        .def("clear", &Mat4List::clear)
        .def("append", &std_item<Mat4List>::add,
            with_custodian_and_ward<1,2>()) // to let container keep value
        .def("__getitem__", &std_item<Mat4List>::get,
            return_value_policy<copy_non_const_reference>())
        .def("__setitem__", &std_item<Mat4List>::set,
            with_custodian_and_ward<1,2>()) // to let container keep value
        .def("__delitem__", &std_item<Mat4List>::del)
        ;

}
