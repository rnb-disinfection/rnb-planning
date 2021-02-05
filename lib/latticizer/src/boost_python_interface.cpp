//
// Created by tamp on 20. 11. 25..
//



#include <boost/python.hpp>
#include "bp_container_interface.h"

char const* greet()
{
    return "hello, world";
}

BOOST_PYTHON_MODULE(moveit_interface_py){
    using namespace boost::python;
    def("greet", greet);
}