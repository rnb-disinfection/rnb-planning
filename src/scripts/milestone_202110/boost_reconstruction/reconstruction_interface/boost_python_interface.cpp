#include <string>

#include "reconstruction_interface.h"
#include <boost/python.hpp>

char const* greet()
{
    return "hello, world";
}

//using namespace RNB::MoveitCompact;
//using namespace gpd::RNB;
//using namespace open3d;
//gpd::GraspDetector detector(config_filename);

BOOST_PYTHON_MODULE(reconstruction_interface_py){
    using namespace boost::python;
    def("greet", greet);

    def("getReconstruction", reconstruction);

}
