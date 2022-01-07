//
// Created by tamp on 20. 11. 25..
//



#include <boost/python.hpp>
#include "bp_container_interface.h"
#include "latticizer.h"

BOOST_PYTHON_MODULE(latticizer){
    using namespace boost::python;
    def("hello", hello);

    class_<RNB::Point3>("Point3", init<>())
            .def(init<double, double, double>())
            .def("__len__", &RNB::Point3::size)
            .def("clear", &RNB::Point3::clear)
            .def("append", &std_item<RNB::Point3>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<RNB::Point3>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<RNB::Point3>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<RNB::Point3>::del)
            ;

    class_<RNB::PointList>("PointList", init<>())
            .def("__len__", &RNB::PointList::size)
            .def("clear", &RNB::PointList::clear)
            .def("append", &std_item<RNB::PointList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<RNB::PointList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<RNB::PointList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<RNB::PointList>::del)
            ;

    class_<RNB::IntList>("IntList", init<>())
            .def("__len__", &RNB::IntList::size)
            .def("clear", &RNB::IntList::clear)
            .def("append", &std_item<RNB::IntList>::add,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &std_item<RNB::IntList>::get,
                 return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &std_item<RNB::IntList>::set,
                 with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &std_item<RNB::IntList>::del)
            ;

    class_<RNB::Latticizer>("Latticizer", init<std::string>())
            .def_readonly("Ncells", &RNB::Latticizer::Ncells)
            .def_readonly("Nw", &RNB::Latticizer::Nw)
            .def_readonly("Nd", &RNB::Latticizer::Nd)
            .def_readonly("Nh", &RNB::Latticizer::Nh)
            .def_readonly("L_CELL", &RNB::Latticizer::L_CELL)
            .def_readonly("OFFSET_ZERO", &RNB::Latticizer::OFFSET_ZERO)
            .def("set_centers", &RNB::Latticizer::set_centers)
            .def("get_center_by_grid", &RNB::Latticizer::get_center_by_grid)
            .def("get_center_by_index", &RNB::Latticizer::get_center_by_index)
            .def("set_cell_vertices", &RNB::Latticizer::set_cell_vertices)
            .def("get_vertice_by_grid", &RNB::Latticizer::get_vertice_by_grid)
            .def("get_vertice_by_index", &RNB::Latticizer::get_vertice_by_index)
            .def("get_colliding_cells_approx", &RNB::Latticizer::get_colliding_cells_approx)
            .def("get_colliding_cells", &RNB::Latticizer::get_colliding_cells)
            .def("get_colliding_cells_box", &RNB::Latticizer::get_colliding_cells_box)
            ;
}