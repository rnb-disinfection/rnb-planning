//
// Created by tamp on 20. 11. 27..
//

#ifndef OPENGJKLIB_OPENGJK_CPP_WRAP_H
#define OPENGJKLIB_OPENGJK_CPP_WRAP_H

#include <vector>
#include "openGJK/openGJK.h"
extern "C" typedef std::vector<PointList> PointListList;

extern "C" typedef std::vector<double> DoubleList;
extern "C" typedef std::vector<DoubleList> DoubleListList;

extern "C" double gjk_cpp(PointList pl1, PointList pl2);

extern "C" double gjk_cpp_min(PointList points, PointListList points_list);

extern "C" DoubleList gjk_cpp_all(PointList points, PointListList points_list);

#endif //OPENGJKLIB_OPENGJK_CPP_WRAP_H
