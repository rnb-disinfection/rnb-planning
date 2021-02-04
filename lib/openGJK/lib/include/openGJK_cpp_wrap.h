//
// Created by tamp on 20. 11. 27..
//

#ifndef OPENGJKLIB_OPENGJK_CPP_WRAP_H
#define OPENGJKLIB_OPENGJK_CPP_WRAP_H

#include <vector>
#include "openGJK/openGJK.h"
typedef std::vector<PointList> PointListList;

typedef std::vector<double> DoubleList;
typedef std::vector<DoubleList> DoubleListList;

double gjk_cpp(PointList pl1, PointList pl2);

double gjk_cpp_min(PointList points, PointListList points_list);

DoubleList gjk_cpp_all(PointList points, PointListList points_list);

#endif //OPENGJKLIB_OPENGJK_CPP_WRAP_H
