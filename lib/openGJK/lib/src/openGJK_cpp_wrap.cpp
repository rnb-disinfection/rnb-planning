//
// Created by tamp on 20. 11. 27..
//
#include "bp_container_interface.h"
#include "openGJK_cpp_wrap.h"


double gjk_cpp(PointList pl1, PointList pl2){
    struct bd bd1, bd2;
    struct simplex s;
    s.clear();
    bd1.numpoints = pl1.size();
    bd1.coord = pl1;
    bd2.numpoints = pl2.size();
    bd2.coord = pl2;

    return gjk(bd1, bd2, &s);
}

double gjk_cpp_min(PointList points, PointListList points_list){
    int len;
    len = points_list.size();
    double val_tmp;
    double val_min = 10000;

    for(int i=0; i<len;i++){
        val_tmp = gjk_cpp(points, points_list[i]);
        val_min = val_tmp < val_min ? val_tmp : val_min;
    }

    return val_min;
}

DoubleList gjk_cpp_all(PointList points, PointListList points_list){
    int len;
    len = points_list.size();
    DoubleList dist_list;
    dist_list.clear();
    for(int i=0; i<len;i++){
        dist_list.push_back(gjk_cpp(points, points_list[i]));
    }

    return dist_list;
}