//
// Created by tamp on 20. 11. 27..
//
#include "bp_container_interface.h"
#include "openGJK_cpp_wrap.h"


double gjk_cpp(PointList pl1, PointList pl2){
    struct bd bd1, bd2;
    struct simplex s;
    bd1.numpoints = pl1.size();
    bd1.coord = pl1;
    bd2.numpoints = pl2.size();
    bd2.coord = pl2;

    return gjk(bd1, bd2, &s);
}