//
// Created by rnb on 21. 2. 5..
//

#include "latticizer.h"
#include <iostream>
int main(){
    hello("builder");

    RNB::Latticizer ltc("/home/rnb/Projects/rnb-planning/lib/openGJK/lib/openGJKlib.so");

    std::cout << "test gjk_fun" <<std::endl;
    RNB::Point3 p11(0,0,0);
    RNB::Point3 p12(0,0,1);
    RNB::Point3 p13(0,1,0);
    RNB::Point3 p14(1,0,0);

    RNB::Point3 p21(-1,-1,-1);
    RNB::Point3 p22(0,0,-1);
    RNB::Point3 p23(0,-1,0);
    RNB::Point3 p24(-1,0,0);

    RNB::PointList plist1;
    plist1.push_back(p11);
    plist1.push_back(p12);
    plist1.push_back(p13);
    plist1.push_back(p14);

    RNB::PointList plist2;
    plist2.push_back(p21);
    plist2.push_back(p22);
    plist2.push_back(p23);
    plist2.push_back(p24);
    double x = ltc.gjk_fun(plist1, plist2);
    std::cout<< "distance: " << x <<std::endl;

    std::cout<< "test set_centers" <<std::endl;
    ltc.set_centers(3,3,3,0.1, 0.15, 0.15, 0.1);
    std::cout<< "centers" <<std::endl;
    std::cout<< ltc.centers <<std::endl;

}