//
// Created by tamp on 20. 12. 31..
//

#include "typedef.h"

/**
 * @brief Extract quaternion from CartPose
 * @author Junsu Kang
 */
Eigen::Quaterniond RNB::MoveitCompact::getQuaternion(CartPose pose){
    return Eigen::Quaterniond (pose[6], pose[3],pose[4],pose[5]);
}

Eigen::Affine3d RNB::MoveitCompact::getAffine3d(RNB::MoveitCompact::CartPose pose){
    Eigen::Quaterniond quat;
    quat = RNB::MoveitCompact::getQuaternion(pose);
    return Eigen::Translation3d(pose.block(0,0,3,1))*quat;
}

RNB::MoveitCompact::Geometry::Geometry(Shape type, CartPose pose, Vec3 dims){
    this->type = type;
    this->pose = pose;
    this->dims = dims;
    this->tf = getAffine3d(pose);
}

Eigen::MatrixXd RNB::MoveitCompact::getQmat(Eigen::Quaterniond a){
    Eigen::MatrixXd Q(4,4);
    double w = a.w(), x = a.x(), y = a.y(), z = a.z();
    Q <<    w, -x, -y, -z,
            x,  w, -z,  y,
            y,  z,  w, -x,
            z, -y,  x,  w;
    return Q;
}

Eigen::MatrixXd RNB::MoveitCompact::getQhat(Eigen::Quaterniond a){
    Eigen::MatrixXd Q(4,4);
    double w = a.w(), x = a.x(), y = a.y(), z = a.z();
    Q <<    w, -x, -y, -z,
            x,  w,  z, -y,
            y, -z,  w,  x,
            z,  y, -x,  w;
    return Q;
}