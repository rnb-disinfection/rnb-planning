//
// Created by tamp on 20. 12. 31..
//

#include "typedef.h"

/**
 * @brief Extract quaternion from RNB::OMPLCompact::CartPose
 * @author Junsu Kang
 */
Eigen::Quaterniond RNB::OMPLCompact::getQuaternion(CartPose pose){
    return Eigen::Quaterniond (pose[6], pose[3],pose[4],pose[5]);
}

Eigen::Affine3d RNB::OMPLCompact::getAffine3d(RNB::OMPLCompact::CartPose pose){
    Eigen::Quaterniond quat;
    quat = RNB::OMPLCompact::getQuaternion(pose);
    return Eigen::Translation3d(pose.block(0,0,3,1))*quat;
}

RNB::OMPLCompact::Geometry::Geometry(ObjectType type, CartPose pose, Vec3 dims){
    this->type = type;
    this->pose = pose;
    this->dims = dims;
    this->tf = getAffine3d(pose);
}

Eigen::MatrixXd RNB::OMPLCompact::getQmat(Eigen::Quaterniond a){
    Eigen::MatrixXd Q(4,4);
    double w = a.w(), x = a.x(), y = a.y(), z = a.z();
    Q <<    w, -x, -y, -z,
            x,  w, -z,  y,
            y,  z,  w, -x,
            z, -y,  x,  w;
    return Q;
}

Eigen::MatrixXd RNB::OMPLCompact::getQhat(Eigen::Quaterniond a){
    Eigen::MatrixXd Q(4,4);
    double w = a.w(), x = a.x(), y = a.y(), z = a.z();
    Q <<    w, -x, -y, -z,
            x,  w,  z, -y,
            y, -z,  w,  x,
            z,  y, -x,  w;
    return Q;
}

Eigen::Matrix3d RNB::OMPLCompact::getHat(Eigen::Vector3d vec){
    Eigen::Matrix3d vec_hat;
    vec_hat <<  0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
    return vec_hat;
}