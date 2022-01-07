#include "latticizer.h"
#include "timer.h"
#include "logger.h"

#include <iostream>

std::string hello(std::string name) {
    std::cout << "Hello, " + name << std::endl;
    return "Hello, " + name;
}

RNB::Latticizer::Latticizer(std::string gjk_lib_path){
    gjk_lib = dlopen(gjk_lib_path.c_str(), RTLD_LAZY);
    if (gjk_lib == NULL){
        std::cout<<"gjk_lib not loaded"<<std::endl;
        std::cout<<dlerror()<<std::endl;
    }
    gjk_fun = (double (*)(PointList, PointList))dlsym(gjk_lib, "gjk_cpp");
    if (gjk_fun == NULL){
        std::cout<<"gjk_cpp not loaded"<<std::endl;
        std::cout<<dlerror()<<std::endl;
    }
}
RNB::Latticizer::~Latticizer(){
    dlclose(gjk_lib);
    std::cout<<"gjk library closed"<<std::endl;
}
void RNB::Latticizer::set_centers(int Nw, int Nd, int Nh, double L_CELL, double x_offset, double y_offset, double z_offset){
    OFFSET_ZERO = Point3(x_offset, y_offset, z_offset);
    this->Nw = Nw;
    this->Nd = Nd;
    this->Nh = Nh;
    this->L_CELL = L_CELL;
    this->Ncells = Nw*Nd*Nh;
    centers = Eigen::MatrixXd(Ncells, 3);
    int Ndh =  Nd*Nh;
    for(int iw=0;iw<Nw;iw++){
        int wbegin = iw*Nd*Nh;
        centers.block(wbegin,0,Ndh,1).fill((iw+0.5)*L_CELL - x_offset);
        for(int id=0;id<Nd;id++){
            int dbegin = id*Nh;
            centers.block(wbegin+dbegin,1,Nh,1).fill((id+0.5)*L_CELL - y_offset);
            for(int ih=0; ih<Nh; ih++){
                centers.block(wbegin+dbegin+ih,2,1,1).fill((ih+0.5)*L_CELL - z_offset);
            }
        }
    }
}

RNB::Point3 RNB::Latticizer::get_center_by_grid(int iw, int id, int ih){
    int wbegin = iw*Nd*Nh;
    int dbegin = id*Nh;
    int idx_cell = wbegin + dbegin + ih;
    return Point3(centers.coeff(idx_cell, 0),
                  centers.coeff(idx_cell, 1),
                  centers.coeff(idx_cell, 2));
}

RNB::Point3 RNB::Latticizer::get_center_by_index(int idx_cell){
    return Point3(centers.coeff(idx_cell, 0),
                  centers.coeff(idx_cell, 1),
                  centers.coeff(idx_cell, 2));
}

void RNB::Latticizer::set_cell_vertices(){
    int rows = centers.rows();
    for(int i=0; i<rows; i++){
        double x_center = centers.coeff(i, 0);
        double y_center = centers.coeff(i, 1);
        double z_center = centers.coeff(i, 2);
        PointList box;
        for(int ix=0; ix<2; ix++){
            for(int iy=0; iy<2; iy++){
                for(int iz=0; iz<2; iz++){
                    box.push_back(Point3((double(ix)-0.5)*L_CELL+x_center,
                                         (double(iy)-0.5)*L_CELL+y_center,
                                         (double(iz)-0.5)*L_CELL+z_center));
                }
            }
        }
        cell_vertices.push_back(box);
    }
}

RNB::PointList RNB::Latticizer::get_vertice_by_grid(int iw, int id, int ih){
    int wbegin = iw*Nd*Nh;
    int dbegin = id*Nh;
    int idx_cell = wbegin + dbegin + ih;
    return cell_vertices[idx_cell];
}

RNB::PointList RNB::Latticizer::get_vertice_by_index(int idx_cell){
    return cell_vertices[idx_cell];
}


RNB::IntList RNB::Latticizer::get_colliding_cells_approx(
        double R11, double R12, double R13,
        double R21, double R22, double R23,
        double R31, double R32, double R33,
        double P1, double P2, double P3,
        int geo_type, Point3 geo_dims){
    Eigen::Matrix3d R_gl;
    R_gl << R11, R12, R13, R21, R22, R23, R31, R32, R33;
    Eigen::Vector3d p_gl(P1, P2, P3);
    Eigen::Vector3d v_tmp;

    // transform cells
    Eigen::MatrixXd centers_gc_abs;
    auto term1 = centers*R_gl.transpose();
    auto term2 = p_gl.transpose();
    centers_gc_abs = (term1.rowwise() + term2).cwiseAbs();
    Eigen::Vector3d dims_hf(geo_dims[0]/2, geo_dims[1]/2, geo_dims[2]/2);

    Eigen::VectorXd dist_list;
    Eigen::VectorXd xy_dist;
    Eigen::VectorXd z_dist;
    switch(geo_type){
        case RNB::GEOTYPE::PLANE:
            dist_list = centers_gc_abs.block(0, 2, centers_gc_abs.rows(), 1);
            break;
        case RNB::GEOTYPE::SPHERE:
            dist_list = centers_gc_abs.rowwise().norm();
            dist_list.array() -= dims_hf[0];
            break;
        case RNB::GEOTYPE::CYLINDER:
            xy_dist = centers_gc_abs.block(0,0,centers_gc_abs.rows(),2).rowwise().norm();
            z_dist = centers_gc_abs.block(0,2,centers_gc_abs.rows(),1);
            xy_dist.array() -= dims_hf[0];
            z_dist.array() -= dims_hf[2];
            xy_dist = xy_dist.cwiseMax(0);
            z_dist = z_dist.cwiseMax(0);
            dist_list = (xy_dist.cwiseProduct(xy_dist)+z_dist.cwiseProduct(z_dist)).cwiseSqrt();
            break;
        case RNB::GEOTYPE::CAPSULE:
            xy_dist = centers_gc_abs.block(0,0,centers_gc_abs.rows(),2).rowwise().norm();
            z_dist = centers_gc_abs.block(0,2,centers_gc_abs.rows(),1);
            z_dist.array() -= dims_hf[2];
            z_dist = z_dist.cwiseMax(0);
            dist_list = (xy_dist.cwiseProduct(xy_dist)+z_dist.cwiseProduct(z_dist)).cwiseSqrt();
            dist_list.array() -= dims_hf[0];
            break;
        case RNB::GEOTYPE::BOX:
            // pass to default - BOX is implemented as default case
        default:
            if (geo_type!=RNB::GEOTYPE::BOX)
                RNB::PRINT_ERROR("UNIMPLEMENTED TYPE - APPROXIMATE WITH BOX");
            dist_list = (centers_gc_abs.rowwise() - dims_hf.transpose()).cwiseMax(0).rowwise().norm();
            break;
    }

    IntList idx_occupy;
    idx_occupy.clear();
    int idx = 0;
    double L_CELL_HF = L_CELL/2;
    for(double* dist_p=dist_list.data(); dist_p!=dist_list.data()+dist_list.size(); dist_p++){
        if((*dist_p)<L_CELL_HF){
            idx_occupy.push_back(idx);
        }
        idx ++;
    }
    return idx_occupy;
}


RNB::IntList RNB::Latticizer::get_colliding_cells(double R11, double R12, double R13,
                                                  double R21, double R22, double R23,
                                                  double R31, double R32, double R33,
                                                  double P1, double P2, double P3,
                                                  PointList vertices, Point3 geo_dims,
                                                  double geo_radius){
    Eigen::Matrix3d orientation_mat;
    orientation_mat << R11, R12, R13, R21, R22, R23, R31, R32, R33;
    Eigen::Vector3d p_geo(P1, P2, P3);
    Eigen::Vector3d v_tmp;

    // transform reference vertices
    for(auto itor=vertices.begin(); itor!=vertices.end(); itor++){
        v_tmp[0] = (*itor)[0];
        v_tmp[1] = (*itor)[1];
        v_tmp[2] = (*itor)[2];
        v_tmp = orientation_mat*v_tmp+p_geo;
        (*itor)[0] = v_tmp[0];
        (*itor)[1] = v_tmp[1];
        (*itor)[2] = v_tmp[2];
    }

    // transform cells
    Eigen::MatrixXd centers_loc;
    auto term1 = centers*orientation_mat;
    auto term2 = (orientation_mat.transpose()*p_geo).transpose();
    centers_loc = (term1.rowwise() - term2).cwiseAbs();
    Eigen::Vector3d dims_hf(geo_dims[0]/2, geo_dims[1]/2, geo_dims[2]/2);

    Eigen::Vector3d L_CELL_vec(L_CELL, L_CELL, L_CELL);
    Eigen::VectorXd dist_list;
    IntList idx_occupy;

    // extract candidates
    Eigen::Vector3d L_CELL_max_vec;
    L_CELL_max_vec.fill(L_CELL * sqrt(3) /2);
    dist_list = (centers_loc.rowwise() - (dims_hf + L_CELL_max_vec).transpose()).rowwise().maxCoeff();
    int idx = 0;
    IntList idx_candi;
    idx_candi.clear();
    for(double* data_p=dist_list.data(); data_p!=dist_list.data()+dist_list.size(); data_p++){
        if((*data_p)<0){
            idx_candi.push_back(idx);
        }
        idx ++;
    }

    // do gjk test on candidates
    idx_occupy.clear();
    for(auto itor_candi=idx_candi.begin(); itor_candi!=idx_candi.end(); itor_candi++){
        double dist = gjk_fun(cell_vertices[(*itor_candi)], vertices) - geo_radius;
        if(dist<1E-3){
            idx_occupy.push_back(*itor_candi);
        }
    }
    return idx_occupy;
}


RNB::IntList RNB::Latticizer::get_colliding_cells_box(double R11, double R12, double R13,
                                                  double R21, double R22, double R23,
                                                  double R31, double R32, double R33,
                                                  double P1, double P2, double P3,
                                                  PointList vertices, Point3 geo_dims,
                                                  double geo_radius){
    Eigen::Matrix3d orientation_mat;
    orientation_mat << R11, R12, R13, R21, R22, R23, R31, R32, R33;
    Eigen::Vector3d p_geo(P1, P2, P3);
    Eigen::Vector3d v_tmp;

    // transform reference vertices
    for(auto itor=vertices.begin(); itor!=vertices.end(); itor++){
        v_tmp[0] = (*itor)[0];
        v_tmp[1] = (*itor)[1];
        v_tmp[2] = (*itor)[2];
        v_tmp = orientation_mat*v_tmp+p_geo;
        (*itor)[0] = v_tmp[0];
        (*itor)[1] = v_tmp[1];
        (*itor)[2] = v_tmp[2];
    }

    // transform cells
    Eigen::MatrixXd centers_loc;
    auto term1 = centers*orientation_mat;
    auto term2 = (orientation_mat.transpose()*p_geo).transpose();
    centers_loc = (term1.rowwise() - term2).cwiseAbs();
    Eigen::Vector3d dims_hf(geo_dims[0]/2, geo_dims[1]/2, geo_dims[2]/2);

    Eigen::Vector3d L_CELL_vec(L_CELL, L_CELL, L_CELL);
    Eigen::VectorXd dist_list;
    IntList idx_occupy;
    if(abs(R11+R22+R33-3)<1E-3){
        dist_list = (centers_loc.rowwise() - (dims_hf + L_CELL_vec).transpose()).rowwise().maxCoeff();
        int idx = 0;
        idx_occupy.clear();
        for(double* data_p=dist_list.data(); data_p!=dist_list.data()+dist_list.size(); data_p++){
            if((*data_p)<0){
                idx_occupy.push_back(idx);
            }
            idx ++;
        }
        return idx_occupy;
    }

    // extract candidates
    Eigen::Vector3d L_CELL_max_vec;
    L_CELL_max_vec.fill(L_CELL * sqrt(3) /2);
    dist_list = (centers_loc.rowwise() - (dims_hf + L_CELL_max_vec).transpose()).rowwise().maxCoeff();
    int idx = 0;
    IntList idx_candi;
    idx_candi.clear();
    for(double* data_p=dist_list.data(); data_p!=dist_list.data()+dist_list.size(); data_p++){
        if((*data_p)<0){
            idx_candi.push_back(idx);
        }
        idx ++;
    }

    // do gjk test on candidates
    idx_occupy.clear();
    for(auto itor_candi=idx_candi.begin(); itor_candi!=idx_candi.end(); itor_candi++){
        double dist = gjk_fun(cell_vertices[(*itor_candi)], vertices) - geo_radius;
        if(dist<1E-3){
            idx_occupy.push_back(*itor_candi);
        }
    }
    return idx_occupy;
}