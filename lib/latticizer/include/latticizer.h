#ifndef LATTICIZER_LATTICIZER_H
#define LATTICIZER_LATTICIZER_H

#include <eigen3/Eigen/Eigen>
#include <string>
#include <dlfcn.h>

std::string hello(std::string name);


namespace RNB{
    /**
     * @class GEOTYPE
     * @brief Geometry type enumeration
     */
    enum GEOTYPE{
        SPHERE = 0,
        CAPSULE = 1,
        BOX = 2,
        MESH = 3,
        ARROW = 4,
        CYLINDER = 5,
        PLANE = 6
    };

    class Point3: public std::vector<double> {
    public:
        Point3(): std::vector<double>(){}
        Point3(double x, double y, double z): std::vector<double>(){
            this->push_back(x);
            this->push_back(y);
            this->push_back(z);
        }
    };

    typedef std::vector<Point3> PointList;
    typedef std::vector<PointList> PointListList;

    typedef std::vector<int> IntList;

    /**
     * @class Latticizer
     * @brief Latticizer
     */
    class Latticizer{
        void *gjk_lib;
    public:
        int Ncells, Nw, Nd, Nh;
        double L_CELL;
        Point3 OFFSET_ZERO;
        Eigen::MatrixXd centers;
        PointListList cell_vertices;

        /**
         * @param gjk_lib_path full path to gjk library .so file
         */
        Latticizer(std::string gjk_lib_path);

        ~Latticizer();

        /**
         * @brief set lattice centers
         */
        void set_centers(int Nw, int Nd, int Nh, double L_CELL, double x_offset, double y_offset, double z_offset);

        Point3 get_center_by_grid(int iw, int id, int ih);

        Point3 get_center_by_index(int idx_cell);

        /**
         * @brief set lattice vertices
         */
        void set_cell_vertices();

        PointList get_vertice_by_grid(int iw, int id, int ih);

        PointList get_vertice_by_index(int idx_cell);

        /**
         * @brief get cell indices that collides with given geometry
         * @param R11 R11~R33 is orientation matrix of the lattice from the geometry
         * @param P1 P1~P3 is position of the lattice from the geometry
         * @param geo_type GEOTYPE
         * @param geo_dims axial dimensions of geometry.
         */
        IntList get_colliding_cells_approx(double R11, double R12, double R13,
                                           double R21, double R22, double R23,
                                           double R31, double R32, double R33,
                                           double P1, double P2, double P3,
                                           int geo_type, Point3 geo_dims);

        /**
         * @brief get cell indices that collides with given geometry
         * @param R11 R11~R33 is orientation matrix of the geometry
         * @param P1 P1~P3 is Position of the geometry
         * @param vertices List of vertices of the geometry at the origin. transformation is done in this function.
         * @param geo_dims axial dimensions of geometry.
         */
        IntList get_colliding_cells(double R11, double R12, double R13,
                                    double R21, double R22, double R23,
                                    double R31, double R32, double R33,
                                    double P1, double P2, double P3,
                                    PointList vertices, Point3 geo_dims,
                                    double geo_radius);

        /**
         * @brief get cell indices that collides with given geometry
         * @remark special case for box, this is added because boost-python supports only 15 arguments (no room for box flag)
         * @param R11 R11~R33 is orientation matrix of the geometry
         * @param P1 P1~P3 is Position of the geometry
         * @param vertices List of vertices of the geometry at the origin. transformation is done in this function.
         * @param geo_dims axial dimensions of geometry.
         */
        IntList get_colliding_cells_box(double R11, double R12, double R13,
                                    double R21, double R22, double R23,
                                    double R31, double R32, double R33,
                                    double P1, double P2, double P3,
                                    PointList vertices, Point3 geo_dims,
                                    double geo_radius);

        double (*gjk_fun)(PointList pl1, PointList pl2); /**< pointer for gjk distance function */
    };
}

#endif //LATTICIZER_LATTICIZER_H
