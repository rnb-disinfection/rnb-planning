#ifndef EXPRESSIONGRAPHS_BSPLINE_PY_BSPLINE_HPP
#define EXPRESSIONGRAPHS_BSPLINE_PY_BSPLINE_HPP

#include <Eigen/Dense>


double py_bspline_expr( double arg, const std::vector<double>& knots, const std::vector<double>& cp, int degree) {
    Eigen::VectorXd _knots( knots.size() );
    Eigen::VectorXd _cp( cp.size() );
    Eigen::VectorXd tmp(degree+1);
    for (int i=0;i<knots.size();++i) {
        _knots[i] = knots[i];
    }
    for (int i=0;i<cp.size();++i) {
        _cp[i] = cp[i];
    }
    return deBoor( arg, _knots, _cp, degree );
}


double py_bspline_expr( double arg, const std::vector<double>& knots, int j, int degree) {
    Eigen::VectorXd _knots( knots.size() );
    Eigen::VectorXd tmp(degree+1);
    for (int i=0;i<knots.size();++i) {
        _knots[i] = knots[i];
    }
    Eigen::VectorXd _cp = Eigen::VectorXd::Zero( knots.size()-d+1 );
    if ((0<=j)&&(j<cp.size())) {
            _cp[j] = 1.0;
    } 
    return deBoor( arg, _knots, _cp, degree );
}

void py_derivknots(const std::vector<double>& knots, 
                   const std::vector<double>& cp,
                   int degree,
                   std::vector<double>& d_knots,
                   std::vector<double>& d_cp ) {
}


bool augknots(const Eigen::VectorXd& knots, int degree, Eigen::VectorXd& aknt);

void derivknots(
        const Eigen::VectorXd& knots, 
        const Eigen::VectorXd& cp, 
        int degree, 
        Eigen::VectorXd& d_knots, 
        Eigen::VectorXd& d_cp);


#endif

