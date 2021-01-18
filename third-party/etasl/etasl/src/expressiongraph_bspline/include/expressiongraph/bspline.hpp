#ifndef EXPRESSIONGRAPHS_BSPLINE_BSPLINE_HPP
#define EXPRESSIONGRAPHS_BSPLINE_BSPLINE_HPP

#include <Eigen/Dense>

/**
 *  B-spline library 
 *
 *  a knot vector for a b-spline with degree D should be padded, i.e. should contain D identical elements at the start,
 *  and also D identical elements at the end.
 *
 *  The padding is minimal (in constrast to some other libraries): 
 *    #control_points = #knots - D + 1
 *
 *  Sometimes you have to give temporary storage to the routines.
 *  The routines do not allocate/deallocate memory internally.
 *
 *  Derivatives are computed by transforming the knot-vector and the control points, and then
 *  applying the regular spline evaluation routines.
 *
 * (c) E. Aertbelien, not for distribution outside KULeuven.
 */

/**
 * returns an index i such that x[i] <= x < x[i+1]
 */
int binary_search(const Eigen::VectorXd&  x, const double newx);


/**
 * Compute B-spline
 *
 * Parameters:
 *  ndx: index of the knot interval that contains x
 *  x  : position to be evaluated
 *  knots        : knot positions [N] (first "degree" elements should be equal, 
 *                                  last "degree" elements also )
 *                 multiplicity of the knots should be < degree+1
 *  cp           : control points (double[N-degree-1])
 *  degree       : degree of the B-spline
 *  tmp          : temporary storage [ minimum size:  (degree+1) ] 
 * Returns:
 *  the value of the B-spline basis. 
 *
 */
double
deBoor(
        int                         ndx,
        double                      x,
        const Eigen::VectorXd&      knots,
        const Eigen::VectorXd&      cp,
        int                         degree,
        Eigen::VectorXd&            tmp
);

double
deBoor(
        double                      x,
        const Eigen::VectorXd&      knots,
        const Eigen::VectorXd&      cp,
        int                         degree,
        Eigen::VectorXd&            tmp
);

/**
 * Compute B-spline
 *
 * Parameters:
 *  ndx: index of the knot interval that contains x
 *  x  : position to be evaluated
 *  knots        : knot positions [N] (first "degree" elements should be equal, 
 *                                  last "degree" elements also )
 *                 multiplicity of the knots should be < degree+1
 *  knots_size   : number of knot positions [==N]
 *  i            : i-th basis function for the B-spline
 *                 0<=i<(N-degree-1) , otherwise always zero
 *  degree       : degree of the B-spline
 *  tmp          : temporary storage [ minimum size:  (degree+1) ] 
 * Returns:
 *  the value of the B-spline basis. 
 *
 */
double
deBoor_basis(
        int                         ndx,
        double                      x,
        const Eigen::VectorXd&      knots,
        int                         i,
        int                         degree,
        Eigen::VectorXd&            tmp
);


double
deBoor_basis(
        double                      x,
        const Eigen::VectorXd&      knots,
        int                         i,
        int                         degree,
        Eigen::VectorXd&            tmp
);

bool augknots(const Eigen::VectorXd& knots, int degree, Eigen::VectorXd& aknt);


void derivknots(
        const Eigen::VectorXd& knots, 
        const Eigen::VectorXd& cp, 
        int degree, 
        Eigen::VectorXd& d_knots, 
        Eigen::VectorXd& d_cp);


#endif

