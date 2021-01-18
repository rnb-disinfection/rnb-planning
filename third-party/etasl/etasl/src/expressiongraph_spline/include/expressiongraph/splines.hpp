#ifndef EXPRESSINGRAPHS_SPLINES_SPLINES_HPP
#define EXPRESSINGRAPHS_SPLINES_SPLINES_HPP

//#define EIGEN_RUNTIME_NO_MALLOC
//        Eigen::internal::set_is_malloc_allowed(false);
#include <cfloat>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

/**
 * solves tri-diagonal system:
 * <code>
 * [b_0 c_0 ....                  [ d_0 ]
 *  a_1 b_1  c_1  ...             [ d_1 ]
 *  0   a_2  b_2  c_2 ... ] * x = [ d_2 ]
 * </code>
 *
 * - a[0] and c[n] are not used.
 * - b and x can be aliased,
 * - b,d,x are changed, x contains the result.
 * - http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
 *
 * \param a [in] lower diagonal
 * \param b [in, changed] main diagonal
 * \param c [in] upper diagonal
 * \param d [in,changed] right hand side
 * \param x [out] solution of linear tridiagonal system.
 */ 
void tridiagonal(const Eigen::VectorXd& a, 
                 Eigen::VectorXd& b,
                 const Eigen::VectorXd& c, 
                 Eigen::VectorXd& d,
                 Eigen::VectorXd& x);


/**
 * same as tridiagonal,with two additional elements to the matrix:
 *    top-right element a(0),   bottom-left element c(n)
 *
 * in contrast to tridiagonal, also a is changed by this routine.
 */
void tridiagonal_periodic(
                 Eigen::VectorXd& a, 
                 Eigen::VectorXd& b, 
                 const Eigen::VectorXd& c, 
                 Eigen::VectorXd& d, 
                 Eigen::VectorXd& x
);


/**
 * uses tridiagonal_periodic to compute ydd (second derivatives) in 
 * each of the points.  These can be used to evaluate the periodic cubic 
 * splines.
 *
 * x(n-1) should be equal to x(0) of the next period.  y(n-1) should be equal to y(0).
 * The period of the periodic spline is x(n-1)-x(0).
 *
 * \param x independent variable
 * \param y dependent variable
 * \param ydd computed 2nd derivative.
 */
 void cubic_spline_periodic(const Eigen::VectorXd& x, 
                           const Eigen::VectorXd& y, 
                           Eigen::VectorXd& ydd);
 

/** 
 * uses tridiagonal to compute the ydd (second derivative) in 
 * each of the points.  These can be used to evaluate the
 * natural splines.  The start and the end of the overall spline
 * has 2nd derivative equal to zero. 
 *
 * \param x independent variable
 * \param y dependent variable
 * \param ydd computed 2nd derivative
 */
void cubic_spline_natural(const Eigen::VectorXd& x, 
                          const Eigen::VectorXd& y, 
                          Eigen::VectorXd& ydd);



/**
 * always assumes xval is inside the range of x.
 * optionally returns the value of the spline, given xval, including
 * its derivatives ydval, 2nd dierivatives, yddval, 3th derivatives ydddval.
 *
 * \param x VectorXd of independent variables
 * \param y VectorXd of dependent variables 
 * \param ydd VectorXd of 2nd derivatives for the dependent variables 
 * \param idx index in the vectors x,y,ydd
 * \param xval value at which you'dd like to compute the interpolation.
 * \param yval will contain value of y corresponding to xval, if not NULL
 * \param ydval will contain value of 1st derivative of y corresponding to xval, if not NULL
 * \param yddval will contain value of 2nd derivative of y corresponding to xval, if not NULL
 * \param ydddval will contain value of 3th deriviative of y corresponding to xval, if not NULL
 */
void cubic_spline_periodic_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& y, 
    const Eigen::VectorXd& ydd, 
    int idx, 
    double xval, 
    double *yval=NULL,
    double *ydval=NULL,
    double *yddval=NULL,
    double *ydddval=NULL);

void cubic_spline_periodic_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int idx, 
    double xval, 
    Eigen::VectorXd* yval, 
    Eigen::VectorXd* ydval=NULL, 
    Eigen::VectorXd* yddval=NULL, 
    Eigen::VectorXd* ydddval=NULL);

/**
 * always assumes xval is inside the range of x.
 * optionally returns the value of the spline, given xval, including
 * its derivatives ydval, 2nd dierivatives, yddval, 3th derivatives ydddval.
 *
 * \param x VectorXd of independent variables
 * \param y MatrixXd of dependent variables 
 * \param ydd MatrixXd of 2nd derivatives for the dependent variables 
 * \param column each column of the matrices represents a spline, this indicates which column to use.(0-based).
 * \param idx index in the vectors x,y,ydd
 * \param xval value at which you'dd like to compute the interpolation.
 * \param yval will contain value of y corresponding to xval, if not NULL
 * \param ydval will contain value of 1st derivative of y corresponding to xval, if not NULL
 * \param yddval will contain value of 2nd derivative of y corresponding to xval, if not NULL
 * \param ydddval will contain value of 3th deriviative of y corresponding to xval, if not NULL
 */
void cubic_spline_periodic_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int column,
    int idx, 
    double xval, 
    double *yval=NULL,
    double *ydval=NULL,
    double *yddval=NULL,
    double *ydddval=NULL);



void cubic_spline_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int idx, 
    double xval, 
    Eigen::VectorXd* yval, 
    Eigen::VectorXd* ydval=NULL, 
    Eigen::VectorXd* yddval=NULL, 
    Eigen::VectorXd* ydddval=NULL
);



/**
 * optionally returns the value of the spline, given xval, including
 * its derivatives ydval, 2nd dierivatives, yddval, 3th derivatives ydddval.
 *
 * If outside of the range of x, the values will be lineary extrapolated,
 * in a way consistent with natural cubic splines.
 *
 * \param x VectorXd of independent variables
 * \param y VectorXd of dependent variables 
 * \param ydd VectorXd of 2nd derivatives for the dependent variables 
 * \param idx index in the vectors x,y,ydd
 * \param xval value at which you'dd like to compute the interpolation.
 * \param yval will contain value of y corresponding to xval, if not NULL
 * \param ydval will contain value of 1st derivative of y corresponding to xval, if not NULL
 * \param yddval will contain value of 2nd derivative of y corresponding to xval, if not NULL
 * \param ydddval will contain value of 3th deriviative of y corresponding to xval, if not NULL
 */

void cubic_spline_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& y, 
    const Eigen::VectorXd& ydd, 
    int idx, 
    double xval, 
    double *yval=NULL,
    double *ydval=NULL,
    double *yddval=NULL,
    double *ydddval=NULL);

/**
 * optionally returns the value of the spline, given xval, including
 * its derivatives ydval, 2nd dierivatives, yddval, 3th derivatives ydddval.
 *
 * If outside of the range of x, the values will be lineary extrapolated,
 * in a way consistent with natural cubic splines.
 *
 * \param x VectorXd of independent variables
 * \param y MatrixXd of dependent variables 
 * \param ydd MatrixXd of 2nd derivatives for the dependent variables 
 * \param column each column of the matrices represents a spline, this indicates which column to use.(0-based).
 * \param idx index in the vectors x,y,ydd
 * \param xval value at which you'dd like to compute the interpolation.
 * \param yval will contain value of y corresponding to xval, if not NULL
 * \param ydval will contain value of 1st derivative of y corresponding to xval, if not NULL
 * \param yddval will contain value of 2nd derivative of y corresponding to xval, if not NULL
 * \param ydddval will contain value of 3th deriviative of y corresponding to xval, if not NULL
 */

void cubic_spline_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int column,
    int idx, 
    double xval, 
    double *yval=NULL,
    double *ydval=NULL,
    double *yddval=NULL,
    double *ydddval=NULL);



/**
 * ASSUMPTIONS:
 * idx_prev should be valid or idx_prev==-1
 * Post conditions during the algorithm:
 *                   (x[idx_low] <= newx)                                         (P5)
 *                   (newx      < x[idx_high])                                    (P6)
 *               (P5 & P6) or ( idx_low==0 & P5) or (idx_high==x.size()) & P6) or
 *                  (idx_low==0 & idx_high==x.size())                              (P7)
 *                (P5 || idx_low==0) and (P6 || idx_high==x.size())                (the same as P7)
 * At the end of the routine:
 * POST CONDITION:      (retval==x.size()-1) & (x[retval] <= newx)                  (P1)
 *                   or (retval==0)          & (newx      < x[1])                   (P2)
 *                   or (x[retval] <= newx)  & (newx      < x[retval+1])            (P3)
 **/
int hunt(const Eigen::VectorXd& x, double newx, int idx_prev);

/**
 * \param start  start of periodic interval in the x array (i.e. x[0])
 * \param end    end of the periodic interval in the y array (i.e. x[x.size()-1])
 * \param value   
 * \returns value that is reduced to the start-end interval.
 */
inline double reduce_periodic(const double& start, const double& end, const double& value) {
    double period = end-start;
    double val = value-start;
    val = val-floor(val/period)*period;
    return val+start;
}

#endif
