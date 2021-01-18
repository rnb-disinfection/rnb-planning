//#define EIGEN_RUNTIME_NO_MALLOC
//        Eigen::internal::set_is_malloc_allowed(false);
#include <expressiongraph/splines.hpp>

#include <cfloat>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

/** example from Eigen docs:
template <typename Derived, typename OtherDerived>
void cov(const MatrixBase<Derived>& x, const MatrixBase<Derived>& y, MatrixBase<OtherDerived> const & C)
{
  typedef typename Derived::Scalar Scalar;
  typedef typename internal::plain_row_type<Derived>::type RowVectorType;
  const Scalar num_observations = static_cast<Scalar>(x.rows());
  const RowVectorType x_mean = x.colwise().sum() / num_observations;
  const RowVectorType y_mean = y.colwise().sum() / num_observations;
  const_cast< MatrixBase<OtherDerived>& >(C) =
    (x.rowwise() - x_mean).transpose() * (y.rowwise() - y_mean) / num_observations;
}
*/

/**
 * solves tri-diagonal system:
 * [b_0 c_0 ....                  [ d_0 ]
 *  a_1 b_1  c_1  ...             [ d_1 ]
 *  0   a_2  b_2  c_2 ... ] * x = [ d_2 ]
 *
 * a[0] and c[n] are not used.
 * b and x can be aliased,
 * b,d,x are changed, x contains the result.
 * http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
 */ 

void tridiagonal(const Eigen::VectorXd& a, 
                 Eigen::VectorXd& b,
                 const Eigen::VectorXd& c, 
                 Eigen::VectorXd& d,
                 Eigen::VectorXd& x) {
    using namespace Eigen;

    int n = a.size();
    if (n==0) return;
    for (int i=1;i<n;++i) {
        b[i] = b[i] - a[i]*c[i-1]/b[i-1];
        d[i] = d[i] - a[i]*d[i-1]/b[i-1];
    }
    x[n-1] = d[n-1]/b[n-1];
    int i=n-2;
    while( i>=0) {
        x[i] = ( d[i] - c[i]*x[i+1] ) / b[i];
        --i;
    }
}

/**
 *  principles behind this code:
 *       Conte, S.D., and deBoor, C. (1972), Elementary Numerical Analysis, McGraw-Hill, New York..
 *  optimized to avoid allocation of additional memory.
 */
void tridiagonal_periodic(
                 Eigen::VectorXd& a, 
                 Eigen::VectorXd& b, 
                 const Eigen::VectorXd& c, 
                 Eigen::VectorXd& d, 
                 Eigen::VectorXd& x
){
    using namespace Eigen;
    int n = a.size();
    if (n==0) return;
    if (n<=2) {
        tridiagonal(a,b,c,d,x);
        return;
    }
    double d_n_1 = d[n-1];
    double b_n_1 = b[n-1];
    double a_n_1 = a[n-1];
    double c_n_1 = c[n-1];
    double m;
    for (int i=1;i<n-2;++i) {
        m = a[i]/b[i-1];
        b[i] = b[i]  - m*c[i-1];
        d[i] = d[i]  - m*d[i-1];
        a[i] = - m*a[i-1];
    }
    m = a[n-2]/b[n-3];
    b[n-2] = b[n-2]  - m*c[n-3];
    d[n-2] = d[n-2]  - m*d[n-3];
    a[n-2] = c[n-2] - m*a[n-3];

    x[n-2]  = d[n-2]/b[n-2];
    a[n-2] =  a[n-2]/b[n-2];
    int i=n-3;
    while( i>=0) {
        x[i]  = ( d[i] - c[i]*x[i+1]  ) / b[i];
        a[i] = (  a[i] - c[i]*a[i+1] ) / b[i];
        --i;
    }
    double alpha  = ( d_n_1  - c_n_1*x[0]  - a_n_1*x[n-2] ) /
                    ( b_n_1  - c_n_1*a[0]  - a_n_1*a[n-2] );
    x[n-1]  = 0.0;
    a[n-1] = -1.0;
    x.head(n) -= alpha*a;  
}


// n = size(x)
// y(0) and y(n-1) should be equal.
// there will be n-1 splines 
//  
void cubic_spline_periodic(const Eigen::VectorXd& x, 
                           const Eigen::VectorXd& y, 
                           Eigen::VectorXd& ydd) {
    int n = x.size(); 
    using namespace Eigen;
    using namespace std; 
    VectorXd a(n-1);  
    VectorXd b(n-1);  
    VectorXd c(n-1);  
    VectorXd d(n-1); 
    {
        // at i==0
        double dx1=x(1)-x(0);
        double dx2=x(n-1)-x(n-2);
        d(0) = ( y(1)-y(0))/dx1 -  ( y(0)-y(n-2) )/dx2;
        a(0) = dx2/6.0;
        b(0) = (dx1+dx2)/3.0;
        c(0) = dx1/6.0;
    }
    for (int i=1;i<n-1;++i) {
        double dx1=x(i+1)-x(i);
        double dx2=x(i)-x(i-1);
        d(i) = ( y(i+1)-y(i))/dx1 -  ( y(i)-y(i-1) )/dx2;
        a(i) = dx2/6.0;
        b(i) = (x(i+1)-x(i-1))/3.0;
        c(i) = dx1/6.0;
    }
    ydd.resize(n);
    tridiagonal_periodic(a,b,c,d,ydd);
    ydd[n-1]=ydd[0];
}

// n = size(x)
// y(0) and y(n-1) should be equal.
// there will be n-1 splines 
//  
void cubic_spline_natural(const Eigen::VectorXd& x, 
                           const Eigen::VectorXd& y, 
                           Eigen::VectorXd& ydd) {
    int n = x.size(); 
    using namespace Eigen;
    using namespace std; 
    if (x.size()==2) {
        ydd[0]=0.0;
        ydd[1]=0.0;
        return;
    }
    VectorXd a(n-2);  
    VectorXd b(n-2);  
    VectorXd c(n-2);  
    VectorXd d(n-2); 
    for (int i=1;i<=n-2;++i) {
        double dx1=x(i+1)-x(i);
        double dx2=x(i)-x(i-1);
        d(i-1) = ( y(i+1)-y(i))/dx1 -  ( y(i)-y(i-1) )/dx2;
        a(i-1) = dx2/6.0;
        b(i-1) = (x(i+1)-x(i-1))/3.0;
        c(i-1) = dx1/6.0;
    }
    a(0)   = 0.0;
    c(n-3) = 0.0;
    VectorXd ydd_reduced(n-2);
    tridiagonal(a,b,c,d, ydd_reduced );
    ydd[0]   = 0.0;
    ydd.segment(1,n-2) = ydd_reduced;
    ydd[n-1] = 0.0;
}



void cubic_spline_periodic_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& y, 
    const Eigen::VectorXd& ydd, 
    int idx, 
    double xval, 
    double *yval,
    double *ydval,
    double *yddval,
    double *ydddval) 
{
    double dx = x[idx+1]-x[idx];
    double A = (x[idx+1]-xval)/dx;
    double B = (xval-x[idx])/dx; 
    if (yval!=NULL) {
        double C = A*(A*A-1.0)*dx*dx/6.0;
        double D = B*(B*B-1.0)*dx*dx/6.0;
        *yval = A*y[idx] + B*y[idx+1] + C*ydd[idx] + D*ydd[idx+1];
    }
    if (ydval!=NULL) {
        *ydval = (y[idx+1]-y[idx])/ dx 
                 - (3*A*A-1.0)/6.0*dx*ydd[idx]
                 + (3*B*B-1.0)/6.0*dx*ydd[idx+1];
    }
    if (yddval!=NULL) {
        *yddval = A*ydd[idx] + B*ydd[idx+1];
    }
    if (ydddval!=NULL) {
        *ydddval = ( ydd[idx+1]-ydd[idx] ) / dx;
    }


}

void cubic_spline_periodic_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int column,
    int idx, 
    double xval, 
    double *yval,
    double *ydval,
    double *yddval,
    double *ydddval) 
{
    double dx = x[idx+1]-x[idx];
    double A = (x[idx+1]-xval)/dx;
    double B = (xval-x[idx])/dx; 
    if (yval!=NULL) {
        double C = A*(A*A-1.0)*dx*dx/6.0;
        double D = B*(B*B-1.0)*dx*dx/6.0;
        *yval = A*y(idx,column) + B*y(idx+1,column) + C*ydd(idx,column) + D*ydd(idx+1,column);
    }
    if (ydval!=NULL) {
        *ydval = (y(idx+1,column)-y(idx,column))/ dx 
                 - (3*A*A-1.0)/6.0*dx*ydd(idx,column)
                 + (3*B*B-1.0)/6.0*dx*ydd(idx+1,column);
    }
    if (yddval!=NULL) {
        *yddval = A*ydd(idx,column) + B*ydd(idx+1,column);
    }
    if (ydddval!=NULL) {
        *ydddval = ( ydd(idx+1,column)-ydd(idx,column) ) / dx;
    }
}

void cubic_spline_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& y, 
    const Eigen::VectorXd& ydd, 
    int idx, 
    double xval, 
    double *yval,
    double *ydval,
    double *yddval,
    double *ydddval)
{
    double dx = x[idx+1]-x[idx];
    double A = (x[idx+1]-xval)/dx;
    double B = (xval-x[idx])/dx; 
    if ( xval>x[x.size()-1] ) {
        // outside the range extrapolate linearly
        if (ydval!=NULL) {
            *ydval = (y(idx+1)-y(idx))/ dx + (dx/6.0)*ydd(idx) + (dx/3.0)*ydd(idx+1);
            if (yval!=NULL) {
                *yval = y(idx+1);
                *yval += (*ydval)*(xval-x[idx+1]);
            }
        } else {
            if (yval!=NULL) {
                *yval = y(idx+1) + 
                        ( (y(idx+1)-y(idx))/ dx + (dx/6.0)*ydd(idx) + (dx/3.0)*ydd(idx+1))  * (xval-x[idx+1]);
            }
        }
        if (yddval!=NULL) {
            *yddval = 0.0;
        }
        if (ydddval!=NULL) {
            *ydddval = 0.0;
        }
        return;
    }
    if  (xval < x[0]) {
        // outside the range extrapolate linearly
        if (ydval!=NULL) {
            *ydval = (y(idx+1)-y(idx))/ dx - (dx/3.0 )*ydd(idx) - (dx/6.0)*ydd(idx+1);
            if (yval!=NULL) {
                *yval =  y(idx);
                *yval += (*ydval)*(xval-x[idx]);
            }
        } else {
            if (yval!=NULL) {
                *yval = y(idx) + 
                            ((y(idx+1)-y(idx))/ dx - (dx/3.0 )*ydd(idx) - (dx/6.0)*ydd(idx+1)) * (xval-x[idx]);
            }
        }
        if (yddval!=NULL) {
            *yddval = 0.0;//yddval->rows());
        }
        if (ydddval!=NULL) {
            *ydddval = 0.0;//ydddval->rows());
        }
        return;
    }

    if (yval!=NULL) {
        double C = A*(A*A-1.0)*dx*dx/6.0;
        double D = B*(B*B-1.0)*dx*dx/6.0;
        *yval = A*y[idx] + B*y[idx+1] + C*ydd[idx] + D*ydd[idx+1];
    }
    if (ydval!=NULL) {
        *ydval = (y[idx+1]-y[idx])/ dx 
                 - (3*A*A-1.0)/6.0*dx*ydd[idx]
                 + (3*B*B-1.0)/6.0*dx*ydd[idx+1];
    }
    if (yddval!=NULL) {
        *yddval = A*ydd[idx] + B*ydd[idx+1];
    }
    if (ydddval!=NULL) {
        *ydddval = ( ydd[idx+1]-ydd[idx] ) / dx;
    }

}

void cubic_spline_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int column,
    int idx, 
    double xval, 
    double *yval,
    double *ydval,
    double *yddval,
    double *ydddval)
{
    double dx = x[idx+1]-x[idx];
    double A = (x[idx+1]-xval)/dx;
    double B = (xval-x[idx])/dx; 
    if ( xval>x[x.size()-1] ) {
        // outside the range extrapolate linearly
        if (ydval!=NULL) {
            *ydval = (y(idx+1,column)-y(idx,column))/ dx + (dx/6.0)*ydd(idx,column) + (dx/3.0)*ydd(idx+1,column);
            if (yval!=NULL) {
                *yval = y(idx+1,column);
                *yval += (*ydval)*(xval-x[idx+1]);
            }
        } else {
            if (yval!=NULL) {
                *yval = y(idx+1,column) + 
                        ( (y(idx+1,column)-y(idx,column))/ dx + (dx/6.0)*ydd(idx,column) + (dx/3.0)*ydd(idx+1,column))  * (xval-x[idx+1]);
            }
        }
        if (yddval!=NULL) {
            *yddval = 0.0;
        }
        if (ydddval!=NULL) {
            *ydddval = 0.0;
        }
        return;
    }
    if  (xval < x[0]) {
        // outside the range extrapolate linearly
        if (ydval!=NULL) {
            *ydval = (y(idx+1,column)-y(idx,column))/ dx - (dx/3.0 )*ydd(idx,column) - (dx/6.0)*ydd(idx+1,column);
            if (yval!=NULL) {
                *yval =  y(idx,column);
                *yval += (*ydval)*(xval-x[idx]);
            }
        } else {
            if (yval!=NULL) {
                *yval = y(idx,column) + 
                            ((y(idx+1,column)-y(idx,column))/ dx - (dx/3.0 )*ydd(idx,column) - (dx/6.0)*ydd(idx+1,column)) * (xval-x[idx]);
            }
        }
        if (yddval!=NULL) {
            *yddval = 0.0;//yddval->rows());
        }
        if (ydddval!=NULL) {
            *ydddval = 0.0;//ydddval->rows());
        }
        return;
    }

    if (yval!=NULL) {
        double C = A*(A*A-1.0)*dx*dx/6.0;
        double D = B*(B*B-1.0)*dx*dx/6.0;
        *yval = A*y(idx,column) + B*y(idx+1,column) + C*ydd(idx,column) + D*ydd(idx+1,column);
    }
    if (ydval!=NULL) {
        *ydval = (y(idx+1,column)-y(idx,column))/ dx 
                 - (3*A*A-1.0)/6.0*dx*ydd(idx,column)
                 + (3*B*B-1.0)/6.0*dx*ydd(idx+1,column);
    }
    if (yddval!=NULL) {
        *yddval = A*ydd(idx,column) + B*ydd(idx+1,column);
    }
    if (ydddval!=NULL) {
        *ydddval = ( ydd(idx+1,column)-ydd(idx,column) ) / dx;
    }
}

void cubic_spline_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int idx, 
    double xval, 
    Eigen::VectorXd* yval, 
    Eigen::VectorXd* ydval, 
    Eigen::VectorXd* yddval, 
    Eigen::VectorXd* ydddval
)
{
    double dx = x[idx+1]-x[idx];
    double A = (x[idx+1]-xval)/dx;
    double B = (xval-x[idx])/dx; 
    if ( xval>x[x.size()-1] ) {
        // outside the range extrapolate linearly
        if (ydval!=NULL) {
            ydval->noalias() = (y.row(idx+1)-y.row(idx))/ dx + (dx/6.0)*ydd.row(idx) + (dx/3.0)*ydd.row(idx+1);
            if (yval!=NULL) {
                yval->noalias() = y.row(idx+1);
                yval->noalias() += (*ydval)*(xval-x[idx+1]);
            }
        } else {
            if (yval!=NULL) {
                yval->noalias() = y.row(idx+1) + 
                        ( (y.row(idx+1)-y.row(idx))/ dx + (dx/6.0)*ydd.row(idx) + (dx/3.0)*ydd.row(idx+1))  * (xval-x[idx+1]);
            }
        }
        if (yddval!=NULL) {
            yddval->setZero();//yddval->rows());
        }
        if (ydddval!=NULL) {
            ydddval->setZero();//ydddval->rows());
        }
        return;
    }
    if  (xval < x[0]) {
        // outside the range extrapolate linearly
        if (ydval!=NULL) {
            ydval->noalias() = (y.row(idx+1)-y.row(idx))/ dx - (dx/3.0 )*ydd.row(idx) - (dx/6.0)*ydd.row(idx+1);
            if (yval!=NULL) {
                yval->noalias() = y.row(idx);
                yval->noalias() += (*ydval)*(xval-x[idx]);
            }
        } else {
            if (yval!=NULL) {
                yval->noalias() = y.row(idx) + 
                            ((y.row(idx+1)-y.row(idx))/ dx - (dx/3.0 )*ydd.row(idx) - (dx/6.0)*ydd.row(idx+1)) * (xval-x[idx]);
            }
        }
        if (yddval!=NULL) {
            yddval->setZero();//yddval->rows());
        }
        if (ydddval!=NULL) {
            ydddval->setZero();//ydddval->rows());
        }
        return;
    }


    if (yval!=NULL) {
        double C = A*(A*A-1.0)*dx*dx/6.0;
        double D = B*(B*B-1.0)*dx*dx/6.0;
        yval->noalias() = A*y.row(idx) + B*y.row(idx+1) + C*ydd.row(idx) + D*ydd.row(idx+1);
    }
    if (ydval!=NULL) {
        ydval->noalias() = (y.row(idx+1)-y.row(idx))/ dx 
                           - ((3*A*A-1.0)/6.0*dx)*ydd.row(idx)
                           + ((3*B*B-1.0)/6.0*dx)*ydd.row(idx+1);
    }
    if (yddval!=NULL) {
        yddval->noalias()  = A*ydd.row(idx) + B*ydd.row(idx+1);
    }
    if (ydddval!=NULL) {
        ydddval->noalias() = ( ydd.row(idx+1)-ydd.row(idx) ) / dx;
    }
}
void cubic_spline_periodic_evaluate(
    const Eigen::VectorXd& x, 
    const Eigen::MatrixXd& y, 
    const Eigen::MatrixXd& ydd, 
    int idx, 
    double xval, 
    Eigen::VectorXd* yval, 
    Eigen::VectorXd* ydval, 
    Eigen::VectorXd* yddval, 
    Eigen::VectorXd* ydddval)
{
    double dx = x[idx+1]-x[idx];
    double A = (x[idx+1]-xval)/dx;
    double B = (xval-x[idx])/dx; 
    if (yval!=NULL) {
        double C = A*(A*A-1.0)*dx*dx/6.0;
        double D = B*(B*B-1.0)*dx*dx/6.0;
        yval->noalias() = A*y.row(idx) + B*y.row(idx+1) + C*ydd.row(idx) + D*ydd.row(idx+1);
    }
    if (ydval!=NULL) {
        ydval->noalias() = (y.row(idx+1)-y.row(idx))/ dx 
                           - ((3*A*A-1.0)/6.0*dx)*ydd.row(idx)
                           + ((3*B*B-1.0)/6.0*dx)*ydd.row(idx+1);
    }
    if (yddval!=NULL) {
        yddval->noalias()  = A*ydd.row(idx) + B*ydd.row(idx+1);
    }
    if (ydddval!=NULL) {
        ydddval->noalias() = ( ydd.row(idx+1)-ydd.row(idx) ) / dx;
    }
}


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
int hunt(const Eigen::VectorXd& x, double newx, int idx_prev) {
    /*int n = x.size();
    double period = x[n-1]-x[0]; 
    double intpart,fracpart;
    std::modf(newx-x[0],&intpart)*/
    int inc     = 1;
    int idx_low = idx_prev;
    int idx_high;
    if (idx_prev!=-1) {
        if (x[idx_low] <= newx ) { 
            // post cond P5 is valid
            if (idx_low >= x.size()-2) {
                return idx_low;  // postcond. P1
            }
            idx_high = idx_low + inc;
            while (x[idx_high] <= newx ) {  // while not P6
                idx_low =  idx_high; // postcond. P5 is valid
                inc     += inc;
                idx_high =  idx_low+inc;
                if (idx_high >= x.size()-1) {
                    idx_high = x.size()-1;
                    break;
                }
            }
            // post cond. P7 valid
        } else {
            if (idx_low == 0) {
                return idx_low; // postcondition P2
            }
            idx_high = idx_low;
            // post cond. P6 is valid
            idx_low  = idx_low - inc;
            while (newx < x[idx_low] ) {  // while not P5
                idx_high =  idx_low;
                inc      += inc;
                idx_low  = idx_high-inc;
                if (idx_low <= 0) {
                    idx_low = 0;  
                    break;
                } 
            }
            // post cond. P7 valid
        }
    } else {
        idx_low = 0;
        idx_high = x.size()-1;
        // post cond. P7 valid
    }
    while (idx_high-idx_low > 1) {
        // idx_high can only (scrictly) decrease, idx_low can only increase
        // pre cond: P7 is valid
        int idx = (idx_high+idx_low) / 2;
        if (x[idx] <= newx) {
            idx_low  = idx; 
        } else {
            idx_high = idx; 
        }
        // postcond: idx_low <= idx < idx_high,   P7
    }
    // postcond: P7 & (idx_high-idx_low==1)  ==> P3 with retval==idx_low
    return idx_low;
}

