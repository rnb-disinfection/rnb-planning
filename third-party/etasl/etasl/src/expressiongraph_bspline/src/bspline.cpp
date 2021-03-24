#include <expressiongraph/bspline.hpp>
#include <iostream>


int binary_search(const Eigen::VectorXd&  x, const double newx) {
    int idx_low = -1;
    int idx_high = x.size();
    while (idx_high-idx_low > 1) {
        int idx = (idx_high+idx_low) / 2;   // postcondition: idx < idx_high 
        if (x[idx] <= newx) {
            idx_low  = idx; 
        } else {
            idx_high = idx; 
        }
    }
    return idx_low;
}

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
)
{
   
    assert( tmp.size() >= degree+1 ); 
    if (ndx+1 < degree) return 0; 
    if (ndx+1 >= knots.size() ) return 0;
    assert(  (cp.size()==knots.size()-degree+1) || ( (degree==0)&&(cp.size()==knots.size()) ) );
    for (int j=0;j<degree+1;++j) {
        tmp[j] = cp[ndx-degree+j+1];
    }
    for (int r=1;r<degree+1;++r) {
        for (int j = degree; j>r-1;--j) {
            double alpha      = (x - knots[j+ndx-degree]) / (knots[j+ndx-r+1] - knots[j+ndx-degree]);
            tmp[j] = (1.0-alpha)*tmp[j-1] + alpha*tmp[j];
        }
    }
    return tmp[degree];
}

double
deBoor(
        double                      x,
        const Eigen::VectorXd&      knots,
        const Eigen::VectorXd&      cp,
        int                         degree,
        Eigen::VectorXd&            tmp
)
{
   int ndx = binary_search(knots, x);
   return deBoor(ndx,x,knots,cp,degree,tmp);
}

/**
 *
 * AANPASSING OM MINDER PADDING TE GEBRUIKEN: 
 *
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
)
{
    assert( tmp.size() >= degree+1 ); 
    if (ndx+1 < degree)           { return 0; }
    if ((ndx+1) >= knots.size() ) { return 0; }
    if (i < ndx-degree+1)         { return 0; }
    if (i > ndx+1 )               { return 0; }
    for (int j=0;j<degree+1;++j) {
        tmp[j] = 0.0;
    }
    tmp[i-ndx+degree-1] = 1.0;
    for (int r=1;r<degree+1;++r) {
        for (int j = degree; j>r-1;--j) {
            double alpha      = (x - knots[j+ndx-degree]) / (knots[j+ndx-r+1] - knots[j+ndx-degree]);
            tmp[j] = (1.0-alpha)*tmp[j-1] + alpha*tmp[j];
        }
    }
    return tmp[degree];
}

double
deBoor_basis(
        double                      x,
        const Eigen::VectorXd&      knots,
        int                         i,
        int                         degree,
        Eigen::VectorXd&            tmp
)
{
    int ndx = binary_search(knots, x);
    return deBoor_basis(ndx,x,knots,i,degree,tmp);
}


bool augknots(const Eigen::VectorXd& knots, int degree, Eigen::VectorXd& aknt) {
    const int nknots = knots.size();
    if (knots.size() <= 1) return false;
    int start=0;
    double el = knots[start];
    for (int i=1;i<nknots;++i) {
        if (knots[i]!=el) {
            start=i-1;
            break;
        }
        el = knots[i];
    }
    el = knots[nknots-1];
    int end = nknots-1;
    for (int i=nknots-2;i>0;--i) {
        if (knots[i]!=el) {
            end = i+1;
            break;
        }
        el = knots[i];
    }
    if (end-start < 1) return false;
    if (degree==0) {
        aknt.resize( degree*2+end-start );
        for ( int i=0;i<aknt.size();++i)  aknt[i]     = knots[i+start+1];
        return true;
    } 
    aknt.resize( degree*2-1+end-start );
    for ( int i=0;i<degree-1;++i)    aknt[i]                    = knots[start];
    for ( int i=start;i<=end;++i)  aknt[i-start+degree-1]     = knots[i];
    for ( int i=end-start+degree;i<aknt.size();++i) aknt[i] = knots[end];
    return true;
}

int number_of_basis_functions( int knots_size,  int degree) {
    return knots_size+degree-1;
}

void derivknots(
        const Eigen::VectorXd& knots, 
        const Eigen::VectorXd& cp, 
        int degree, 
        Eigen::VectorXd& d_knots, 
        Eigen::VectorXd& d_cp) {
    if (degree>1) {
        d_knots.setZero( knots.size()-2);
        d_cp.setZero( cp.size()-1);
        for (int i=0;i<cp.size()-1;++i) {
            d_cp[i] = degree * (cp[i+1]-cp[i])/(knots[i+degree]-knots[i]);
        }
        for (int i=0;i<knots.size()-2;++i) {
            d_knots[i] = knots[i+1];
        }
    } else {
        d_knots.setZero( knots.size()-1);
        d_cp.setZero( cp.size()-1);
        for (int i=0;i<cp.size()-1;++i) {
            d_cp[i] = degree * (cp[i+1]-cp[i])/(knots[i+degree]-knots[i]);
        }
        for (int i=0;i<knots.size()-1;++i) {
            d_knots[i] = knots[i+1];
        }
    }
}
