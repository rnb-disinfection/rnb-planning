#include <expressiongraph/cubicspline.hpp>
#include <boost/make_shared.hpp>
#include <fstream>

int main() {
    using namespace KDL;
    using namespace std;
    int    n   = 6;
    int    dim = 2;
    double R   = 0.4;
    Eigen::VectorXd x(n);
    Eigen::MatrixXd y(n,dim);
    int i=0;
    {
        ofstream of("spline1_inp.dat");
        for (double th =0, i=0;   i<n;    th+=2*M_PI/(n-1),i++ ) {
            x(i)   = th;
            y(i,0) = R*cos(th);
            y(i,1) = R*sin(th);
            of << 0.5*x(i) << "\t" << y(i,0) << "\t" << y(i,1) << "\n";
        }
    }
    Expression<double>::Ptr s = input(0); 
    CubicSpline::Ptr cspline   = cubic_spline(s, x, y,CubicSpline::PERIODIC);
    cspline->setPolicy( CubicSpline::BISECTION);
    Expression<double>::Ptr y1 = getSplineOutput(cspline,0);
    Expression<double>::Ptr y2 = getSplineOutput(cspline,1);
    {
        ofstream of("spline1_outp.dat");
        for (double v=0;v<=4*M_PI;v+=0.1) {
            y1->setInputValue(0,v);
            y2->setInputValue(0,v);
            of << v << "\t";
            of << y1->value() << "\t";
            of << y2->value() << "\t";
            of << y1->derivative(0) << "\t";
            of << y2->derivative(0) << "\n";
        }
    }
    return 0;
}

