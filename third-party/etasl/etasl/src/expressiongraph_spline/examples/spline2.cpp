#include <expressiongraph/cubicspline.hpp>
#include <boost/make_shared.hpp>
#include <fstream>

int main() {
    using namespace KDL;
    using namespace std;
    int    n   = 10;
    int    dim = 2;
    double R   = 0.4;
    Eigen::VectorXd x(n);
    Eigen::MatrixXd y(n,dim);
    int i=0;
    {
        ofstream of("spline2_inp.dat");
        for (double th =0, i=0;   i<n;    th+=2*M_PI/(n-1),i++ ) {
            x(i)   = th*th/7.0;
            y(i,0) = R*cos(x(i));
            y(i,1) = R*sin(x(i));
            of << x(i) << "\t" << y(i,0) << "\t" << y(i,1) << "\n";
        }
    }
    Expression<double>::Ptr s = input(0); 
    CubicSpline::Ptr cspline = cubic_spline(s, x, y);
    Expression<double>::Ptr y1 = getSplineOutput(cspline, 0);
    Expression<double>::Ptr y2 = getSplineOutput(cspline, 1);
    {
        ofstream of("spline2_outp.dat");
        for (double s=0;s<=M_PI;s+=0.005) {
            y1->setInputValue(0,2*s);
            y2->setInputValue(0,2*s);
            of << s << "\t";
            of << y1->value() << "\t";
            of << y2->value() << "\t";
            of << y1->derivative(0) << "\t";
            of << y2->derivative(0) << "\n";
        }
    }

    cspline->normalize(200);
     {
        ofstream of("spline2_norm.dat");
        for (double s=0;s<=M_PI;s+=0.05) {
            y1->setInputValue(0,s);
            y2->setInputValue(0,s);
            of << s << "\t";
            of << y1->value() << "\t";
            of << y2->value() << "\t";
            of << y1->derivative(0) << "\t";
            of << y2->derivative(0) << "\n";
        }
    }

   
    return 0;
}

