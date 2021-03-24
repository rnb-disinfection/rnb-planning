#include <expressiongraph/cubicspline.hpp>
#include <boost/make_shared.hpp>
#include <fstream>

int main() {
    using namespace KDL;
    using namespace std;
    int    n   = 10;
    int    dim = 2;
    double R   = 0.4;
    CubicSpline::Ptr cspline   = cubic_spline();
    cspline->setInput( input(0) );
    int retval =cspline->readPoints("input.csv", " \t",0);
    if (retval!=CubicSpline::RET_SUCCESS) {
        cerr << "could not open file " << endl;
        return -1;
    }
    Expression<double>::Ptr y1 = getSplineOutput(cspline,0);
    Expression<double>::Ptr y2 = getSplineOutput(cspline,1);
    if (!y1 || ! y2) {
        cerr << "error generating the spline " << endl;
        return -1;
    }
    {
        ofstream of("spline3_outp.dat");
        for (double s=cspline->getMinArgument();s<=cspline->getMaxArgument();s+=0.005) {
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

