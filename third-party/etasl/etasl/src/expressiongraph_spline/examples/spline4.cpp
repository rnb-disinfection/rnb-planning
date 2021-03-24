// before running this example, use generate_input2.py to generate the appropriate input.

#include <expressiongraph/cubicspline.hpp>
#include <boost/make_shared.hpp>
#include <fstream>
#include <boost/timer.hpp>
#include <cmath>

int main() {
    using namespace KDL;
    using namespace std;
    int    n   = 10;
    int    dim = 2;
    double R   = 0.4;
    CubicSpline::Ptr cspline   = cubic_spline();
    cspline->setInput( input(0) );
    Expression<double>::Ptr y1;
    Expression<double>::Ptr y2;
    { 
        boost::timer timer;
        cerr << "Reading the spline data " << endl;
        int retval =cspline->readPoints("input2.csv", " \t",0);
        y1 = getSplineOutput(cspline,0);
        y2 = getSplineOutput(cspline,1);
        if (!y1 || ! y2) {
            cerr << "error generating the spline " << endl;
            return -1;
        }
        cerr << "number of points in spline " << cspline->getNrOfRows() << endl;
        cerr << "time : " << timer.elapsed() << " secs" << endl;
    }
    {
        boost::timer timer;
        double n    = 100000.0;
        double start=cspline->getMinArgument();
        double end  =cspline->getMaxArgument(); 
        double step = (end-start)/n ;
        end = end - step/2.0;
        int    n2   = 1;
        double sum=0; // it is difficult to avoid optimization of the loop below :
        cerr << "from " << start << " to " << end << " with steps of " << step << endl;
        cerr << "Interpolating with range with " << n << " points for " << n2 << " times" << endl;
        for (int i=0;i<n2;++i) {
            for (double s=start; s<end; s+=step) { 
                y1->setInputValue(0,s);
                y2->setInputValue(0,s);
                sum += y1->value();
                sum += y2->value();
                sum += y1->derivative(0);
                sum += y2->derivative(0);
            }
        }
        cerr << "total sum " << sum << endl;  // without this everything is optimized away !
        cerr << "time to evaluate " << n << " points " << n2 << " times: " << timer.elapsed() << " secs" << endl;
    }
    {
        boost::timer timer;
        cerr << "output to file " << endl;
        ofstream of("spline4_outp.dat");
        Eigen::VectorXd x(3);
        for (double s=cspline->getMinArgument();s<=cspline->getMaxArgument();s+=0.005) {
            y1->setInputValue(0,s);
            y2->setInputValue(0,s);
            of << s << "\t";
            of << y1->value() << "\t";
            of << y2->value() << "\t";
            of << y1->derivative(0) << "\t";
            of << y2->derivative(0) << "\n";
        }
        cerr << "time writint to file: " << timer.elapsed() << " secs" << endl;
    }

    return 0;
}

