#include <expressiongraph/cubicspline.hpp>
#include <boost/make_shared.hpp>
#include <fstream>

int main() {
    using namespace KDL;
    using namespace std;
    int    n   = 20;
    int    dim = 2;
    double R   = 0.4;
    Eigen::VectorXd x(n);
    Eigen::MatrixXd y(n,dim);
    int i=0;
    {
        ofstream of("spline5_inp.dat");
        double f=0.1;
        for (double th =0, i=0;   i<n;    th+=2*M_PI/(n-1),i++ ) {
            x(i)   = th;
            y(i,0) = R*cos(x(i));
            y(i,1) = R*sin(x(i));
            of << 0.5*x(i) << "\t" << y(i,0) << "\t" << y(i,1) << "\n";
        }
    }
    Expression<double>::Ptr s     = Constant(2.0)*input(0)+input(0)*input(0)/Constant(4.0); 
    CubicSpline::Ptr cspline      = cubic_spline(s, x, y,CubicSpline::PERIODIC);
    cspline->setPolicy( CubicSpline::BISECTION);
    Expression<double>::Ptr y1    = getSplineOutput(cspline,0);
    Expression<double>::Ptr y2    = getSplineOutput(cspline,1);
    Expression<double>::Ptr y1d   = y1->derivativeExpression(0);
    Expression<double>::Ptr y2d   = y2->derivativeExpression(0);
    Expression<double>::Ptr y1dd  = y1d->derivativeExpression(0);
    Expression<double>::Ptr y2dd  = y2d->derivativeExpression(0);
    Expression<double>::Ptr y1ddd = y1dd->derivativeExpression(0);
    Expression<double>::Ptr y2ddd = y2dd->derivativeExpression(0);
    cout << *cspline << endl;
    s->setInputValue(0,0.1);
    cout << "input der ";
    s->derivativeExpression(0)->print(cout);
    cout << "\n y1d symbolic ";
    y1d->print(cout);
    cout << endl;
    cout << "x y1 y2 y1d y2d y1d y2d y1dd y2dd y1dd y2dd y1ddd y2ddd y1ddd y2ddd y1dddd y2dddd" << endl;
    cout << "0  1  2   3  4    5   6   7     8    9   10    11    12    13    14     15     16\n" << endl;
    {
        ofstream of("spline5_outp.dat");
        for (double v=0;v<=2*M_PI;v+=0.01) {
            y1->setInputValue(0,v);
            y2->setInputValue(0,v);
            y1d->setInputValue(0,v);
            y2d->setInputValue(0,v);
            y1dd->setInputValue(0,v);
            y2dd->setInputValue(0,v);
            y1ddd->setInputValue(0,v);
            y2ddd->setInputValue(0,v);

            of << v << "\t";
            of << y1->value() << "\t";
            of << y2->value() << "\t";
            of << y1->derivative(0) << "\t";
            of << y2->derivative(0) << "\t";
            of << y1d->value() << "\t";
            of << y2d->value() << "\t";
            of << y1d->derivative(0) << "\t";
            of << y2d->derivative(0) << "\t";
            of << y1dd->value() << "\t";
            of << y2dd->value() << "\t";
            of << y1dd->derivative(0) << "\t";
            of << y2dd->derivative(0) << "\t";
            of << y1ddd->value() << "\t";
            of << y2ddd->value() << "\t";
            of << y1ddd->derivative(0) << "\t";
            of << y2ddd->derivative(0) << "\n";
 
        }
    }
   {
        cspline->normalize(100);
        y1    = getSplineOutput(cspline,0);
        y2    = getSplineOutput(cspline,1);
        y1d   = y1->derivativeExpression(0);
        y2d   = y2->derivativeExpression(0);
        y1dd  = y1d->derivativeExpression(0);
        y2dd  = y2d->derivativeExpression(0);
        y1ddd = y1dd->derivativeExpression(0);
        y2ddd = y2dd->derivativeExpression(0);
     
        ofstream of("spline5_norm_outp.dat");
        for (double v=0;v<=4;v+=0.01) {
            y1->setInputValue(0,v);
            y2->setInputValue(0,v);
            y1d->setInputValue(0,v);
            y2d->setInputValue(0,v);
            y1dd->setInputValue(0,v);
            y2dd->setInputValue(0,v);
            y1ddd->setInputValue(0,v);
            y2ddd->setInputValue(0,v);

            of << v << "\t";
            of << y1->value() << "\t";
            of << y2->value() << "\t";
            of << y1->derivative(0) << "\t";
            of << y2->derivative(0) << "\t";
            of << y1d->value() << "\t";
            of << y2d->value() << "\t";
            of << y1d->derivative(0) << "\t";
            of << y2d->derivative(0) << "\t";
            of << y1dd->value() << "\t";
            of << y2dd->value() << "\t";
            of << y1dd->derivative(0) << "\t";
            of << y2dd->derivative(0) << "\t";
            of << y1ddd->value() << "\t";
            of << y2ddd->value() << "\t";
            of << y1ddd->derivative(0) << "\t";
            of << y2ddd->derivative(0) << "\n";
 
        }
    }
    return 0;
}

