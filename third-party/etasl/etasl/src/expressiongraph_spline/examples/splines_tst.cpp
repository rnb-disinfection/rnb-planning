#include <expressiongraph/splines.hpp>
#include <iostream>

int main() {
    using namespace std;
    using namespace Eigen;

    //cout << "\n\nSpline tests\n" << endl;
    VectorXd x(5);
    x << 0,1,2,3,4;
    VectorXd y(5);  
    y << 1,2,3,2,1;
    VectorXd ydd(5);
    //cubic_spline_periodic(x,y,ydd);
    cubic_spline_natural(x,y,ydd);
    int idx_prev = -1;
    for (double v=-4;v<=8.0;v+=0.1) {
        //double v2 = reduce_periodic(x[0],x[x.size()-1], v); 
        double v2=v;
        double yval;
        double ydval;
        double yddval;
        double ydddval;
        idx_prev = hunt(x,v2,idx_prev); 
        cubic_spline_evaluate(x,y,ydd,idx_prev,v2,&yval,&ydval, &yddval,&ydddval);
        cout << v << "\t" << yval << "\t" << ydval << "\t" << yddval << "\t" << ydddval << "\n";
    }
    cout << endl;
}
