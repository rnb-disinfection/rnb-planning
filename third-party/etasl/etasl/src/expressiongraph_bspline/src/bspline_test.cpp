#include <iostream>
#include <expressiongraph/bspline.hpp>


void disp(const std::string& name, const Eigen::VectorXd& tau, const Eigen::VectorXd&  b) {
    std::cout << name << "=np.array(";
    std::cout << "[ [" << tau[0] << ",\t"<< b[0] << " ]";
    for ( int i=1;i<tau.size();++i) {
        std::cout << ",\n  [ " << tau[i] << ",\t" << b[i] << " ]";
    }
    std::cout <<"]);"<<std::endl;
}

void dispknt(const std::string& name, const Eigen::VectorXd& knt) {
    if (knt.size()==0) {
        std::cout << name << "=np.array([]);";
        return;
    }
    std::cout << name << "=np.array(";
    std::cout << "[ " << knt[0];
    for ( int i=1;i<knt.size();++i) {
        std::cout << ", "  << knt[i];
    }
    std::cout <<"]);"<<std::endl;
}



int number_of_basis_functions( int knots_size,  int degree) {
    return knots_size+degree-1;
}


int main() {
    using namespace std;
    using namespace Eigen;
    cout << "import matplotlib.pyplot as plt" << endl;
    cout << "import numpy as np" << endl;
 
    int     degree      = 5;
    VectorXd knots(2);
    knots <<  0.0, 1.0;//, 2.0; // , 3.0;// , 4.0 ,5.0 , 6.0 , 7.0 ,8.0 , 9.0 , 10.0;
    dispknt("knots",knots);
    VectorXd aknots;
    bool res = augknots(knots,degree,aknots);
    assert(res==true);
    cout << "ankots_size="<< aknots.size() << endl;
    dispknt("aknots",aknots);
    int     tau_size    = 200;
    VectorXd tau = VectorXd::LinSpaced(tau_size, 0.0, knots[knots.size()-1]-1E-7);
    VectorXd result(tau_size);
    int nb = number_of_basis_functions(knots.size(), degree);
    VectorXd cp = VectorXd::Zero(nb);

    VectorXd d_knots;
    VectorXd d_cp;
    VectorXd d_result(tau_size);

    VectorXd dd_knots;
    VectorXd dd_cp;
    VectorXd dd_result(tau_size);


    dispknt("knots",knots);
    for (int i=0; i < nb;++i) {
        VectorXd tmp(degree+1);
        cp[i]=1.0;
        dispknt("cp",cp);
        for ( int r=0;r<tau_size;++r) {
            //result[r] = deBoor_basis(tau[r], aknots, i, degree, tmp);     
            result[r] = deBoor(tau[r], aknots, cp,degree, tmp);     
            derivknots(aknots,cp,degree,d_knots,d_cp);
            d_result[r] = deBoor(tau[r], d_knots, d_cp, degree-1, tmp);
            derivknots(d_knots,d_cp, degree-1, dd_knots, dd_cp);
            dd_result[r] = deBoor(tau[r], dd_knots, dd_cp, degree-2, tmp);
        }
        disp("result",tau, result);
        cout << "plt.figure("<< i << ");plt.subplot(3,1,1);"<<endl;
        cout << "plt.plot(result[:,0], result[:,1])" << endl;
        cout << "plt.title(r'$B_{"<<i<<","<<degree<<"}(\\tau)$');"<< endl;
        //cout << "plt.xlabel(r'$\\tau$');"<< endl;
        cout << "plt.xlabel(' cp [" << cp.transpose() << "] aknots["<< aknots.transpose() << "]');" << endl;

        disp("d_result",tau, d_result);
        cout << "plt.subplot(3,1,2)"<<endl;
        cout << "plt.plot(d_result[:,0], d_result[:,1])" << endl;
        cout << "plt.plot(result[:-1,0], np.diff(result[:,1])/(result[1,0]-result[0,0]))" << endl;
        //cout << "plt.xlabel(r'$\\tau$');"<< endl;
        cout << "plt.ylabel(r'deriv.');"<< endl;
        cout << "plt.xlabel(' d_cp [" << d_cp.transpose() << "] d_knots["<< d_knots.transpose() << "]');" << endl;

        disp("dd_result",tau, dd_result);
        cout << "plt.subplot(3,1,3)"<<endl;
        cout << "plt.plot(dd_result[:,0], dd_result[:,1])" << endl;
        cout << "plt.plot(result[:-2,0], np.diff(np.diff(result[:,1]))/(result[1,0]-result[0,0])/(result[1,0]-result[0,0]))" << endl;
        //cout << "plt.xlabel(r'$\\tau$');"<< endl;
        cout << "plt.ylabel(r'deriv.');"<< endl;
        cout << "plt.xlabel(' dd_cp [" << d_cp.transpose() << "] dd_knots["<< d_knots.transpose() << "]');" << endl;
 
        cp[i]=0.0;
    }
    cout << "plt.show()" << endl;
    return 0;
}
