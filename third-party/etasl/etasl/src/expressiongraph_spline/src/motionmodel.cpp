/**
 * reads in a motion model according to:
 *  Learning a Predictive Model of Human Gait for the Control of a Lower-limb Exoskeleton,
 *  Erwin Aertbelien, Joris De Schutter, 5th IEEE RAS/EMBS International Conference on
 *  Biomedical Robotics and Biomechatronics, August 12-15, 2014, Anhembi Convention Center in São Paulo, Brazil
 *
 *  E. Aertbelien (feb. 2015)
 */

#include <expressiongraph/motionmodel.hpp>
#include <fstream>

std::ostream& operator << (std::ostream& os, const KDL::MotionModel& m) {
    os << "MotionModel with " << m.getNrOfJoints() << " joints and " << m.getNrOfModes() << " modes";
    if (m.getType()==0) {
        os << " (using natural cubic splines) ";
    } else {
        os << " (using periodic cubic splines) ";
    }
    return os;
}

namespace KDL {

MotionModel::MotionModel(int _type, unsigned int _nrOfJoints):
    type(_type), nrOfJoints(_nrOfJoints),cspline(new CubicSpline(_type)) 
{
}


int MotionModel::readModel(const std::string& fn, const std::string& sep, int skipheader, bool velmodel) {
    // reading the file:
    int retval;
    retval = cspline->readPoints(fn,sep,skipheader);
    std::cout << "nr of outputs " << cspline->getNrOfOutputs() << std::endl;
    if (retval!=0) return retval;
    if (cspline->getNrOfOutputs() % nrOfJoints != 0) {
        return -6; /* number of columns is inconsistent with the nrOfJoints parameter */
    }
    m = cspline->getNrOfOutputs()/nrOfJoints - 1;

    // defining the input to the spline: s=v*t+s0
    cspline->setInput( input(1)*input(0)+input(2) );

    // defining the input for x_star
    std::vector<Expression<double>::Ptr> inps(m);
    for (unsigned int mode=0;mode<m;++mode) {
        inps[mode] = input( mode + 3); // better for efficiency to reuse an already created input(...)
    }
    // construct the expressions from the output (eq 15)
    outputs.resize(nrOfJoints);     
    for (unsigned int joint=0;joint<nrOfJoints;++joint) { 
        outputs[joint] = getSplineOutput( cspline, m*nrOfJoints+joint ); // b_{joint}
        for (unsigned int mode=0;mode<m;++mode) {
            outputs[joint] = outputs[joint] + getSplineOutput( cspline, mode*nrOfJoints+joint) * inps[mode];
        }
    }
    
    if (velmodel) {
        // a velocity model is requested.
        // since, given the KF state, v,s0 and x_star are constant (and time deriv==0),
        // the total time derivative of h towards time is equal to the partial time derivative.
        int time_ndx = 0;  // time has variable index 0 
        for (unsigned int joint=0;joint<nrOfJoints;++joint) {
            outputs[joint] = outputs[joint]->derivativeExpression( time_ndx );
        }
    }
    return 0;
}

void MotionModel::setInputValues( double t, double v, double s0, const Eigen::VectorXd& x_star) {
    assert( x_star.size() == m );
    for (unsigned int i=0;i<outputs.size();++i) {
        outputs[i]->setInputValue(0,t);
        outputs[i]->setInputValue(1,v);
        outputs[i]->setInputValue(2,s0);
        for (unsigned int j=0;j<m;++j) {
            outputs[i]->setInputValue(j+3,x_star(j));
        }
    }
}


void MotionModel::value( Eigen::VectorXd& h ) {
    assert(h.size()==nrOfJoints);
    for (unsigned joint=0;joint<nrOfJoints;++joint) {
        h(joint) = outputs[joint]->value();  
    }  
}

void MotionModel::jacobian( Eigen::MatrixXd& H ) {
    assert(H.rows()== nrOfJoints);
    assert(H.cols()== m+2);
    for (unsigned int joint=0;joint<nrOfJoints;++joint) {
        for (unsigned int i=0;i<m+2;++i) {
            H(joint, i) = outputs[joint]->derivative(i+1);
        }
    }
}

} // namespace KDL
