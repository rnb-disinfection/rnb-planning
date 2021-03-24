#ifndef KDL_EXPRESSIONGRAPH_MANIPULABILITY_DIR_HPP
#define KDL_EXPRESSIONGRAPH_MANIPULABILITY_DIR_HPP

#include <kdl/expressiontree.hpp>
#include <kdl/conversions.hpp>
#include <fstream>

namespace KDL {

class Manipulability_Dir : public MIMO {
    public:
        Eigen::MatrixXd J,JJT,JJTi,Ji,Jiu,f_matrix;
        Eigen::MatrixXd J_der, JJT_der, JJTi_der, Ji_der, Jiu_der, f_matrix_der;
        Eigen::MatrixXd u;
        Eigen::MatrixXd u_der;
        double fvalue;
        std::vector<int> list_of_var;
        int ndof;   
        std::map<int,double> cached_derivs;
        Expression<Frame>::Ptr kinchain;
        double lambda;

        typedef boost::shared_ptr<Manipulability_Dir> Ptr;

        // u is also dependent on the joint variables.
        /**
         * node to compute direction^T pinv(J)^T pinv(J) direction, where the pseudo-inverse J is computed as a
         * damped pseudo-inverse with the given dampingfactor.  The Jacobian is computed by differentiating
         * _kinchain towards the variables given in the array of variable numbers _list_of_var.
         * 
        */
        Manipulability_Dir(Expression<Frame>::Ptr _kinchain, Expression<Twist>::Ptr direction, double dampingfactor, const std::vector<int>& _list_of_var);

        double compute_value();

        double compute_derivative(int i);

        virtual MIMO::Ptr clone();
        virtual ~Manipulability_Dir() {
            std::cout << "delete Manipulability_Dir() " << std::endl;
        }
};


class Manipulability_Dir_Output : public MIMO_Output<double> {
    public:
        typedef boost::shared_ptr<Manipulability_Dir_Output> Ptr;

        Manipulability_Dir_Output(MIMO::Ptr m):MIMO_Output<double>("Manipulability_Dir",m) {
        }
        
        double value() {
            Manipulability_Dir::Ptr p = boost::static_pointer_cast<Manipulability_Dir>(mimo);
            return p->compute_value();
        }

        double derivative(int i){
            Manipulability_Dir::Ptr p = boost::static_pointer_cast<Manipulability_Dir>(mimo);
            return p->compute_derivative(i);
        }
        MIMO_Output::Ptr clone() {
            Manipulability_Dir_Output::Ptr tmp( new Manipulability_Dir_Output(getMIMOClone()));
            return tmp;
        }
        virtual ~Manipulability_Dir_Output() {std::cout << "delete Manipulability_Dir_Output()" << std::endl;}
};

/**
 * returns an expressiongraph for the expression: 
 * <code>
 *  argmin_{q_dot} q_dot^T*q_dot subject to J(q)*q_dot = u
 * </code>
 * where J(q) corresponds to the Jacobian of the given chain towards the given list of variables, and
 * u correponds to the given direction.
 * This corresponds to the expression:
 * <code>
 *    u^T pinv(J)^T pinv(J) u
 * </code>
 * list_of_var defines over which variables you define the manipulability.
 * You can take the derivative towards any variables, not only the variables in _list_of_var.
 *
 * Due to the dampingfactor, the maximum value of this manipulability criterion is 
 * 1/dampingfactor^2. 
 *
 * \caveat Since this criterion corresponds to a squared norm, you probably want to take the square root of this criterion.
 *
 * \caveat direction is not normalized, if you want to have it normalized, you have to do it yourself.
 *
 * \caveat You can take the derivative towards these variables, but also towards other variables.
 *         ( e.g. moving a mobile base to improve the manipulability of the manipulator ).
 */
inline Expression<double>::Ptr create_manipulability_dir(
        Expression<Frame>::Ptr kinchain,
        Expression<Twist>::Ptr direction,
        double dampingfactor,
        const std::vector<int>& list_of_var ) {
    Manipulability_Dir::Ptr m( new Manipulability_Dir(kinchain, direction, dampingfactor, list_of_var ));
    Expression<double>::Ptr p( new  Manipulability_Dir_Output( m ) );
    //Expression<double>::Ptr p( new  InputType(1,0.0) );
    //return cached<double>(p); 
    return p; 
}

} // namespace KDL
#endif
