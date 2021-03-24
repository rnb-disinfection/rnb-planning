#ifndef EXPRESSIONGRAPH_BSPLINE_EXPR_HPP
#define EXPRESSIONGRAPH_BSPLINE_EXPR_HPP
/**
 * Expressiongraph node to compute a B-spline basis-function.
 *
 * (c) 2017, Erwin Aertbelien, Dep. Of Mech. Eng., KULeuven.
 *  
 */


#include <iostream>
#include <exception>
#include <stdexcept>
#include <kdl/expressiontree.hpp>
#include <expressiongraph/bspline.hpp>



namespace KDL {


    Expression<double>::Ptr bspline_expr( 
                        Expression<double>::Ptr  a1, 
                        const Eigen::VectorXd&  knots,
                        const Eigen::VectorXd&  cp,
                        int                     d);
 

    class BSplineNode:
        public UnaryExpression<double,double>
    {
        Eigen::VectorXd tmp;
        Eigen::VectorXd knots;
        Eigen::VectorXd cp;
        int             degree;
        Eigen::VectorXd d_knots;
        Eigen::VectorXd d_cp;
        double val;

    public:

        BSplineNode(    Expression<double>::Ptr _a1, 
                        const Eigen::VectorXd&   _knots,
                        const Eigen::VectorXd&  _cp,
                        int                     _degree) :
            UnaryExpression<double,double>("BSpline",_a1),
            tmp(_degree+1),
            knots(_knots),
            cp(_cp),
            degree(_degree) {
                derivknots(knots,cp, degree, d_knots, d_cp);
                //std::cout << "d_knots " << d_knots.transpose() << std::endl;
                //std::cout << "d_cp " << d_cp.transpose() << std::endl;
        }

        virtual double value() {
            val = this->argument->value();
            return deBoor(val, knots,cp,degree,tmp);
        }
        virtual double derivative(int i) {
            double dval = argument->derivative(i);
            if (dval==0) {
                return 0;
            } else {
                return deBoor(val, d_knots,d_cp,degree-1,tmp)*dval;
            }
        }

        virtual Expression<double>::Ptr derivativeExpression(int i) {
            std::set<int> vset;
            argument->getDependencies(vset);
            if (vset.count(i)==0) {
                return Constant<double>(0.0);
            } else {
                return bspline_expr(argument, d_knots,d_cp,degree-1)*argument->derivativeExpression(i);
            }
        }

        virtual Expression<double>::Ptr clone() {
            return bspline_expr(argument, knots,cp,degree);
        } 
        ~BSplineNode() {}
    };



    /**
     * Returns an expression that evaluates a B-Spline
     * \param a1: a double expression, the B-spline will be evaluated at this point.
     * \param knots: a knot vector.  Should be increasing.  Knots can have a multiplicity
     *  up to "degree".  The spline will be continuous up to derivative (degree-multiplicity).
     *  It should start and end with knots of multiplicity "degree" (called an augmented knot vector
     *  in B-spline literature).
     * \param cp:    a vector with the values for the control points.  
     *      cp.size() == #knots.size() + degree -1 
     * \param d:  the degree of the B-spline (should be >=0)
     */
    inline Expression<double>::Ptr bspline_expr( 
                        Expression<double>::Ptr  a1, 
                        const Eigen::VectorXd&  knots,
                        const Eigen::VectorXd&  cp,
                        int                     d) {
        const char* msg2= "bspline_expr: first argument should be an (non-zero) expression pointer";
        const char* msg1="bspline_expr: both start and end elements of the knot vector should have multiplicity equal to the degree of the B-spline";
        const char* msg3="bspline_expr: knot vector should be increasing";
        const char* msg4="bspline_expr: size(knots) should be == size(cp)-degree+1";
        const char* msg5="bspline_expr: d should be >= 0";
        if (!a1) {
            throw std::domain_error(msg2);
        }
        if (knots.size() < 2*d) {
            throw std::domain_error(msg1);
        }
        double el = knots[0];
        for (int i=1;i<d;++i) {
            if (knots[i]!=el) {
                throw std::domain_error(msg1);
            }
        }
        el = knots[knots.size()-1];
        for (int i=1;i<d;++i) {
            if (knots[knots.size()-i-1]!=el) {
                throw std::domain_error(msg1);
            }
        }
        el = knots[0];
        for (int i=1;i<knots.size();++i) {
            if (el > knots[i]) {
                throw std::domain_error(msg3);
            }
        }
        if (cp.size()!=knots.size() - d +1) {
            throw std::domain_error(msg4);
        }
        if (d < 0) {
            throw std::domain_error(msg5);
        }
        return boost::make_shared< BSplineNode >(
                 a1, knots, cp, d
               );
    }




    /**
     * Returns an expression that evaluates a B-Spline
     * \param a1: a double expression, the B-spline will be evaluated at this point.
     * \param knots: a knot vector.  Should be increasing.  Knots can have a multiplicity
     *  up to "degree".  The spline will be continuous up to derivative (degree-multiplicity).
     *  It should start and end with knots of multiplicity "degree" (called an augmented knot vector
     *  in B-spline literature).
     * \param j: return the value of the j-th B-spline basis function.
     * \param d:  the degree of the B-spline (should be >=0)
     */
    inline Expression<double>::Ptr bspline_basis( 
                        Expression<double>::Ptr  a1, 
                        const Eigen::VectorXd&  knots,
                        int                     j,
                        int                     d) {
        const char* msg2= "bspline_expr: first argument should be an (non-zero) expression pointer";
        const char* msg1="bspline_expr: both start and end elements of the knot vector should have multiplicity equal to the degree of the B-spline";
        const char* msg3="bspline_expr: knot vector should be increasing";
        const char* msg5="bspline_expr: d should be >= 0";
        if (!a1) {
            throw std::domain_error(msg2);
        }
        if (knots.size() < 2*d) {
            throw std::domain_error(msg1);
        }
        double el = knots[0];
        for (int i=1;i<d;++i) {
            if (knots[i]!=el) {
                throw std::domain_error(msg1);
            }
        }
        el = knots[knots.size()-1];
        for (int i=1;i<d;++i) {
            if (knots[knots.size()-i-1]!=el) {
                throw std::domain_error(msg1);
            }
        }
        el = knots[0];
        for (int i=1;i<knots.size();++i) {
            if (el > knots[i]) {
                throw std::domain_error(msg3);
            }
        }
        if (d < 0) {
            throw std::domain_error(msg5);
        }
        Eigen::VectorXd cp = Eigen::VectorXd::Zero( knots.size()-d+1 );
        if ((0<=j)&&(j<cp.size())) {
            cp[j] = 1.0;
            return boost::make_shared< BSplineNode >( a1, knots, cp, d);
        } else {
            return Constant<double>(0.0);
        }
    }


};// namespace KDL



#endif
