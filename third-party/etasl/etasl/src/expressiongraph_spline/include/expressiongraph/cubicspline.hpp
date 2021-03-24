#ifndef KDL_EXPRESSIONGRAPH_CUBICSPLINE_HPP
#define KDL_EXPRESSIONGRAPH_CUBICSPLINE_HPP

#include <kdl/expressiontree.hpp>
#include <kdl/conversions.hpp>
#include <fstream>
#include <boost/lexical_cast.hpp>

namespace KDL {

class CubicSpline_Output;

extern const char* CubicSpline_Messages[];

/**
 * Expressiongraph node that encapsulates a cubic spline
 * i.e. a spline with 3th order polyonomial segments where continuity
 * of the 1st and 2nd derivative is enforced.
 *
 */ 
class CubicSpline : public MIMO {
        int prepared_status;  // has interp.prepare() already been called ? and what was the result?
        int policy;
        int idx;
    protected:
        Eigen::VectorXd x;   //used by the output classes
        Eigen::MatrixXd y;   //used by the output classes
        Eigen::MatrixXd ydd;  //used by the output classes
        Eigen::VectorXd yval;   //used by the output classes
        Eigen::VectorXd ydval;   //used by the output classes
        Eigen::VectorXd yddval;   //used by the output classes
        Eigen::VectorXd ydddval;   //used by the output classes
        int stype;
        int order;    // compute up to the order-th derivative.
        // to be used by output classes
        void compute();
    public:

        enum Policy {HUNT=0,BISECTION=1};
        enum Error  {RET_SUCCESS=0, RET_NOT_PREPARED=1,RET_NROFROWS=2, 
                     RET_SIZE_MISMATCH=3, RET_COULD_NOT_OPEN_FILE=4};
        enum SplineType { NATURAL=0, PERIODIC=1};
        typedef boost::shared_ptr<CubicSpline> Ptr;

        CubicSpline(const Expression<double>::Ptr s, const Eigen::VectorXd& x, const Eigen::MatrixXd& y, int stype=NATURAL);
        CubicSpline(const Eigen::VectorXd& x, const Eigen::MatrixXd& y,int stype=NATURAL);
        CubicSpline(const Expression<double>::Ptr s,int stype=NATURAL);
        CubicSpline(int stype=NATURAL);

        const char* spline_error_message(int err);

        double getMinArgument() const {
            if (x.size()>0) {
                return x(0);
            } else {
                return 0.0;
            }
        }
        double getMaxArgument()  const{
            if (x.size()>0){
                return x(x.size()-1);
            } else {
                return getMinArgument();
            }
        }

        /**
         * get the number of outputs
         */ 
        int getNrOfOutputs() const { return y.cols();}

        /**
         * get the number of rows in x and y.
         */
        int getNrOfRows() const { return y.rows();}

        /**
         * get the policy by which to lookup values:
         */ 
        int getPolicy() const { return policy;}


        /**
         * the spline will compute derivatives up to the returned order
         */
        int getOrder() const { return order;}

        /**
         * get the type of spline (natural==0 or periodic==1)
         */
        int getType() const { return stype;}

        /**
         * sets the policy by which to lookup values.
         * \see getPolicy
         */
        void setPolicy(int p) { policy=p;}

        /**
         * returns prepare status code:
         * * -1000 : prepared is not called.
         * * -1    : x array is not sorted
         * * -2    : x and y do not have the same amount of rows
         * * -3    : size of y is smaller than 2.
         * * 0     : everything is alright. 
         */
        int  getPreparedStatus() const { return prepared_status;}
 
        /**
         * prepare() is called to prepare this class for the evaluation of the
         * interpolation.  You can call it separately, if not is called automatically
         * during the first evaluation.  It takes $O(n)$ computing time, with $n$ 
         * the number of points. 
         */ 
        int prepare();

        /**
         * sets the input expression variable of the spline the given expression,
         * \param s_new an expressiongraph representing the input to this spline.
         */
        void setInput(const Expression<double>::Ptr& s_new);

        /**
         * set the independent variable x and the dependent variables y
         * The spline will interpolate these points.
         **/
        int setPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& y);

        /**
         * read the independent and dependent variables from a tab delimited
         * file.
         * \param fn : filename
         * \param sep: separators to separate columns (e.g. ",")
         * \param skipheader : the number of rows the skip (typically because of an header)
         * This method returns the following values:
         * * 0: success
         * * -1 if file could not be read, 
         * * -2 if the rows do not have the same amount of columns, 
         * * -3 if the file does not have at least two columns (for x and for y), 
         * * -4 if spline preparation failed, e.g. because the x points where not in ascending order, 
         * * -5 if you are extending existing data with a different amount of columns.
         */
        int readPoints(const std::string& fn, const std::string& sep,int skipheader);

        virtual MIMO::Ptr clone();

      
        /**
         * returns a 1-dimensional CubicSpline  that maps the length of the spline
         * in this object to the domain of the spline.  
         * The input variable expression for this spline is still impty.
         * \param n the length is interpolated in n points. 
         */
        CubicSpline::Ptr getNormalizer(int n);
 
        /**
         * returns a 1-dimensional CubicSpline  that maps the length of the spline
         * in this object to the domain of the spline.  
         * The input variable expression for this spline is still impty.
         * \param n the length is interpolated in n points. 
         * \param startsp start with this value on the original spline
         * \param endsp   end with this value on the original spline (can be larger then the spline, although this is only
         *                useful when it is a periodic spline.
         */
        CubicSpline::Ptr getNormalizerWithBounds(int n,double startsp,double endsp);


        
        /**
         * uses getNormalizer and replace_input to adapt the spline in this
         * object such that it has a natural parameterisation
         * \param n use an n-times interpolated length to approximate the inverse length 
         * function.
         */
        void normalize(int n);

        
        /**
         * gets the value of a spline for a specific value of its argument svalue
         */
        double getOutputValue(int column,double svalue);

        /**
         * gets the value of the derivative of a spline for a specific value of its argument svalue
         */
        double getOutputDerivative(int column, double svalue);

        virtual ~CubicSpline() {
        }

        friend class CubicSpline_Output;
};


class CubicSpline_Output : public MIMO_Output<double> {
        int outputnr;
        int deriv_order;
    protected:
        CubicSpline_Output(MIMO::Ptr m, int _outputnr, int _deriv_order=1):
            MIMO_Output<double>("CubicSplineOutput_"+boost::lexical_cast<std::string>(_outputnr) +"_deriv"+boost::lexical_cast<std::string>(_deriv_order),m), 
            outputnr(_outputnr),
            deriv_order(_deriv_order)
        {
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->order = std::max(p->order,deriv_order);
        }
    public:
        typedef boost::shared_ptr<CubicSpline_Output> Ptr;

        
        double value() {
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->compute();
            switch (deriv_order) {
                case 1:return p->yval[outputnr];
                case 2:return p->ydval[outputnr];
                case 3:return p->yddval[outputnr];
                case 4:return p->ydddval[outputnr];
                default: return 0.0;
            }
        }

        double derivative(int i){
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->compute();
            switch (deriv_order) {
                case 1:return p->ydval[outputnr]*p->inputDouble[0]->derivative(i);
                case 2:return p->yddval[outputnr]*p->inputDouble[0]->derivative(i);
                case 3:return p->ydddval[outputnr]*p->inputDouble[0]->derivative(i);
                default: return 0.0;
            }
        }

        virtual typename Expression<double>::Ptr derivativeExpression(int i) {
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            CubicSpline_Output::Ptr tmp( new CubicSpline_Output(mimo, outputnr, deriv_order+1));
            return tmp * p->inputDouble[0]->derivativeExpression(i);
        }

        MIMO_Output::Ptr clone() {
            CubicSpline_Output::Ptr tmp( new CubicSpline_Output(getMIMOClone(), outputnr,deriv_order));
            return tmp;
        }

        virtual ~CubicSpline_Output() {}
        friend Expression<double>::Ptr getSplineOutput(const CubicSpline::Ptr spline, int n);
};

Expression<double>::Ptr getSplineOutput(const CubicSpline::Ptr spline, int n);
/**
class CubicSpline_Deriv_Output : public MIMO_Output<double> {
        int outputnr;
    protected:
       CubicSpline_Deriv_Output(MIMO::Ptr m, int _outputnr):
            MIMO_Output<double>("CubicSpline_Deriv_Output_"+boost::lexical_cast<std::string>(_outputnr),m), 
            outputnr(_outputnr)
        {
             CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->order = std::max(p->order,2);
        }
    public:
        typedef boost::shared_ptr<CubicSpline_Deriv_Output> Ptr;

       
        double value() {
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->compute();
            return p->ydval[outputnr];
        }

        double derivative(int i){
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->compute();
            return p->yddval[outputnr]*p->inputDouble[0]->derivative(i);
        }

        MIMO_Output::Ptr clone() {
            CubicSpline_Deriv_Output::Ptr tmp( new CubicSpline_Deriv_Output(getMIMOClone(), outputnr));
            return tmp;
        }

        virtual ~CubicSpline_Deriv_Output() {}
        friend Expression<double>::Ptr getSplineDerivOutput(const CubicSpline::Ptr spline, int n, int towards);
};
Expression<double>::Ptr getSplineDerivOutput(const CubicSpline::Ptr spline, int n, int towards);


class CubicSpline_2nd_Deriv_Output : public MIMO_Output<double> {
        int outputnr;
    protected:
       CubicSpline_2nd_Deriv_Output(MIMO::Ptr m, int _outputnr):
            MIMO_Output<double>("CubicSpline_2nd_Deriv_Output_"+boost::lexical_cast<std::string>(_outputnr),m), 
            outputnr(_outputnr)
        {
             CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->order = std::max(p->order,2);
        }
    public:
        typedef boost::shared_ptr<CubicSpline_Deriv_Output> Ptr;

       
        double value() {
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->compute();
            return p->yddval[outputnr];
        }

        double derivative(int i){
            CubicSpline::Ptr p = boost::static_pointer_cast<CubicSpline>(mimo);
            p->compute();
            return p->ydddval[outputnr]*p->inputDouble[0]->derivative(i);
        }

        MIMO_Output::Ptr clone() {
            CubicSpline_2nd_Deriv_Output::Ptr tmp( new CubicSpline_2nd_Deriv_Output(getMIMOClone(), outputnr));
            return tmp;
        }

        virtual ~CubicSpline_2nd_Deriv_Output() {}
        friend Expression<double>::Ptr getSpline2ndDerivOutput(const CubicSpline::Ptr spline, int n, int towards);
};

Expression<double>::Ptr getSpline2ndDerivOutput(const CubicSpline::Ptr spline, int n, int towards);
**/

inline CubicSpline::Ptr cubic_spline(const Expression<double>::Ptr s, const Eigen::VectorXd& x, const Eigen::MatrixXd& y, int stype=0) {
    CubicSpline::Ptr expr( new CubicSpline(s, x, y,stype) );
    return expr;
}
inline CubicSpline::Ptr cubic_spline(const Expression<double>::Ptr s, int stype=0) {
    CubicSpline::Ptr expr( new CubicSpline(s,stype) );
    return expr;
}
inline CubicSpline::Ptr cubic_spline(int stype=0) {
    CubicSpline::Ptr expr( new CubicSpline(stype) );
    return expr;
}

/**
 *
inline Expression<double>::Ptr create_CubicSpline(
        Expression<Frame>::Ptr kinchain,
        Expression<Twist>::Ptr direction,
        double dampingfactor,
        const std::vector<int>& list_of_var ) {
    CubicSpline::Ptr m( new CubicSpline(kinchain, direction, dampingfactor, list_of_var ));
    Expression<double>::Ptr p( new  CubicSpline_Output( m ) );
    //Expression<double>::Ptr p( new  InputType(1,0.0) );
    //return cached<double>(p); 
    return p; 
}
*/
} // namespace KDL

std::ostream& operator<<(std::ostream& os, const KDL::CubicSpline& spline);


#endif
