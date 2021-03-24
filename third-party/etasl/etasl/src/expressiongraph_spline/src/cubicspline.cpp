#include <kdl/expressiontree.hpp>
#include <kdl/conversions.hpp>
#include <fstream>
#include <expressiongraph/cubicspline.hpp>
#include <expressiongraph/splines.hpp>
//#include <expressiongraph/cubicsplineNd.hpp>
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <boost/tokenizer.hpp>
#include <string>
#include <cfloat>

namespace KDL {

const char* CubicSpline::spline_error_message(int err) {
    switch (err) {
        case CubicSpline::RET_SUCCESS: return "succesful";
        case CubicSpline::RET_NOT_PREPARED: return "spline data structures are not prepared";
        case CubicSpline::RET_NROFROWS: return "not enough rows for cubic spline of the specified type";
        case CubicSpline::RET_SIZE_MISMATCH: return "x and y data structures have a different number of rows";
        case CubicSpline::RET_COULD_NOT_OPEN_FILE: return "could not open file";
        default: return "unknown error";
    }
}



// u is also dependent on the joint variables.
CubicSpline::CubicSpline(const Expression<double>::Ptr s, const Eigen::VectorXd& _x, const Eigen::MatrixXd& _y,int _stype):
    MIMO("CubicSpline"), 
    x(_x), 
    y(_y), 
    ydd(_y.rows(),_y.cols()),
    stype(_stype),
    order(1)
{
    setPolicy( HUNT );
    prepared_status = -1000;
    inputDouble.push_back( s );
    assert(_x.rows()==_y.rows());
    prepare();
}

CubicSpline::CubicSpline(const Eigen::VectorXd& _x, const Eigen::MatrixXd& _y,int _stype):
    MIMO("CubicSpline"), 
    x(_x), 
    y(_y),
    stype(_stype),
    order(1)
{
    setPolicy( HUNT );
    prepared_status = -1000;
    assert(_x.rows()==_y.rows());
    x = _x;
    y = _y;
    prepare();
}

CubicSpline::CubicSpline(const Expression<double>::Ptr s,int _stype):
    MIMO("CubicSpline"),
    stype(_stype),
    order(1)
{
    inputDouble.push_back( s );
    setPolicy( HUNT );
    prepared_status = RET_NOT_PREPARED;
}

CubicSpline::CubicSpline(int _stype):
    MIMO("CubicSpline"),
    stype(_stype),
    order(1)
{
    setPolicy( HUNT );
    prepared_status = RET_NOT_PREPARED;
}

int CubicSpline::prepare() {
    if (prepared_status!=RET_SUCCESS) {
        if (y.rows() < 2)       return prepared_status=RET_NROFROWS;
        if (x.rows()!=y.rows()) return prepared_status=RET_SIZE_MISMATCH;
        ydd.resize(x.rows(), y.cols());
        Eigen::VectorXd yddcol(x.rows());
        if (stype == PERIODIC) {
            for (int i=0;i<y.cols();++i) {
                cubic_spline_periodic(x,y.col(i),yddcol);
                ydd.col(i) = yddcol;
                idx = -1;
            }
        } else {
            for (int i=0;i<y.cols();++i) {
                cubic_spline_natural(x,y.col(i),yddcol);
                ydd.col(i) = yddcol;
                idx = -1;
            }
        }
        yval.resize(y.rows());
        ydval.resize(y.rows());
        yddval.resize(y.rows());
        ydddval.resize(y.rows());
    }
    return prepared_status = RET_SUCCESS;
}

void CubicSpline::compute() {
    if (cached) return;  
    if (!prepare()!=RET_SUCCESS) {
        assert( prepared_status==0 );
    }
    if (stype== PERIODIC) {
        double val = reduce_periodic( x[0], x[x.size()-1], inputDouble[0]->value() );
        if (policy==HUNT) { 
            idx = hunt( x, val, idx); 
        } else { 
            idx = hunt( x, val, -1); 
        }
        switch( order ) {
            case 1:
                cubic_spline_periodic_evaluate(x,y,ydd,idx, val, &yval, &ydval);
                break; 
            case 2:
                cubic_spline_periodic_evaluate(x,y,ydd,idx, val, &yval, &ydval,&yddval);
                break; 
            case 3:
            default:
                cubic_spline_periodic_evaluate(x,y,ydd,idx, val, &yval, &ydval,&yddval,&ydddval);
                break; 
        }
    } else {
        double val = inputDouble[0]->value();
        if (policy==HUNT) { 
            idx = hunt( x, val, idx); 
        } else { 
            idx = hunt( x, val, -1); 
        }
        switch( order ) {
            case 1:
                cubic_spline_evaluate(x,y,ydd,idx, val, &yval, &ydval);
                break; 
            case 2:
                cubic_spline_evaluate(x,y,ydd,idx, val, &yval, &ydval,&yddval);
                break; 
            case 3: 
            default:
                cubic_spline_evaluate(x,y,ydd,idx, val, &yval, &ydval,&yddval,&ydddval);
                break; 
        }
    }
    cached=true;
    return;
}

double CubicSpline::getOutputValue(int column,double svalue) {
    if ((column<0)||(column>=y.cols())) return NAN;
    double yval;
    if (!prepare()!=RET_SUCCESS) {
        assert( prepared_status==0 );
    }
    if (stype== PERIODIC) {
        double val = reduce_periodic( x[0], x[x.size()-1], svalue );
        if (policy==HUNT) { 
            idx = hunt( x, val, idx); 
        } else { 
            idx = hunt( x, val, -1); 
        }
        cubic_spline_periodic_evaluate(x,y,ydd,column,idx, val, &yval);
    } else {
        if (policy==HUNT) { 
            idx = hunt( x, svalue, idx); 
        } else { 
            idx = hunt( x, svalue, -1); 
        }
        cubic_spline_evaluate(x,y,ydd,column,idx, svalue, &yval);
    }
    return yval;
}

double CubicSpline::getOutputDerivative(int column,double svalue) {
    if ((column<0)||(column>=y.cols())) return NAN;
    double ydval;
    if (!prepare()!=RET_SUCCESS) {
        assert( prepared_status==0 );
    }
    if (stype== PERIODIC) {
        double val = reduce_periodic( x[0], x[x.size()-1], svalue );
        if (policy==HUNT) { 
            idx = hunt( x, val, idx); 
        } else { 
            idx = hunt( x, val, -1); 
        }
        cubic_spline_periodic_evaluate(x,y,ydd,column,idx, val, NULL,&ydval);
    } else {
        if (policy==HUNT) { 
            idx = hunt( x, svalue, idx); 
        } else { 
            idx = hunt( x, svalue, -1); 
        }
        cubic_spline_evaluate(x,y,ydd,column,idx, svalue, NULL,&ydval);
    }
    return ydval;
}

void CubicSpline::setInput(const Expression<double>::Ptr& s_new) {
    inputDouble.resize(1);
    inputDouble[0] = s_new;
    invalidate_cache();
}

int CubicSpline::setPoints(const Eigen::VectorXd& _x, const Eigen::MatrixXd& _y) {
    x = _x;
    y = _y;
    return prepare();
}

MIMO::Ptr CubicSpline::clone() {
    CubicSpline::Ptr tmp( new CubicSpline(inputDouble[0]->clone(),x,y));
    return tmp;
} 

CubicSpline::Ptr CubicSpline::getNormalizerWithBounds(int n, double start_s, double end_s) {
    assert( n > 2 );
    CubicSpline::Ptr tmp(new CubicSpline());
    tmp->x.resize(n);
    tmp->y.resize(n,1);
    double step    = (end_s-start_s)/n;
    double s       = start_s;
    double L       = 0;
    Eigen::VectorXd y_prev(y.cols());
    Eigen::VectorXd y_cur(y.cols());
    idx = hunt(x,s,0);
    cubic_spline_evaluate( x, y, ydd, idx, s, &y_prev);
    tmp->x(0)   =  0.0;
    tmp->y(0,0) =  start_s;
    for (int i=1;  i<n;  i++) {
        s+= step;
        idx = hunt(x,s,0);
        cubic_spline_evaluate( x, y, ydd, idx, s, &y_cur);
        L += (y_cur-y_prev).norm();
        tmp->x(i)   = L;
        tmp->y(i,0) = s;
        y_prev = y_cur;
    }
    tmp->prepare();
    return tmp;
}


CubicSpline::Ptr CubicSpline::getNormalizer(int n) {
    return getNormalizerWithBounds(n, getMinArgument(), getMaxArgument() );
}


void CubicSpline::normalize(int n) {
    CubicSpline::Ptr normalizer (getNormalizer(n));
    normalizer->setInput(inputDouble[0]);
    setInput(getSplineOutput(normalizer,0));
}

int CubicSpline::readPoints(const std::string& fn, 
                            const std::string& sep, int skipheader) {
    using namespace std;
    using namespace boost;
    ifstream ifile(fn.c_str());
    std::vector< std::string > splitup;
    std::vector< std::vector<double> > values;
    std::string line;
    int    colsize = -1;
    int    rowsize=0;
    if (ifile.is_open()) { 
        while (ifile.good()) {
            getline(ifile, line);
            if (rowsize < skipheader) {
                skipheader--;
                continue;
            }
            boost::escaped_list_separator<char> esep("\\",sep,"\"\'");
            typedef boost::tokenizer< escaped_list_separator<char> > Tok;
            Tok tok(line,esep);
            std::vector<double> vline;
            for (Tok::iterator el=tok.begin();el!=tok.end();++el) {
                if (el->size()!=0) {
                    vline.push_back( strtod(el->c_str(),NULL) );
                }
            }
            if (colsize==-1) {
                colsize=vline.size();
                if (colsize < 2) {
                    return -3; // need at least two columns
                }
            }
            if (vline.size()==0) continue; // skip empty rows
            if ((int)vline.size()!=colsize) {
                return -2;     // all rows need to have the same amount of columns
            }
            values.push_back( vline );
            rowsize++;
        }
        int startrow = x.rows();
        if (startrow==0) {
            x = Eigen::VectorXd(rowsize); 
            y = Eigen::MatrixXd(rowsize,colsize-1); 
        } else {
            if (y.cols()!=colsize-1) {
                return -5; // expanding the splines with a different number of columns
            }
            x.conservativeResize(startrow+rowsize);
            y.conservativeResize(startrow+rowsize,colsize-1); 
        }
        for (int i=0;i<rowsize;++i) {
            std::vector<double>& v(values[i]);
            x(startrow+i) = v[0];
            for (int j=1;j<colsize;++j) {
                y(startrow+i,j-1) = v[j]; 
            }
        }
        ifile.close();
        return prepare();
    } else {
        return RET_COULD_NOT_OPEN_FILE; // could not read the file.
    }
}

Expression<double>::Ptr getSplineOutput(const CubicSpline::Ptr spline, int n) {
    using namespace std;
    if ((n<0) || (n >= spline->getNrOfOutputs())) {
        assert( 0 && "n is not between 0 and number of outputs - 1" );
        return Expression<double>::Ptr(); 
    }
    if (spline->prepare()!=CubicSpline::RET_SUCCESS) {
        assert( 0 && "prepare returned false" );
        return Expression<double>::Ptr();
    }
    if (spline->inputDouble.size()!=1) {
        assert( 0 && "input variable expression is not defined" );
        return Expression<double>::Ptr();
    } 
    return Expression<double>::Ptr(new CubicSpline_Output(spline,n));
}
/*
Expression<double>::Ptr getSplineDerivOutput(const CubicSpline::Ptr spline, int n, int towards) {
    using namespace std;
    if ((n<0) || (n >= spline->getNrOfOutputs())) {
        assert( 0 && "n is not between 0 and number of outputs - 1" );
        return Expression<double>::Ptr(); 
    }
    if (spline->prepare()!=CubicSpline::RET_SUCCESS) {
        assert( 0 && "prepare returned false" );
        return Expression<double>::Ptr();
    }
    if (spline->inputDouble.size()!=1) {
        assert( 0 && "input variable expression is not defined" );
        return Expression<double>::Ptr();
    } 
    Expression<double>::Ptr tmp(new CubicSpline_Deriv_Output(spline,n,0));
    return tmp * spline->inputDouble[0]->derivativeExpression(towards);
}
*/
/*
Expression<double>::Ptr getSpline2ndDerivOutput(const CubicSpline::Ptr spline, int n, int towards) {
    using namespace std;
    if ((n<0) || (n >= spline->getNrOfOutputs())) {
        assert( 0 && "n is not between 0 and number of outputs - 1" );
        return Expression<double>::Ptr(); 
    }
    if (spline->prepare()!=CubicSpline::RET_SUCCESS) {
        assert( 0 && "prepare returned false" );
        return Expression<double>::Ptr();
    }
    if (spline->inputDouble.size()!=1) {
        assert( 0 && "input variable expression is not defined" );
        return Expression<double>::Ptr();
    }
    Expression<double>::Ptr gd( new CubicSpline_Deriv_Output(spline,n));
    Expression<double>::Ptr sd( spline->inputDouble[0]->derivativeExpression(towards));
    Expression<double>::Ptr gdd(new CubicSpline_2nd_Deriv_Output(spline,n));
    return Expression<double>::Ptr(new CubicSpline_2nd_Deriv_Output(spline,n))* spline->inputDouble[0]->derivativeExpression(towards);
}
*/
}// namespace KDL

std::ostream& operator<<(std::ostream& os, const KDL::CubicSpline& spline) {
    os << "CubicSpline Object {\n\tnumber of outputs : "<<spline.getNrOfOutputs()
       <<"\n\t number of rows : " << spline.getNrOfRows()
       <<"\n\t running from " << spline.getMinArgument() << " to " << spline.getMaxArgument()
       <<"\n\t type of spline :  ";
    if (spline.getType()==KDL::CubicSpline::NATURAL) {
            os << "natural";
    } else {
            os << "periodic";
    }
    os <<"\n\t value lookup policy " << spline.getPolicy()
       << "\n\tstatus : " << spline.getPreparedStatus() 
       << "\n\tcompute derivatives up to orde : " << spline.getOrder() 
       << "\n}\n"<< std::endl;
    return os;
}


