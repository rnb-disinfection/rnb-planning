#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>

namespace KDL {

bool Context::existsConstraint(const std::string& name) {
    for (size_t i=0;i<cnstr_scalar.size();++i) {
        if (cnstr_scalar[i].name==name) return true;
    }
    for (size_t i=0;i<cnstr_box.size();++i) {
        if (cnstr_box[i].name==name) return true;
    }

    return false;
}

int Context::addInequalityConstraint(
        const std::string&          name,
        Expression<double>::Ptr     model,
        Expression<double>::Ptr     meas,
        double                      target_lower,
        double                      target_upper,
        Controller::Ptr             controller_lower,
        Controller::Ptr             controller_upper,
        Expression<double>::Ptr     weight,
        int                         priority
) {
    std::string fullname = group_name()+"."+name;
    if (!controller_lower->checkParameters()) return -1;
    if (!controller_upper->checkParameters()) return -1;
    if (priority < 0) return -2;
    if (existsConstraint(fullname)) return -3; 
    if (target_lower > target_upper) return -4;
    
    ConstraintScalar c;
    c.name             = name;
    c.groupname        = group_name();
    c.active           = false; 
    c.model            = model;
    c.meas             = meas;
    c.target_lower     = target_lower;
    c.target_upper     = target_upper;
    c.controller_lower = controller_lower;
    c.controller_upper = controller_upper;
    c.weight           = weight;
    c.priority         = priority;
    cnstr_scalar.push_back(c);
    activity_has_changed   = true; 
    return 0;
}


int Context::addBoxConstraint(
        const std::string&          name,
        int                         variablenr,
        double                      target_lower,
        double                      target_upper
    ) {
    std::string fullname = group_name()+"."+name;
    if (variablenr < 0) return -5;
    if (target_lower > target_upper) return -4;
    if (existsConstraint(fullname)) return -3; 

    ConstraintBox c;
    c.name                   = fullname;
    c.groupname              = group_name();
    c.active                 = false;
    c.variablenr             = variablenr;
    c.target_lower           = target_lower;
    c.target_upper           = target_upper;
    cnstr_box.push_back(c); 
    activity_has_changed   = true; 
    return 0;
}

void Context::setInputValues_constraints(const std::vector<int>& ndx,const Eigen::VectorXd& values) {
    update_active();
    for (size_t indx=0;indx<cnstr_scalar.size();++indx) {
        ConstraintScalar& c = cnstr_scalar[indx];
        if (c.active) {
            c.model->setInputValues(ndx,values);
            c.meas->setInputValues(ndx,values);
            c.controller_lower->setInputValues(ndx,values);
            c.controller_upper->setInputValues(ndx,values);
            c.weight->setInputValues(ndx,values);
        }
    }
}

void Context::setInputValues_constraints(const std::vector<int>& ndx,const std::vector<double>& values) {
    update_active();
    for (size_t indx=0;indx<cnstr_scalar.size();++indx) {
        ConstraintScalar& c = cnstr_scalar[indx];
        if (c.active) {
            c.model->setInputValues(ndx,values);
            c.meas->setInputValues(ndx,values);
            c.controller_lower->setInputValues(ndx,values);
            c.controller_upper->setInputValues(ndx,values);
            c.weight->setInputValues(ndx,values);
        }
    }
}

void Context::addToOptimizer_constraints(ExpressionOptimizer& opt) {
    update_active();
    for (size_t indx=0;indx<cnstr_scalar.size();++indx) {
        ConstraintScalar& c = cnstr_scalar[indx];
        if (c.active) {
            c.model->addToOptimizer(opt);
            c.meas->addToOptimizer(opt);
            c.controller_lower->addToOptimizer(opt);
            c.controller_upper->addToOptimizer(opt);
            c.weight->addToOptimizer(opt);
        }
    }
}
 
void Context::printConstraints( std::ostream& os ) {
    update_active();
    using namespace std;
    using boost::format;
    using boost::io::group; 
    os << "Constraints(:"<< (cnstr_scalar.size() + cnstr_box.size()) << ") :"<< endl;
    os << format("%|166T-|")<<endl;
    os << format("%1$=10s|%2$=60s|%3$=16s|%4$=16s|%5$=12s|%6$=10s|%7$=12s|%8$=12s|\n") % "Type" % "Name" % "lower bound" % "upper bound" % "weight" % "Priority" % "Current Val." % "Active";
    os << format("%|166T-|")<<endl;
    std::string red = "\033[1;31m";
    std::string off = "\033[0m";
    //os << "\033[1;31mbold red text\033[0m\n";
    for (size_t indx=0;indx<cnstr_scalar.size();++indx) {
        ConstraintScalar& c = cnstr_scalar[indx];
        if (c.active) {
            os << format("%1$=10s|%2$=60s|%3$=16d|%4$=16d|%5$=12d|%6$=10d|%7$=12d|%8$=12d|\n")
                     % "Scalar"
                     % c.name
                     % c.target_lower
                     % c.target_upper
                     % c.weight->value()
                     % c.priority
                     % c.meas->value()
                     % c.active;
        }
    }
    for (size_t indx=0;indx<cnstr_box.size();++indx) {
        ConstraintBox& c = cnstr_box[indx];
        //os << format("\t%1$=20s|%2$=40s|  Var%3$-11s\n") 
        os << format("%1$=10s|%2$=60s|%3$=16d|%4$=16d|%5$=12s|  Var%6$=5d|%7$=12s|%8$=12d|\n")
                 % "Box"
                 % c.name
                 % c.target_lower
                 % c.target_upper
                 % "-"
                 % c.variablenr
                 % "-"
                 % c.active;
    }
    os << format("%|166T-|\n");
}
 
} // namespace KDL

