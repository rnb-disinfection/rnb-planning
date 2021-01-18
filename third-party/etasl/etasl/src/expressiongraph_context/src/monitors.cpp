#include <string>
#include <vector>
#include <algorithm>
#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>


namespace KDL {


void Context::addDefaultObserver(Observer::Ptr observer) {
    defaultobserver = observer;
}

void Context::addMonitor(
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      lower,
        double                      upper,
        const std::string&          action_name,
        const std::string&          argument,
        Observer::Ptr               observer 
) {
    assert(lower<=upper);

    MonitorScalar m;
    std::string fullname = group_name()+"."+name;
    m.name        = fullname;
    m.groupname   = group_name();
    m.active      = false;
    m.expr        = expr;   
    m.lower       = lower;
    m.upper       = upper;
    m.action_name = action_name;
    m.argument    = argument; 
    m.observer    = observer;
    mon_scalar.push_back(m);
    activity_has_changed = true;
    //std::cerr << "addMonitor " << m.action_name << std::endl;
}

void Context::resetMonitors() {
    for (size_t indx=0;indx<mon_scalar.size();++indx) {
        MonitorScalar& m = mon_scalar[indx];
        if (m.active) {
            m.inside_int = true;
        }
    } 
}

void Context::checkMonitors() {
   update_active();
   for (size_t indx=0;indx<mon_scalar.size();++indx) {
        MonitorScalar& m = mon_scalar[indx];
        if (m.active) {
            double value = m.expr->value();
            //std::cout << "checking " << mon_scalar[i].name << " value " << value << std::endl;
            //std::cout << "limits " << mon_scalar[i].lower << "  " << mon_scalar[i].upper << std::endl;
            // if outside the limits:
            if ( (value < m.lower) || (m.upper < value) ) {
                //std::cerr << "exceeding limits " << std::endl;
                if (m.inside_int) {
                    //std::cerr << "triggered " << std::endl;
                    m.inside_int = false;
                    if (m.observer) {
                        m.observer->monitor_activated( m ); 
                    } else {
                        // if no specific observer is available, use the default one, if it exists:
                        if (defaultobserver) {
                            defaultobserver->monitor_activated( m );
                        } 
                    }
                }
            } else {
                m.inside_int = true;
            }
        }
    } 
}

void Context::setInputValues_monitors(const std::vector<int>& ndx,const Eigen::VectorXd& values) {
    update_active();
    for (size_t indx=0;indx<mon_scalar.size();++indx) {
        MonitorScalar& m = mon_scalar[indx];
        if (m.active) {
            m.expr->setInputValues(ndx,values);
        }
    }
}

void Context::setInputValues_monitors(const std::vector<int>& ndx,const std::vector<double>& values) {
    update_active();
    for (size_t indx=0;indx<mon_scalar.size();++indx) {
        MonitorScalar& m = mon_scalar[indx];
        if (m.active) {
            m.expr->setInputValues(ndx,values);
        }
    }
}

void Context::addToOptimizer_monitors(ExpressionOptimizer& opt) {
    update_active();
    for (size_t indx=0;indx<mon_scalar.size();++indx) {
        MonitorScalar& m = mon_scalar[indx];
        if (m.active) {
            m.expr->addToOptimizer(opt);
        }
    }
}

void Context::printMonitors( std::ostream& os ) {
    using namespace std;
    using boost::format;
    using boost::io::group; 
    os << "Monitors("<< mon_scalar.size()   << "):\n";
    os << format("\t%|64T-|\n");
    os << format("\t%1$=30s|%2$=16s|%3$=16s|\n") % "Name" % "Action type" % "Active";
    os << format("\t%|64T-|\n");

   for (size_t indx=0;indx<mon_scalar.size();++indx) {
       MonitorScalar& m = mon_scalar[indx];
       os << format("\t%1$=30s|%2$=16s|%3$=12d|\n") 
             % m.name 
             % m.action_name
             % m.active;

    }
    os << format("\t%|64T-|\n");
}

} // namespace KDL

