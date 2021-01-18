#include <expressiongraph/controller.hpp>
#include <boost/make_shared.hpp>
#include <iostream>

namespace KDL {

//----------------------------------------------------------------------------------------------
//   ControllerBase
//----------------------------------------------------------------------------------------------


ControllerBase::ControllerBase(const std::string& _name):
    name(_name) {}

ControllerBase::ControllerBase(const ControllerBase& arg):
    vars(arg.vars),name(arg.name) {
    /*for (VarMap::iterator it = vars.begin(); it!= vars.end();++it) {
        setParameter(it->first, it->second);
    }*/
}

const std::string& 
ControllerBase::getName() {
    return name;
}

bool
ControllerBase::setParameter(const std::string& name, Expression<double>::Ptr value) 
{
    ////std::cout << "ControllerBase::setParameter " << name << " = " << value->value() << std::endl;
    vars[name] = value;
    return true;
} 

Expression<double>::Ptr
ControllerBase::getParameter(const std::string& name) 
{
    VarMap::iterator it = vars.find(name);
    if (it==vars.end()) {
        return Expression<double>::Ptr();
    } else {
        return it->second;
    }
}


void ControllerBase::getParameterNames(std::vector<std::string>& names) {
    names.clear();
    for (VarMap::iterator it=vars.begin();it!=vars.end();++it) {
            names.push_back(it->first); 
    }
}

bool ControllerBase::checkParameters() {}

void 
ControllerBase::setInputValues(
    const std::vector<int>& ndx,
    const Eigen::VectorXd& values
) {
    for (VarMap::iterator it = vars.begin(); it!= vars.end();++it) {
        it->second->setInputValues(ndx,values);
    }
}

void 
ControllerBase::setInputValues(
    const std::vector<int>& ndx,
    const std::vector<double>& values
) {
    for (VarMap::iterator it = vars.begin(); it!= vars.end();++it) {
        it->second->setInputValues(ndx,values);
    }
}

void 
ControllerBase::addToOptimizer(ExpressionOptimizer& opt) {
    for (VarMap::iterator it = vars.begin(); it!= vars.end();++it) {
        it->second->addToOptimizer(opt);
    }
}

ControllerBase::~ControllerBase() {}




//----------------------------------------------------------------------------------------------
//   ControllerRegistory
//----------------------------------------------------------------------------------------------

void ControllerRegistry::register_controller(Controller::Ptr ctrl) {
    ctrlmap[ctrl->getName()] = ctrl; 
}

Controller::Ptr ControllerRegistry::lookupPrototype(const std::string& name) {
    ControllerMap::iterator it = ctrlmap.find(name);
    if (it==ctrlmap.end()) {
        return Controller::Ptr();
    } else {
        return it->second;
    }
}

Controller::Ptr ControllerRegistry::create(const std::string& name) {
    ControllerMap::iterator it = ctrlmap.find(name);
    if (it==ctrlmap.end()) {
        return Controller::Ptr();
    } else {
        return it->second->clone();
    }
}


//----------------------------------------------------------------------------------------------
//   ControllerProportional
//----------------------------------------------------------------------------------------------




/**
 * Default controller : proportional controller with feedforward.
 */
class ControllerProportional : public ControllerBase {
    Expression<double>::Ptr K;
public:
        ControllerProportional():ControllerBase("proportional") {
            K = Constant<double>(4.0);
            ControllerBase::setParameter("K",K);
        }
        ControllerProportional( const ControllerProportional& obj) :
            ControllerBase(obj) {
                K = obj.K;
            }
        virtual bool setParameter(const std::string& name, Expression<double>::Ptr value) {
            //std::cout << "ControllerProportional::setParameter " << name << " = " << value->value() << std::endl;
            ControllerBase::setParameter(name,value);
            if (name=="K") {
                K=value;
                return true;
            }
            return false;
        }
        virtual bool checkParameters() {
            return (K->value() >= 0);
        }
        virtual Controller::Ptr clone() {
            //std::cout << "clone ControllerProportional" << std::endl;
            return boost::make_shared<ControllerProportional>(*this); 
        }
        virtual double compute( double error, double feedfwd ) {
            //static int count=0;
            //if (++count > 100) {
                //std::cout << "K : " << K->value() << "   error : " << error << "      feedfwd: "<< feedfwd << "    result: " <<  (K->value()*(error) + feedfwd) << std::endl;
            //    count=0;
            //}
            return K->value()*(error) + feedfwd;
        }
        virtual ~ControllerProportional() {}
}; 

Controller::Ptr create_controller_proportional() {
    return boost::make_shared<ControllerProportional>();
}



//----------------------------------------------------------------------------------------------
//   ControllerProportionalSaturated
//----------------------------------------------------------------------------------------------


/**
 * Controller that saturates it control values to a given minimum and maximum 
 */
class ControllerProportionalSaturated : public ControllerBase {
    Expression<double>::Ptr K;
    Expression<double>::Ptr min_output;
    Expression<double>::Ptr max_output;
public:
        ControllerProportionalSaturated():ControllerBase("proportional_saturated") {
            K = Constant<double>(4.0);
            min_output = Constant<double>(-1.5);
            max_output = Constant<double>(1.5);
            ControllerBase::setParameter("K",K);
            ControllerBase::setParameter("min_output",min_output);
            ControllerBase::setParameter("max_output",max_output);
        }
        ControllerProportionalSaturated( const ControllerProportionalSaturated& obj) :
            ControllerBase(obj) {
                K = obj.K;
                min_output = obj.min_output;
                max_output = obj.max_output;
            }
 
        virtual bool setParameter(const std::string& name, Expression<double>::Ptr value) {
            //std::cout << "ControllerProportionalSaturated::setParameter " << name << " = " << value->value() << std::endl;
            ControllerBase::setParameter(name,value);
            if (name=="K") {
                K=value;
                return true;
            }
            if (name=="min_output") {
                min_output = value;
                return true;
            }
            if (name=="max_output") {
                max_output = value;
                return true;
            }
            return false;
        }
        virtual bool checkParameters() {
            return (K->value() >= 0) && (min_output->value() < max_output->value() );
        }
        virtual Controller::Ptr clone() {
            //std::cout << "clone ControllerProportionalSaturated"<< std::endl;
            return boost::make_shared<ControllerProportionalSaturated>(*this); 
        }
        virtual double compute( double error, double feedfwd ) {
            double val =  K->value()*(error) + feedfwd;
            if (val < min_output->value()) val = min_output->value();
            if (val > max_output->value()) val = max_output->value();
            return val;
        }
        virtual ~ControllerProportionalSaturated() {}
}; 

Controller::Ptr create_controller_proportional_saturated() {
    return boost::make_shared<ControllerProportionalSaturated>();
}

}// namespace KDL;

