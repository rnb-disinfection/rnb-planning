#ifndef EXPRESSSIONGRAPH_CONTROLLER_CONSTRAINT_HPP
#define EXPRESSSIONGRAPH_CONTROLLER_CONSTRAINT_HPP

#include <map>
#include <kdl/expressiontree_double.hpp>

namespace KDL {

/** 
 * A pure abstract interface describing a control strategy for
 * constraints.
 *
 * Variables of a cloned controller refer back to the original
 * controller.
 *
 * - There is a base class ControllerBase to facilitate implementations.
 * - Typically the implementation and class of the controller is completely
 *   hidden in the .cpp file and only a register-function is provided
 * - 
 */
class Controller {
    public:
        typedef boost::shared_ptr<Controller> Ptr;

        /**
         * set and invididual parameter with given name to the given expression
         */
        virtual bool setParameter(const std::string& name, Expression<double>::Ptr value) = 0;

        /**
         * get a parameter or return a nil pointer otherwise
         */
        virtual Expression<double>::Ptr getParameter(const std::string& name)=0; 

        /**
         * get parameter list
         */
        virtual void getParameterNames(std::vector<std::string>& names)=0;

        /**
         * check whether the ensemble of parameters is valid.
         */
        virtual bool checkParameters() = 0;

        /**
         * gets the name of this controller
         */
        virtual const std::string& getName()=0;

        /**
         * cfr. expressiongraphs. passed to the underlying expressions.
         */
        virtual void setInputValues(const std::vector<int>& ndx,const Eigen::VectorXd& values) = 0;

        /**
         * cfr. expressiongraphs. passed to the underlying expressions.
         */
        virtual void setInputValues(const std::vector<int>& ndx,const std::vector<double>& values) = 0;


        /**
         * cfr. expressiongraphs. passed to the underlying expressions.
         */
        virtual void addToOptimizer(ExpressionOptimizer& opt) = 0;

        /**
         * actually computes the control strategy
         */
        virtual double compute( double error_value, double feedfwd ) = 0;

        /**
         * makes a virtual and deep copy of this object
         */
        virtual Controller::Ptr clone() = 0;

        virtual ~Controller() {}
};

/**
 * A Base class to help implementations for a Controller
 */
class ControllerBase : public Controller {
protected:
    typedef std::map<std::string, Expression<double>::Ptr> VarMap;
    VarMap vars;
    std::string name;
public:
        /**
         * create a controller with reasonable default values.
         */
        ControllerBase(const std::string& _name);
        /**
         * create a controller with the given variables defined.
         */
        ControllerBase(const ControllerBase& arg);
        virtual bool setParameter(const std::string& name, Expression<double>::Ptr value);
        virtual Expression<double>::Ptr getParameter(const std::string& name); 
        virtual void getParameterNames(std::vector<std::string>& names);
        virtual const std::string& getName();
        virtual bool checkParameters();
        virtual void setInputValues(const std::vector<int>& ndx,const Eigen::VectorXd& values);
        virtual void setInputValues(const std::vector<int>& ndx,const std::vector<double>& values);
        virtual void addToOptimizer(ExpressionOptimizer& opt);
        virtual double compute( double error, double feedfwd )=0;
        virtual Controller::Ptr clone()=0;
        virtual ~ControllerBase();
}; 

class ControllerRegistry {
        typedef std::map< std::string, Controller::Ptr>  ControllerMap;
        ControllerMap ctrlmap;
    public:
        typedef boost::shared_ptr< ControllerRegistry > Ptr;

        void register_controller(Controller::Ptr ctrl);
        /**
         * looks up a controller with given name and creates a new instance by cloning
         */
        Controller::Ptr create(const std::string& name); // nil if non existing


        /**
         * looks up a controller with a given name and returns the prototype
         * i.e. if you change the expression of its parameters, all newly created instances will inherit
         * these changed default parameters.
         *
         * use this to set default parameters for a controller
         */
        Controller::Ptr lookupPrototype(const std::string& name); // nil if non existing
};


extern Controller::Ptr create_controller_proportional();
extern Controller::Ptr create_controller_proportional_saturated();

}; // KDL
#endif
