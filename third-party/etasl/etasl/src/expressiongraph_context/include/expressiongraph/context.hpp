#ifndef EXPRESSIONGRAPH_CONTEXT_HPP
#define EXPRESSIONGRAPH_CONTEXT_HPP
#include <kdl/expressiontree_expressions.hpp>
#include <kdl/expressiontree_double.hpp>
#include <kdl/expressiontree_var.hpp>
#include <boost/variant.hpp>
#include <boost/tuple/tuple.hpp>
#include <vector>
#include <string>
#include <bitset>
#include <sstream>
#include <cstdlib>
#include <exception>
#include <boost/any.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <expressiongraph/controller.hpp>

#include <iostream>

namespace KDL {


/**
 * A constraint on a scalar value.  Can be both equality and inequality constraint.
 * (setting lower and upper equal makes it an equality constraint)
 *
 * upper_controller == lower_controller for equality (i.e. pointing to the same object)
 */
struct ConstraintScalar {
    ConstraintScalar(){}

    std::string                        name;
    std::string                        groupname;
    bool                               active;        ///< indicates whether this constraint is active, i.e. belonging to an active group.
    Expression<double>::Ptr            model;
    Expression<double>::Ptr            meas;
    double                             target_lower;
    Controller::Ptr                    controller_lower;
    double                             target_upper;
    Controller::Ptr                    controller_upper;
    int                                priority;
    Expression<double>::Ptr            weight;
};

/**
 * A box constraint on the velocity of a variable
 */
struct ConstraintBox {
    std::string                         name;
    std::string                         groupname;
    bool                                active;        ///< indicates whether this constraint is active, i.e. belonging to an active group.
    int                                 variablenr;
    double                              target_lower;
    double                              target_upper;
};


struct MonitorScalar;

/**
 * An observer can be triggered by a Monitor.
 *
 * This class follows the observer pattern.
 * There can be an Observer for each Monitoring object, or there
 * can be one Observer to react to all Monitoring objects, or any
 * other variations.
 * \nosubgrouping
 */
class Observer {
public:
    /// smart pointer to an Observer object.
    typedef boost::shared_ptr<Observer> Ptr;
    /**
     * The solver will call this when MonitoringScalar is activated.
     * \param [in] mon the monitor that was activated.
     */
    virtual void monitor_activated(const  MonitorScalar& mon) = 0;
    virtual ~Observer() {}
};


/**
 * A data structure that describes monitoring of a scalar expression. 
 */
struct MonitorScalar {
    std::string                        name;          ///< name of the monitoring condition
    std::string                        groupname;     ///< group of the monitoring condition 
    bool                               active;        ///< indicates whether this monitor is active, i.e. belonging to an active group.
    Expression<double>::Ptr            expr;          ///< expression to monitor
    double                             lower, upper;  ///< monitor is _edge_ triggered when the expression is outside [lower, upper]
    std::string                        action_name;   ///< additional type identifier for the action.  Can be used by the observer
    std::string                        argument;      ///< additional argument for the action.  Semantics depend on the observer.
    Observer::Ptr                      observer;      ///< a (smart) pointer to an Observer object that will be called when the monitor is triggered.
    bool                               inside_int;    ///< expr was inside the interval.
};

/**
 * A double variable. Can be different type of variables such as robot variable
 * feature variable or time variable. 
 */
struct VariableScalar {
    std::string                       name;
    std::string                       type;
    int                               number;  ///< index number of this variable
    Expression<double>::Ptr           expr;
    double                            initial_value;
    Expression<double>::Ptr           weight;
};

/**
 * make a readable and valid identifier out of a string.
 * - A valid identifier consists of only the following symbols  [A-Z a-z 0-9 _]
 * - and starts with a symbol [A-Z a-z].  
 * - Other characters are replaced by "_", 
 * - all leading "_" are dropped, 
 * - all trailing "_" are dropped
 */
std::string make_identifier(const std::string& str);


/**
 * visitor to count the number of (scalar) output variables.
 * it is up to the caller to ensure that setInputValue() is called on the expressiongraphs.
 */
class OutputCounter :
    public boost::static_visitor<void>
{
       int counter; 
    public:
        OutputCounter(): counter(0) {}
        void operator()(Expression<double>::Ptr arg) {
            counter+=1;
        }
        void operator()(Expression<Vector>::Ptr arg) {
            counter+=3;
        }
        void operator()(Expression<Rotation>::Ptr arg) {
            counter+=9;
        }
        void operator()(Expression<Frame>::Ptr arg) {
            counter+=12;
        }
        int getCount() {
            return counter;
        }
        void operator()(Expression<Twist>::Ptr arg) {
            counter+=6;
        }
        void operator()(Expression<Wrench>::Ptr arg) {
            counter+=6;
        }
};



typedef boost::variant< Expression<double>::Ptr,
                         Expression<Vector>::Ptr,
                         Expression<Rotation>::Ptr,
                         Expression<Frame>::Ptr,
                         Expression<Twist>::Ptr,
                         Expression<Wrench>::Ptr >     ExpressionVarType;


/**
 * A Context object assembles the complete specification of a single task.
 * It contains variable, constraints and monitoring specifications.
 * This class encapsulates a named list of constraints.
 * 
 * The API of the Context class is subdivided into subgroups along
 * three dimensions:
 * - type
 *     - member variable
 *     - method
 * - type of use
 *     - specification of a task
 *     - getting information at solver-time
 * - type of methods, methods involving:
 *     - constraints
 *     - variables
 *     - monitors
 *     - outputs
 *     - grouping
 *
 * The use of input variable definitions:
 *     - During specification:  You can create an input using createInputChannel<T>(name).  This creates an expression representing
 *       the input that can be further used in the definitions.
 *     - During execution you can request a reference to this VariableType expression using 
 *       getInputChannel<T>(name).  This will return a nil pointer if the name does not
 *       exists.  This will return directly a VariableType pointer that you can use
 *       to set the value and Jacobian for the corresponding input (during execution!)
 *     - The input variables are treated as if they were dependent on time, such that the cache is updated at each time step.  The 
 *       normal rule of always calling setInputValue before value() still applies.
 *
 * The use of output variable defintions:
 *     - During specification you can assign an expression to a named outputvariable using setOutputExpression(name, expr)
 *       This assigns the symbolic expressiongraph to an outputvariable with the given name.
 *     - During execution you can get the expression corresponding to a given name and evaluate the expression:
 *         using getOutputExpression(name).
 *
 * \warning Specification routines are not to be used in hard-realtime.  For the Solver oriented
 * methods realtime-ness depends on the method at hand.
 *
 * \nosubgrouping
 */
class Context  {
    private:
        bool    activity_has_changed;
    public:
        typedef std::map<std::string, boost::any  >   InputVarMap;
        typedef std::map<std::string, ExpressionBase::Ptr  >   OutputVarMap;
        typedef std::map<std::string, VariableType<double>::Ptr > StartTimeMap;
        typedef std::map<std::string,double> StringMap;
        StringMap solver_property;
        std::map<std::string,std::string> solver_string_property;
        typedef boost::shared_ptr< Context > Ptr;
        Context(); 


        //----------------------------------------------------------
        /** 
         * \name MEMBERS: grouping 
         */
        /// @{  
        bool                     should_finish;               ///< whoever is executing the context, should finish
        std::vector<std::string> current_group_stack;         ///< defines the current group stack ( a list of names of groups newly added constraints will belong to)
        StartTimeMap             start_times;                 ///< the expression graph variable corresponding to start time for each of the groups.
        std::set<std::string>    active_groups;                  ///< currently active groups.
        /// @}
        //----------------------------------------------------------
        /** 
         * \name MEMBERS: constraints 
         */
        std::vector< ConstraintScalar>   cnstr_scalar;        ///< list of scalar constraints.
        std::vector< ConstraintBox >     cnstr_box;           ///< list of box constraints.

        /// @}
        //----------------------------------------------------------
        /** 
         * \name MEMBERS: monitors 
         */
        std::vector< MonitorScalar>   mon_scalar;             ///< list of scalar monitors.
        Observer::Ptr                 defaultobserver;        ///< default observer

        /// @}
        //----------------------------------------------------------
        /** 
         * \name MEMBERS: variables 
         */
        int                              next_varnr;          ///< next variable nummer that will be used for allocation.
        std::set<std::string>            var_types;           ///< allowable types
        std::set<std::string>            var_names;           ///< names that are already used.
        Expression<double>::Ptr          time;                ///< expression graph representing time.
        std::vector< VariableScalar>     var_scalar;          ///< list of scalar variables

        /// @}
        //----------------------------------------------------------
        /** 
         * \name MEMBERS: input channels 
         */
        /// @{  
        InputVarMap   input_vars;                            ///< map of names to  input channel variables (full name, including group).
        OutputVarMap  output_vars;

        /// @}
        //----------------------------------------------------------
        /** 
         * \name MEMBERS: outputs 
         */
        /// @{  
        std::vector< ExpressionVarType > output_exprs;        ///< list of expressions for output
        std::vector< std::string >       output_names;        ///< list of names for output
        std::vector< std::string >       output_types;        ///< list of types for output

        /// @}
        
        /** 
         * \name MEMBERS: controllers 
         */
        /// @{  
        ControllerRegistry::Ptr ctrl_registry;        ///< registry of all possible controller prototypes
        Controller::Ptr         default_ctrl;         ///< default controller 


        //----------------------------------------------------------
        /**
         * \name METHODS : controllers 
         */
        /// @{  
        virtual ControllerRegistry::Ptr getControllerRegistry() {
            return ctrl_registry;
        }
 
        virtual void setDefaultController(Controller::Ptr c) {
            default_ctrl = c; 
        } 

        virtual Controller::Ptr getDefaultControllerPrototype() {
            return default_ctrl;
        }

        virtual Controller::Ptr createDefaultController() {
            return default_ctrl->clone();
        }

        /// @}
        //----------------------------------------------------------
        /**
         * \name METHODS for specification: grouping 
         */
        /// @{  

        /**
         * sets a flag to signal that whoever is executing the context should finish.
         */
        void setFinishStatus() {should_finish = true;}        
        /**
         * sets a flag to signal that whoever is executing the context should finish.
         */
        void clearFinishStatus() {should_finish = false;}        
        /**
         * gets the status of a flag to signal that whoever is executing the context should finish.
         */
        bool getFinishStatus() { return should_finish;}

        /** 
         * get the current full group name.
         */
        std::string group_name();

        /**
         * returns a string containing the active group names
         */
        std::string active_group_names();

        /**
         * pushes a group name to the current group stack.
         * constraints are added to this current group stack and all groups below
         * the stack
         */
        void push_group(const std::string& name);

        /**
         * pops a group name from the current group stack.
         */
        void pop_group();


        /**
         * get the relative time for the given group name
         */
        Expression<double>::Ptr get_start_time(const std::string& name);

        /**
         * get the relative time for the current group 
         */
        Expression<double>::Ptr get_start_time() {
            return get_start_time(group_name());
        }

        /**
         * returns true if activities have changed.
         */
        bool is_activity_changed() {
            return activity_has_changed;
        }
        /**
         * activate a group name
         */
        void activate(const std::string& name);
       
        /**
         * deactivate a group name
         */
        void deactivate(const std::string& name);

        /**
         * deactivate all group name
         */
        void deactivate_all(const std::string& name);

        /**
         * executes group activation commands.
         * A group activation command is a comma or space separated list of one of the following commands:
         *  
         * - "+group_name"  activates the group with the given name
         * - "-group_name"  deactivate the group with the given name 
         * - "-*"           deactivate all groups
         * - "-partial_name*" deactivate all groups that start with the given partial name 
         *
         * \returns a boolean indicating whether the commands where parsed succesfully.
         */
        bool activate_cmd(const std::string& cmd);

        /**
         * update active constraints and monitors list: takes the list of active group names
         * and constructs a list of active constraints.  
         *
         * This routine will be called with the setInputValues, addToOptimizer  type of 
         * methods.  
         * 
         * can be called multiple times without decreasing efficiency.
         */ 
        void update_active(); 
 
public:
        /// @}
        //----------------------------------------------------------
        /**
         * \name METHODS for specification:  constraints 
         * The constraints can be of different types: Inequality, equality, scalar, rotation, box.
         * This is mainly an interface to define and check constraints. 
         * The solver can access the member variables in order to obtain the details of the constraints.
         * 
         * The Add... methods can have the following result:
         *   - 0  : successfully added.
         *   - -1 : control constant K should be >= 0.
         *   - -2 : priority should be >= 0
         *   - -3 : name already exists.
         *   - -4 : target_lower should be <= compared to target_upper.
         *   - -5 : variablenumber should be >= 0
         * @{
         */
        /// @{  



        /**
         * adds a scalar inequality constraint to the context, allowing to
         * specify custom controllers.
         * \param [in] name name of the inequality constraint.
         * \param [in] model expression graph representing an expression that models
         *             the expected evolution of the measurement in function of the robot joints
         *             feature variables, time and inputChannels. 
         * \param [in] meas expression graph representing the measurement, only the value of this
         *             expressiongraph is ever used.  The Jacobian of these expression is never used.
         * \param [in] target_lower: desired value of the lower inequality
         * \param [in] target_upper: desired value of the upper inequality
         * \param [in] controller_lower: represents the controller to be used to enforce the lower inequality.
         * \param [in] weight expressiongraph representing the weight of the constraint
         * \param [in] priority:  priority of the constraint.  
         * \param [in] equality: boolean, if true the constraint is an equality constraint and only 
         *             des_value_lower and controller_lower is used.
         * \returns 0 if successful, !=0 otherwise.
         */
        int addInequalityConstraint(
                const std::string&          name,
                Expression<double>::Ptr     model,
                Expression<double>::Ptr     meas,
                double                      target_lower,
                double                      target_upper,
                Controller::Ptr             controller_lower,
                Controller::Ptr             controller_upper,
                Expression<double>::Ptr     weight,
                int                         priority
            );
 
       
        /**
         * adds an inequality constraint on the velocity of a variable.
         * This is a separate type of constraint since it can, in most cases, be very efficiently implemented.
         * The constraint is always a "hard" constraint,i.e. one that cannot be violated
         * \param [in] name          name of the inequality constraint.
         * \param [in] variablenr    variable on which the constraint should be imposed.
         * \param [in] target_lower  lower limit on the velocity
         * \param [in] target_upper  upper limit on the velocity
         * \returns 0 if successful, !=0 otherwise.
         */
         int addBoxConstraint(
                const std::string&          name,
                int                         variablenr,
                double                      target_lower,
                double                      target_upper
            );

        /**
         * print information on the constraints to the given stream
         */
        void printConstraints(std::ostream& os);

        /// @}
        //----------------------------------------------------------
        /**
         * \name METHODS for specification:  monitors 
         */

        /**
         * adds a default observer object for all Montitors.
         * This will be used when the observer is not specified when adding
         * the monitors to this Context.
         * \param [in] observer smart pointer to an Observer object.
         */
        void addDefaultObserver(Observer::Ptr observer);

        /**
         * Adds a monitor to the current context.
         * \param [in] name  the name of the monitoring rule
         * \param [in] expr  expression graph for the monitoring expression
         * \param [in] lower the lower limit for the expression graph expression
         * \param [in] upper the upper limit for the expression graph expression
         * \param [in] action_name name of the action. Could be used by the observer between different types of
         *             monitors calling the observer. Or it could be used to transmit additional information, such
         *             as the next state for a state machine, etc...
         * \param [in] observer a pointer to an observer object to be called when the monitor is triggered. If it is a nil pointer, 
         *             the default observer will be used.  A nil pointer can be constructed using "Observer::Ptr()"
         */ 
        void addMonitor(
                const std::string&          name,
                Expression<double>::Ptr     expr,
                double                      lower,
                double                      upper,
                const std::string&          action_name,
                const std::string&          argument,
                Observer::Ptr               observer 
        );

        /**
         * Adds a monitor to the current context.
         * The default observer will be called when the monitor is triggered.
         * \param [in] name  the name of the monitoring rule
         * \param [in] expr  expression graph for the monitoring expression
         * \param [in] lower the lower limit for the expression graph expression
         * \param [in] upper the upper limit for the expression graph expression
         * \param [in] action_name name of the action. Could be used by the observer between different types of
         *             monitors calling the observer. Or it could be used to transmit additional information, such
         *             as the next state for a state machine, etc...
         */ 
        void addMonitor(
                const std::string&          name,
                Expression<double>::Ptr     expr,
                double                      lower,
                double                      upper,
                const std::string&          action_name,
                const std::string&          argument
        ) {
            addMonitor(name,expr,lower,upper,action_name,argument,Observer::Ptr() );
        }

        /**
         * print information on the monitors to the given stream
         */ 
        void printMonitors(std::ostream& os);

        /// @}
        //----------------------------------------------------------
        /**
         * \name METHODS for specification:  variables 
         */

        /**
         * Allocate a scalar variable.
         * \param [in] name name of the scalar variable.
         * \param [in] type type of the scalar variable.
         * \param [in] initial_value initial value for the variable.
         * \param [in] weight joint space weight (this is an expression graph).  This corresponds
         *             to the square root of the diagonal of the weight matrix, i.e. variable/weight is measured
         *             using the Euclidian norm.
         * \return nil pointer if allocation failed. (duplicate name, unknown type)
         */
        Expression<double>::Ptr addScalarVariable(
                const std::string& name,
                const std::string& type,
                double initial_value,
                Expression<double>::Ptr weight);

        /**
         * Allocate a scalar variable.
         * \param [in] name name of the scalar variable.
         * \param [in] type type of the scalar variable.
         * \param [in] initial_value initial value for the variable.
         * \param [in] weight joint space weight (this is an expression graph).  This corresponds
         *             to the square root of the diagonal of the weight matrix, i.e. variable/weight is measured
         *             using the Euclidian norm.
         * \return nil pointer if allocation failed. (duplicate name, unknown type)
         */
        Expression<double>::Ptr addDerivedVariable(
                const std::string& name,
                const std::string& type,
                double initial_value,
                int derivation_number,
                double dt,
                Expression<double>::Ptr weight);
        /**
         * gets a scalar expression pointing to the named variable (position level).
         * \param [in] name name of the scalar variable.
         * \returns an expression of the variable with the given name, or a null-expression if the name could not be found.
         */
        Expression<double>::Ptr   getScalarExpr(const std::string& name);

        /**
         * get the index belonging to a scalar variable.
         * \param [in] name name of the scalar variable
         * \returns a variable number of the variable with the given name, or -1 if the name could not be found.
         */
        int getScalarNdx(const std::string& name);


        /**
         * use named values (i.e. a std::map<std::string,double>) to specifiy the initial values.
         */
        void setInitialScalarValues(const std::map<std::string,double>& iv);

        /**
         * print information on the variables to the given stream
         */
        void printVariables(std::ostream& os);
        
        /**
         * \brief calls setInputValue on expressions inside the variable definitions (currently only weights are an expression).
         * \see documentation of expression graphs
         */ 
        void setInputValues_variables(const std::vector<int>& ndx,const Eigen::VectorXd& values);

        /**
         * \brief calls setInputValue on expressions inside the variable definitions (currently only weights are an expression).
         * \see documentation of expression graphs
         */ 
        void setInputValues_variables(const std::vector<int>& ndx,const std::vector<double>& values);
        
        /**
         * \brief calls addToOptimizer on expressions inside the variable definitions (currently only weights are an expression).
         * \see documentation of expression graphs
         */
        void addToOptimizer_variables(ExpressionOptimizer& opt);
 

        /// @}
  
        //----------------------------------------------------------
        /**
         * \name METHODS for specification:  outputs 
         * @{
         */
        /// @{  

        /** 
         * adds an expression graph to an output specification
         * \param [in] name name of the output
         * \param [in] e expression graph representing the output
         */
        template <typename T>
        void addOutput(const std::string& name, const std::string& type,typename Expression<T>::Ptr e) {
            output_names.push_back(name);
            output_types.push_back(type);
            output_exprs.push_back( e ); 
        }

        /**
         * print information on the output specification to the given stream.
         */
        void printOutputs(std::ostream& os);

        //----------------------------------------------------------
        /** 
         * \name METHODS for specification: input/output channels
         */
 
        /// @{  
        /**
         * create an input channel (VariableType object) and returns
         * a pointer to it.  If the name already exists, it will be
         * overwritten.  The short name is specified and automatically
         * completed to the full name.
         */
        template <typename T>
        typename Expression<T>::Ptr createInputChannel(const std::string& name) {
            typename VariableType<T>::Ptr tmp = Variable<T>(getScalarNdx("time"),1);
            input_vars[group_name() + "." + name] = tmp;
            return cached<T>(tmp); 
        }
        /**
         * create an input channel (VariableType object) and returns
         * a pointer to it.  Also give an initial value to the input
         * channel. If the name already exists, it will be
         * overwritten.  The short name is specified and automatically
         * completed to the full name.
         */
        template <typename T>
        typename Expression<T>::Ptr createInputChannel(const std::string& name, const T& initial_value) {
            typename VariableType<T>::Ptr tmp = Variable<T>(getScalarNdx("time"),1);
            input_vars[group_name() + "." + name] = tmp;
            tmp->setValue(initial_value);
            tmp->setJacobian(0, AutoDiffTrait<T>::zeroDerivative());
            return cached<T>(tmp); 
        }

        /**
         * declares an input object.  This provides a way to share an object on both the lua
         * side and the solver side.
         * 
         * \warning If the name already exists, it will be
         * overwritten.  
         *
         * \warning The short name is specified and automatically
         * completed to the full name.
         *
         * \warning uses the same data-structure as the inputChannels, so the name should be 
         * unique among both inputObjects and inputChannels
         */
        template <typename T>
        void declareInputObject(const std::string& name,const typename boost::shared_ptr<T>&  obj) {
            input_vars[group_name() + "." + name] = obj;
            std::cout << "declareInputObject " << typeid( obj).name() << std::endl;
        }




        /**
         * set output channel and returns a pointer to it.  If the name already exists, it will be
         * overwritten.  The short name is specified and automatically
         * completed to the full name.
         */
        template <typename T>
        void setOutputExpression(const std::string& name, typename Expression<T>::Ptr expr) {
            output_vars[group_name() + "." + name] = expr;
        }


        /// @}  

        /// @}
        //----------------------------------------------------------
        /**
         * \name METHODS for Solvers: constraints 
         */
        /// @{

        /**
         * Checks whether a constraint with the given name exists.
         */
        bool existsConstraint(const std::string& name);

        /**
         * calls setInputValue on all monitor expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_constraints(const std::vector<int>& ndx,const Eigen::VectorXd& values);

        /**
         * calls setInputValue on all monitor expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_constraints(const std::vector<int>& ndx,const std::vector<double>& values);
        
        /**
         * calls addtoOptimizer for all expressions.
         * ( see documentation of expression graphs)
         */
        void addToOptimizer_constraints(ExpressionOptimizer& opt);
 
        /// @}
        //----------------------------------------------------------
        /**
         * \name METHODS for Solvers: monitors 
         */
        /// @{

        /**
         * resets all monitors at the start of a loop, such that, when you start with
         * values outside the interval, an event is triggered once at start.
         * \warning Forgetting to call resetMonitors() can lead to unexpected ignoring of monitors in cases where
         *          the value of the expression of a monitor always is outside the interval.
         */
        void resetMonitors();

         /**
         * check all the monitor conditions and call the associated observers if the monitor conditions are triggered.
         * \pre setInputValue or equivalents should have been called before calling check().
         */
        void checkMonitors();

        /**
         * calls setInputValue on all monitor expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_monitors(const std::vector<int>& ndx,const Eigen::VectorXd& values);

        /**
         * calls setInputValue on all monitor expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_monitors(const std::vector<int>& ndx,const std::vector<double>& values);
        
        /**
         * calls addtoOptimizer for all monitoring expressions.
         * ( see documentation of expression graphs)
         */
        void addToOptimizer_monitors(ExpressionOptimizer& opt);
        
        /// @}

        //----------------------------------------------------------
        /** 
         * \name METHODS for Solvers: inputchannels
         */
        /// @{  

        /**
         * returns VariableType::Ptr for the given name.
         * returns a nill pointer if there are errors ( name does not exists or wrong type).
         * The full name should be used (i.e. including group names).
         * \warning the "global." is now optional and will automatically be checked.
         */
        template <typename T>
        typename VariableType<T>::Ptr getInputChannel(const std::string& name) {
            InputVarMap::iterator it = input_vars.find(name);
            if (it==input_vars.end())  {
                it = input_vars.find(std::string("global.")+name);
                if (it==input_vars.end())  {
                    return typename VariableType<T>::Ptr();
                }
            }
            if (it->second.type() == typeid(typename VariableType<T>::Ptr) ) {
                return boost::any_cast< typename VariableType<T>::Ptr >(it->second);
            } else {
                return typename VariableType<T>::Ptr();
            }
        }

        /**
         * returns a previously declared input object.
         * returns a nill pointer if there are errors ( name does not exists or wrong type).
         * The full name should be used (i.e. including group names).
         * \warning the "global." is now optional and will automatically be checked.
         */
        template <typename T>
        typename boost::shared_ptr<T> getInputObject(const std::string& name) {
            InputVarMap::iterator it = input_vars.find(name);
            if (it==input_vars.end())  {
                it = input_vars.find(std::string("global.")+name);
                if (it==input_vars.end())  {
                    return typename boost::shared_ptr<T>();
                }
            }
            if (it->second.type() == typeid(typename boost::shared_ptr<T>) ) {
                return boost::any_cast< typename boost::shared_ptr<T> >(it->second);
            } else {
                return typename boost::shared_ptr<T>();
            }
        }

        /**
         * Get the output expression.
         * \param [in] name  (full) name of the output expression.
         * \returns Expression::Ptr for the given name.
         * \returns a nill pointer if there are errors ( name does not exists or wrong type).
         * \warning The full name should be used (i.e. including group names).
         * \warning This expression will be updated together with the solver.
         * \warning the "global." is now optional and will automatically be checked.
         */
        template <typename T>
        typename Expression<T>::Ptr getOutputExpression(const std::string& name) {
            OutputVarMap::iterator it = output_vars.find(name);
            if (it==output_vars.end())  {
                it = output_vars.find(std::string("global.")+name);
                if (it==output_vars.end()) {
                    return typename Expression<T>::Ptr();
                }
            }
            return boost::dynamic_pointer_cast< Expression<T> >(it->second);
        }

        /**
         * print out a list of the input channels that are defined to the
         * specified output stream.
         */
        void printInputChannels(std::ostream& os);

        /**
         * print out a list of the outputexpressions that are defined to the
         * specified output stream.
         */
        void printOutputExpressions(std::ostream& os);

        /// @}  
 

        //----------------------------------------------------------
        /**
         * \name METHODS for Solvers:  variables 
         * Typically, a certain solver and application will define some default variables and types, such as
         * time variables and Time,Robot and Feature variable types.
         */ 
 
        /**
         * Adds an allowable type.
         * \param [in] type name of the type to be added to the list.
         */
        void addType(const std::string& type);
        
        /**
         * get a set of strings representing all declared variable types.
         * \param types [out] a set of type strings.
         */
         void getTypes(std::set<std::string>& types); 

        /**
         * adds all indices of a given type for all scalar variables to ndx
         * \param [in]  name of a variable type.
         * \param [out] ndx a vector of all variables of the given type.  The found indices are <b>added</b> to
         *              the vector ndx.
         * \warning if type does not exists, nothing is added to ndx.
         */
         void getScalarsOfType(const std::string& type, std::vector<int>& ndx);

        /**
         * gets all indices of a given type for all rotational variables.
         * For each rotational variables, 3 elements are given back using ndx, corresponding
         * to the elements of the rotational velocity vector.
         * \param [in]  name of a variable type 
         * \param [out] ndx a vector of all variables of the given type.
         void getRotationsOfType(const std::string& type, std::vector<int>& ndx);
         */

        /**
         * gets the variable name for a given index (Scalar variables).
         * For Rotational variables, the variable name is appended with "_x", "_y" or "_z"
         * to indicate which component of the rotational velocity is refered to.
         * \param [in] ndx variable number.
         * \return the variable name or empty if the variable number is not found.
         */
        std::string getName(int ndx);

        /**
         * gets the type name for a given index (Scalar variables).
         * \param [in] ndx variable number.
         * \return the variable type name or empty if the variable number is not found.
         */
        std::string getType(int ndx);

        /**
         * gets the structure corresponding to a variable with a given name.
         * \param [in] name name of the variable
         */
        VariableScalar* getScalarStruct(const std::string& name);

        /**
         * gets the structure corresponding to a variable with a given variable number.
         * \param [in] ndx the variable number.
         */
        VariableScalar* getScalarStruct(int ndx);

        /// @}
        /** 
         * \name METHODS for Solvers: outputs
         */
         /// @{
 
        /**
         * Applies a visitor to the Output specification.   
         * see output.cpp for some examples of visitors.
         * \param [in] v Visitor object
         * \param [in] type only apply the visitor to outputs with the given type.
         *             if type == "" then all outputs are used.
         * \pre setInputValue should have been called before.
         */
        template <typename V>
        void applyVisitor( V& v, const std::string& type="") {
            if (type.size()==0) {
                for (int i=0;i<(int)output_exprs.size();++i) {
                    boost::apply_visitor(v,output_exprs[i]);
                }
            } else {
                for (int i=0;i<(int)output_exprs.size();++i) {
                    if (output_types[i]==type) { 
                        boost::apply_visitor(v,output_exprs[i]);
                    }
                }
            }
        }

        /**
         * returns the number of scalar values the output will contain.
         */
        int outputCount(const std::string& type);

        /**
         * stores the output variables into an Eigen Vector
         * \pre setInputValue should have been called before.
         */
        void outputToVector(const std::string& type,Eigen::VectorXd& v);

        /**
         * Outputs the output variables to a stream.
         * \param [in] os       output stream.
         * \param [in] fieldsep seperation between each fields.
         * \param [in] linesep  seperation after each line.
         * \pre setInputValue should have been called before.
         */
        void outputToStream(const std::string& type,
                            std::ostream& os, 
                            const std::string& fieldsep, 
                            const std::string& linesep);

        /**
         * calls setInputValue on all monitor expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_outputs(const std::vector<int>& ndx,const Eigen::VectorXd& values);

        /**
         * calls setInputValue on all monitor expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_outputs(const std::vector<int>& ndx,const std::vector<double>& values);
        
        /**
         * calls addtoOptimizer for all monitoring expressions.
         * ( see documentation of expression graphs)
         */
        void addToOptimizer_outputs(ExpressionOptimizer& opt);
 


        /// @}

        /** 
         * \name METHODS for Solvers: output expressions (output channels)
         */
         /// @{
 
         /**
         * calls setInputValue on all outputvar expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_outputvars(const std::vector<int>& ndx,const Eigen::VectorXd& values);

        /**
         * calls setInputValue on all outputvar expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues_outputvars(const std::vector<int>& ndx,const std::vector<double>& values);
        
        /**
         * calls addtoOptimizer for all outputvar expressions.
         * ( see documentation of expression graphs)
         */
        void addToOptimizer_outputvars(ExpressionOptimizer& opt);
 
        /// @}
        //----------------------------------------------------------
        /**
         * \name METHODS for Solvers: general
         */
        /// @{

        /**
         * calls setInputValue on all expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues(const std::vector<int>& ndx,const Eigen::VectorXd& values);

        /**
         * calls setInputValue on all expressions.
         * ( see documentation of expression graphs)
         */ 
        void setInputValues(const std::vector<int>& ndx,const std::vector<double>& values);
        
        /**
         * calls addtoOptimizer for all expressions.
         * ( see documentation of expression graphs)
         */
        void addToOptimizer(ExpressionOptimizer& opt);


        /**
         * get a solver property, using the default if it not exists.
         * \param [in] name name of the property.
         * \param [in] default value
         * \return the value of the property.
         */
        double getSolverProperty(const std::string& name, double defaultval);


        /**
         * get a solver property, using the default if it not exists.
         * \param [in] name name of the property.
         * \param [in] default value
         * \return the value of the property.
         */
        std::string getSolverStringProperty(const std::string& name, const std::string& defaultvalue);

        /**
         * set a solver property        
         * \param [in] name name of the property.
         * \param [in] value of the property.
         */
         void setSolverProperty(const std::string& name, double value);

        /**
         * set a solver property        
         * \param [in] name name of the property.
         * \param [in] value of the property.
         */
         void setSolverStringProperty(const std::string& name, const std::string& value);

        virtual ~Context() {}
        /// @}
};


inline Context::Ptr create_context() {
    Context::Ptr ctx ( new Context() );
    return ctx;
}

std::ostream& operator << ( std::ostream& os, const Context::Ptr v );

class OutputGenerator {
    public:
        typedef boost::shared_ptr<OutputGenerator> Ptr;

        OutputGenerator(): next_generator() {}
        OutputGenerator( Ptr next );

        virtual int init(Context::Ptr outp);
        virtual int update(Context::Ptr outp);
        virtual int finish(Context::Ptr outp);
        virtual ~OutputGenerator();
    private:
        Ptr  next_generator;
};


};//namespace
#endif
