#ifndef EXPRESSIONGRAPH_BASE_SOLVER_HPP
#define EXPRESSIONGRAPH_BASE_SOLVER_HPP

#include <expressiongraph/context.hpp>
#include <Eigen/Dense>
#include <kdl/conversions.hpp>

namespace KDL {

#define HUGE_VALUE 1e20

/**
 * This solver understands three priorities:
 *   - 0:  hard constraint, used in initial loop closure.
 *   - 1:  hard constraint.
 *   - 2:  soft constraint.
 *
 * The solver expects three types of variables:
 *   - robot : the actuated variables
 *   - feature : feature variables
 *   - time    : type of the time variable.
 * There should be one variable defined with name "time".
 *
 * There are two different sets of variables:
 *  - There is a state, representing everything that to represent the state (including explicit time).
 *    This is used to  set the input values for all expression graphs.
 *  - There is the set of optimized variables. This is used by the QP-solver.
 * \nosubgrouping
 */
class solver {
protected:



        
        /// member variables (protected access)
        /// @{
        
        Context::Ptr        ctx;                ///< smart pointer to the currently prepared Context
        ExpressionOptimizer expr_opt;           ///< used to optimize the evaluation of expression graphs.
        std::vector<int>    all_ndx;            ///< a list of variable numbers for robot and feature, and the time variable.
        std::vector<int>    optim_ndx;          ///< a list of variable numbers involved in the optimization.
		/**
		 * the state: all that is necessary to evaluate an instantaneous instance of all
		 * expressions. These are robot variables, feature variables and time variable
		 */
		Eigen::VectorXd     state;
		int                 nv_robot;           ///< number of robot variables
		int                 nv_feature;         ///< number of feature variables

		bool                firsttime;          ///< indicates when solved is called for the first time.
		bool                initialization;     ///< true if this is the initialization phase.
		double              dt;                 ///< sample time.


		int                 nv;                 ///< total number of variables
		std::vector<int>    ndx_optim;          ///< index of all optimization variable
		int                 ndx_time;           ///< variable number of the time variable:

		Eigen::VectorXd     solution;           ///< results from the optimalisation: joint and feature velocities, slack variables
		double              norm_change;        ///< the norm of the last solution
        /// @}
public:
	typedef boost::shared_ptr< solver > Ptr;


    /**
     * Prepare for solving during an iterative initialization.
     * \param [in] ctx Context to initialize  the solver for.
     * \param [in] time time to start execution with.
     * \return 0 if sucessful, returns:
     *    - -1 if an unknown priority level is used.
     */
	virtual int prepareInitialization(Context::Ptr ctx)                  = 0;

    /**
     * set the initial values in the context to the current state.
     * (typically done after an initialization run).
     */    
	virtual void setInitialValues()                                      = 0;

	virtual void printMatrices(std::ostream& os)                         = 0;

    /**
     * Prepare for solving during an iterative execution.
     * \param [in] ctx Context to initialize  the solver for.
     * \param [in] time time to start execution with.
     * \return 0 if sucessful, returns:
     *    - -1 if an unknown priority level is used.
     */
	virtual int prepareExecution(Context::Ptr ctx)                        = 0;

    /**
     * returns the norm af the relevant state, ie feature variables for an initialisation step,
     * robot + feature variables for execution step.
     * only valid to call during initialization.
     */
	virtual double getNormChange()                                        = 0;
    
    /**
     * Sets the state variable (robot | feature | time) to the given values.
     */
	virtual void setState(const Eigen::VectorXd& _state)                   = 0;

    /**
     * Gets the state variable (robot | feature | time).
     */
	virtual void getState(Eigen::VectorXd& _state)                         = 0;

    /**
     * solves the optimization problem.
     * The typical call pattern is: 
     \code{.cpp} 
     \endcode 
     * \return 0 if successfull, use error_message method to interprete the error codes.
     */
	virtual int solve()                                                    = 0;

    /**
     * returns a string describing the error given by the code
     */
    virtual std::string errorMessage(int code)                             = 0;

    /**
     * returns the name of the solver
     */
    virtual std::string getName()                                          = 0;

    /**
     * gets the objective function value for the last time you caleed solve()
     */
	virtual double getWeightedResult()                                     = 0;

    /**
     * if you do not want to solve() but still want to evaluate all the 
     * expressions
     */
	virtual void evaluate_expressions()                                    = 0;


	virtual void getJointNameToIndex(std::map<std::string,int>& namemap)   = 0;
	virtual void getJointNameVector(std::vector<std::string>& namevec)     = 0;
	virtual void getFeatureNameToIndex(std::map<std::string,int>& namemap) = 0;
	virtual void getFeatureNameVector(std::vector<std::string>& namevec)   = 0;
	virtual int  getNrOfJointStates()                                      = 0;
	virtual int  getNrOfFeatureStates()                                    = 0;
	virtual void setJointStates(const Eigen::VectorXd& jntstate)           = 0;
	virtual void getJointStates(Eigen::VectorXd& jntstate)                 = 0;
	virtual void setAndUpdateJointStates(const Eigen::VectorXd& jntstate)  = 0;
	virtual void setFeatureStates(const Eigen::VectorXd& featstate)        = 0;
	virtual void getFeatureStates(Eigen::VectorXd& featstate)              = 0;
	virtual void setTime( double time)                                     = 0;
	virtual double getTime()                                               = 0;

    /**
     * /caveat not to be called during the initialization phase.
     */
	virtual void getJointVelocities(Eigen::VectorXd& _jntvel)              = 0;

    /**
     * /caveat not to be called during the initialization phase.
     */
	virtual void getFeatureVelocities(Eigen::VectorXd& _featvel)            = 0;

    /**
     * one update step of the execution/initialization of the task.
     * Is equivalent with using solve() and the setState(), getActuatorVelocities() and getFeatureVelocities() methods.
     * Is useful when simulating tasks.
     */
	virtual int  updateStep(double dt)                                       = 0;
    

	virtual ~solver() {}
};

} // namespace KDL
#endif

