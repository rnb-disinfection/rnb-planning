/**
 * /file itasc_adaptor.hpp
 * This file defines an adaptor for iTaSC_Tasks to the expression graph task specification framework.
 */
#ifndef EXPRESSIONGRAPH_ITASC_ADAPTOR_HPP
#define EXPRESSIONGRAPH_ITASC_ADAPTOR_HPP

#include <expressiongraph/context.hpp> 
#include <expressiongraph/itasc_task.hpp>
#include <kdl/expressiontree_expressions.hpp>
#include <kdl/expressiontree_double.hpp>
#include <kdl/expressiontree_mimo.hpp>
#include <Eigen/LU>

//#define PRINT_MATRICES

namespace KDL {


/**
 * - manages calling the iTaSC task
 * - computes A = Cq - Cf*inv(Jf)*Jq
 * - and its des. value = ydot_d_0 + Cf*inv(Jf)*tu
 * - WHAT TO DO IN CASE OF SINGULARITY  of Jf ?
 * - Note that both tsk->update() and this adaptor will have to compute inv(Jf)!
 * - encapsulates an iTaSC task as f( q, chi_f(q) )
 * - cache of the computed values as long as setInputValue() is not called.
 * - be sure that it is compatible with the expression optimizer.
 * \caveat Default value for the joint variable is always equal to zero ! 
 *   (i.e. the value when setInputValue(...) is never called on the joint variable.)
 * \TODO adaptor does not receive new variable values when using the optimizer !
 *       solution use explicit dependency on the correct input() expression graphs, and this
 *       is automatically handled.
 */
class iTaSC_Adaptor:public MIMO {
    public:
        typedef boost::shared_ptr<iTaSC_Adaptor> Ptr;

        Eigen::MatrixXd     A; 
        Eigen::MatrixXd     Cq_partial; 
        Eigen::MatrixXd     Cq; 
        Eigen::MatrixXd     Cf; 
        Eigen::MatrixXd     Jq; 
        Eigen::MatrixXd     Jf; 
        Eigen::VectorXd     ydot_d_0;
        Eigen::VectorXd     target_velocity;
        Eigen::VectorXd     Wy;
        Eigen::VectorXd     q;
        Eigen::VectorXd     q_partial;
        Eigen::VectorXd     tu;
        iTaSC_Task::Ptr     tsk;
        std::map<int,int>   varnr_to_columnnr;
        int                 nrofjoints;
        int                 nrofjoints_partial;
        int                 nroffeatures;
        int                 nrofoutputs;
        int                 nrofloops;
        Expression<Frame>::Ptr o1; 
        Expression<Frame>::Ptr o2; 
        Expression<Frame>::Ptr T;

        Eigen::MatrixXd                      tmp_JfiJq;
        Eigen::PartialPivLU<Eigen::MatrixXd> tmp_lu;
        Eigen::VectorXd                      tmp_Jfitu;
        std::vector<int>                     listOfVariableNumbers;
        int                                  time_ndx;
        /**
         * \param [in] _tsk a shared ptr to an iTaSC_Task object.
         * \param [in] _o1  a frame o1, expressed as an expression graph
         * \param [in] _o2  a frame o2, expressed as an expression graph
         * \param [in] listOfVariableNumbers a list of the variableNumbers corresponding to the columns of Jq and Jf.
         * \caveat to make sense, _o1 and _o2 should be transformations expressed with respect to the same base,
         *         whatever this base is (it should not necessarily be the world frame)
         */
        iTaSC_Adaptor( iTaSC_Task::Ptr _tsk, 
                       Expression<Frame>::Ptr _o1, 
                       Expression<Frame>::Ptr _o2,
                       const std::vector<int>& _listOfVariableNumbers,
                       int _time_ndx);

        /**
         * calls the underlying task and fills in the appropriate member variables
         * of iTaSC_Adaptor ( to be used by an OutputExpression and a WeightExpression)
         * Main computational burden lies within this method.
         */
        virtual void compute();

        /**
         * returns the number of outputs
         */
        virtual int getNrOfOutputs();
 

        /**
         * called by the output expressions. 
         */
        virtual double getDerivative(int output_number,int variable_number); 

        /**
         * called by the weight expressions.
         */
        virtual double getWeight(int output_number); 

        /**
         * get a clone for the iTaSC_Adaptor. 
         */
        virtual MIMO::Ptr clone();


};


/**
 * create_iTaSC_adaptor creates a shared pointer to an iTaSC adaptor object.
 * \param [in] _tsk a shared ptr to an iTaSC_Task object.
 * \param [in] _o1  a frame o1, expressed as an expression graph
 * \param [in] _o2  a frame o2, expressed as an expression graph
 * \caveat to make sense, _o1 and _o2 should be transformations expressed with respect to the same base,
 *         whatever this base is (it should not necessarily be the world frame)
 */
iTaSC_Adaptor::Ptr create_iTaSC_adaptor(
               iTaSC_Task::Ptr _tsk, 
               Expression<Frame>::Ptr _o1, 
               Expression<Frame>::Ptr _o2,
               const std::vector<int>& _listOfVariableNumbers,
               int time_ndx);


/**
 * iTaSC_Output is used to access the output of the iTaSC_Adaptor as an expression graph node.  
 */
class iTaSC_Output: public MIMO_Output<double> {
    public:
        typedef boost::shared_ptr<iTaSC_Output> Ptr;
        int                output_number;

        iTaSC_Output() {}
        iTaSC_Output( MIMO::Ptr _adaptor, int _output_number):
                    MIMO_Output<double>("iTaSC_Output",_adaptor),
                    output_number(_output_number) {}

        double value() {
            iTaSC_Adaptor::Ptr p = boost::static_pointer_cast<iTaSC_Adaptor>(mimo);
            p->compute();
            return 0.0;
        }

        double derivative(int i){
            iTaSC_Adaptor::Ptr p = boost::static_pointer_cast<iTaSC_Adaptor>(mimo);
            p->compute();
            #ifdef PRINT_MATRICES
                std::cerr << "iTaSC_Output::derivative("<<i<<"):"<< p->getDerivative(output_number,i) << std::endl;
            #endif
            return p->getDerivative(output_number,i); 
        }

        MIMO_Output::Ptr clone() {
            iTaSC_Output::Ptr tmp( new iTaSC_Output(getMIMOClone(),output_number));
            return tmp;
        }
};


/**
 * creates an iTaSC_Output that is used to access the output of the iTaSC_Adaptor as an expression graph node.  
 */
iTaSC_Output::Ptr create_itasc_output(iTaSC_Adaptor::Ptr _adaptor, int _output_number);

/**
 * iTaSC_Weight is used to access the output of the iTaSC_Adaptor as an expression graph node.  
 */
class iTaSC_Weight: public MIMO_Output<double> {
    public:
        typedef boost::shared_ptr<iTaSC_Weight> Ptr;
        int                output_number;

        iTaSC_Weight() {}
        iTaSC_Weight( MIMO::Ptr _adaptor, int _output_number):
                    MIMO_Output<double>("iTaSC_Weight",_adaptor),
                    output_number(_output_number) {}

        double value() {
            iTaSC_Adaptor::Ptr p = boost::static_pointer_cast<iTaSC_Adaptor>(mimo);
            p->compute();
            return p->getWeight(output_number);
        }

        double derivative(int i){
            iTaSC_Adaptor::Ptr p = boost::static_pointer_cast<iTaSC_Adaptor>(mimo);
            p->compute();
            return 0.0; 
        }

        MIMO_Output::Ptr clone() {
            iTaSC_Weight::Ptr tmp( new iTaSC_Weight(getMIMOClone(),output_number));
            return tmp;
        }
};


/**
 * creates an iTaSC_Weight that is used to access the output of the iTaSC_Adaptor as an expression graph node.  
 */
iTaSC_Weight::Ptr create_itasc_weight(iTaSC_Adaptor::Ptr _adaptor, int _output_number);

/**
 * \param [in] ctx Context to add the task to.
 * \param [in] name name of the task.
 * \param [in] tsk  the iTaSC task at hand. 
 * \param [in] o1   object and frame o1 (as an expression graph).
 * \param [in] o2   object and frame o2 (as an expression graph). o1 and o2 should be expressed with respect to the same frame.
 * \param [in] listOfVarNames list of variable names that indicate the meaning of the elements of q and columns of Cq.
 * \param [in] task_weight weight for the current task ( as an expression graph).
 * \param [in] priority priority of the constraints added by this task.
 * \returns 0 if the task was successfully added.  returns -1 if unknown variable names were used, or -2 if it was not
 *          possible to add the constraints (e.g. because of an already existing name for the constraints)
 */
int addTask( Context::Ptr ctx, const std::string& name,
              iTaSC_Task::Ptr tsk,
              Expression<Frame>::Ptr o1, Expression<Frame>::Ptr o2,
              const std::vector<std::string>& listOfVarNames,
              Expression<double>::Ptr task_weight,
              int priority);

/**
 * \param [in] ctx Context to which the task belong
 * \param [in] task_name Name of the task (note: not the name of every single constraint)
 */
int removeTask( Context::Ptr ctx, const std::string& task_name, unsigned int num_constraint );

}// namespace KDL

#endif
