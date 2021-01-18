#ifndef EXPRESSIONGRAPH_ITASC_TASK
#define EXPRESSIONGRAPH_ITASC_TASK

#include <boost/shared_ptr.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

namespace KDL {

/**
 * iTaSC_Task encapsulates an iTaSC-style class ( i.e. the combination
 * of a VKC, CC and a trajectory generator).  iTaSC is explained in ref(2).
 *
 * This documentation uses the geometric semantics specification to specify the exact semantics
 * of its input/ouput variables. (see ref. 1)
 * 
 * (1) De Laet, T., Bellens, S., Smits, R., Aertbeliën, E., Bruyninckx, H., De Schutter, J., "Geometric Relations between Rigid Bodies: Semantics for Standardization", 
 * IEEE Robotics & Automation Magazine, vol. 20, no. 1, 2013, pp. 84-93.
 *
 * (2) De Schutter, J., De Laet, T., Rutgeerts, J., Decré, W., Smits, R., Aertbeliën, E., Claes, K., Bruyninckx, H., "Constraint-based task specification and estimation for sensor-based robot systems in the presence of geometric uncertainty", International Journal of Robotics Research, vol. 26, no. 5, 2007, pp. 433-455.
 *
 * \TODO further documentation of what an iTaSC-style class is.
 */
class iTaSC_Task {
    public:
        /**
         * type of a shared pointer to this class.
         */
        typedef boost::shared_ptr<iTaSC_Task> Ptr;

        /**
         * sets the number of joints that this task can handle to n.
         */
        virtual void setNrOfJoints(int n)=0;

        /**
         * gets the number of outputs that this task gives.
         */
        virtual int getNrOfOutputs()=0;

        /**
         * Computes 1 update step of the iTaSC task.
         * The CALLER preallocates the output matrices/vectors Jf,Cf,Cq,ydot_d_0, Wy to the
         * correct dimensions.
         *
         * The parameter declarations use the variables nrofjoints, nroffeatures with the 
         * following meaning  :
         *  - nrofjoints : the number of joints that this class is _explicitly_ using.  This can be
         *                 smaller than the actual number of joints involved.  There can be (implicit) dependencies on other joints
         *                 due to the expression graphs o1 and o2. This is something
         *                 determined by the designer of the iTaSC_Task subclass.
         *  - nroffeatures : for the iTaSC_Adaptor always 6.
         *  - nrofoutputs  : choosen by the iTaSC_Task subclass designer. The number of control outputs (y in iTaSC terminology).
         *
         * \warning in the implementation of iTaSC adaptor nrofjoints signifies 
         *         the total number of joints
         *       (not only explicit, but also implicit dependencies, via o1 and o2), nrofjoints_partial
         *       then corresponds to the number of explciit joints.
         *
         * \param [in] T  RelPose(o2|o2,o1|o1,o1) (pose of o2 on body o2 wrt. o1 on body o1 expressed in o1).   
         * \param [in] t  RelTwist(o1|o1,o1,o2)  (ref.point o1 on object o1 expressed in o1)
         * \param [in] q  joint values. [ nrofjoints x 1]
         * \param [out] Jf The Jacobian of the loop closure towards the feature variables. 
         *                each column has the following semantics: RelTwist(o1|o2,o1,o1) (ref.point o1 on object o2 expressed in o1) 
         *                [ 6 x nroffeatures]
         * \param [out] Cf The Jacobian of the output equation towards the feature variables (also called the feature selection matrix).
         *                [ nrofoutputs x nroffeatures]         
         * \param [out] Cq The Jacobian of the output equation towards the joint variables (also called the joint selection matrix).
         *                [ nrofoutputs x nrofjoints ]
         * \param [out] ydot_d_0 The desired velocity for the given constraints.
         *                [ nrofoutputs x 1]
         * \param [out] Wy The weights to be applied to the outputs. (only diagonal elements, specified as a vector).
         *                [ nrofoutputs x 1]
         * \param [out] tu Ju*Chi_u_dot.  The twist caused by the change in the uncertainty variables. (if the task contains an estimator).
         */ 
        virtual void update(const Frame& T, 
                            const Twist& t, 
                            const Eigen::VectorXd& q,
                            Eigen::MatrixXd& Jf,      
                            Eigen::MatrixXd& Cf,         
                            Eigen::MatrixXd& Cq,          
                            Eigen::VectorXd& ydot_d_0,      
                            Eigen::VectorXd& Wy,             
                            Twist&    tu             
                      )=0;

        /**
         * creates a deep, virtual copy of this iTasc_Task object.
         */
        virtual iTaSC_Task::Ptr clone()=0;

        /**
         * classes with virtual methods should ALWAYS define a virtual destructor.
         */
        virtual ~iTaSC_Task() {}
};

}// namespace KDL

#endif

