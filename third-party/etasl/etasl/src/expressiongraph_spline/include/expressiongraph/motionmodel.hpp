#ifndef EXPRESSIONGRAPH_MOTION_MODEL_HPP
#define EXPRESSIONGRAPH_MOTION_MODEL_HPP
/**
 * reads in a motion model according to:
 *  Learning a Predictive Model of Human Gait for the Control of a Lower-limb Exoskeleton,
 *  Erwin Aertbelien, Joris De Schutter, 5th IEEE RAS/EMBS International Conference on
 *  Biomedical Robotics and Biomechatronics, August 12-15, 2014, Anhembi Convention Center in São Paulo, Brazil
 *
 *  E. Aertbelien (feb. 2015)
 */

#include <expressiongraph/cubicspline.hpp>
#include <boost/make_shared.hpp>

namespace KDL {

    class MotionModel {
            int type;  // non-periodic: 0, periodic : 1
            unsigned int m;     // number of modes (size of x_star)
            unsigned int nrOfJoints; // number of joints to use. 
            CubicSpline::Ptr cspline;
        public:
            typedef boost::shared_ptr<MotionModel> Ptr; 

            unsigned int getNrOfJoints() const { return nrOfJoints;}
          
            unsigned int getNrOfModes() const { return m;}

            int getType() const { return cspline->getType();}

            /**
             * After reading the model, this will contain expressions for the nrOfJoint elements of h (cf. eq.15).
             */
            std::vector< Expression<double>::Ptr > outputs;

            /**
             * Constructor
             * \param type [in] : specifies the type of interpolation to use for H and b: non-periodic=0 (CubicSpline::NATURAL), periodic=1 (CubicSpline::PERIODIC)
             * \param nrOfJoints [in]:   the number of joints that are simultanuously measured. 
             */
           MotionModel(int _type, unsigned int _nrOfJoints);

            /**
             * reads in a csv file that describes the motion model using spline interpolation points
             * (see eq. 15 of the paper). We have a spline for each element of \f$\mathbf{h}\f$ (i.e. each joint in the model)
             * and corresponding to  each element of \f$x^*\f$
             * * the first column contains the progress variable s
             * * the subsequent columns contain for each mode, the spline corresponding to the different joints, i.e.  first mode 1 of joint 1, then  mode 1 of joint 2, then mode 1 of joint 3,....
             *   and only then mode 2 of joint 1, etc....
             * * the last columns contains the b vector for joint 1, joint 2 , ....
             * * the number of modes is deduced from the number of columns in the file, as well as the number of interpolation points, which is deduced from
             *   the number of rows.
             * * the variable index numbers (for setInputValue and derivative(..) ) are arranged as follows:
             *     * \f$ t  = 0      \f$
             *     * \f$ v  = 1      \f$
             *     * \f$ s0 = 2      \f$
             *     * \f$ x^*_0 = 3   \f$
             *     * \f$ \ldots      \f$
             *     * \f$ x^*_{m-1} = m+2 \f$
             *
             * \param fn [in]  : filename
             * \param sep [in] : separators to separate columns (e.g. ",", multiple characters can be used if there are different possibilities)
             * \param skipheader  [in] : the number of rows the skip (typically because of an header)
             * \param velmodel   [in] : if true, the computed h and H will be for velocity measurements of the joint angles.
             *                   (velmodel=true is untested...)
             *
             * This method returns the following values:
             * * 0: success
             * * -1 if file could not be read, 
             * * -2 if the rows do not have the same amount of columns, 
             * * -3 if the file does not have at least two columns (for x and for y), 
             * * -4 if spline preparation failed, e.g. because the x points where not in ascending order, 
             * * -5 if you are extending existing data with a different amount of columns.
             * * -6 number of columns is inconsistent with nrOfJoints parameter.
             **/
           int readModel(const std::string& fn, const std::string& sep, int skipheader, bool velmodel=false);

            /**
             * set the input for this model. Cf. paper for the meaning of these variables.
             **/
           void setInputValues( double t, double v, double s0, const Eigen::VectorXd& x_star);


            /**
             * evaluate the model for the current input variables
             *
             * \param h [out] will contain the evaluation of the model (i.e. of eq. 15). Size of h is #modes.
             *
             * \warning should only be called after a call to setInputValues(...)
             */
            void value( Eigen::VectorXd& h );
            /**
             * evaluate the Jacobian of the model towards the input variables [v, s0, x_star] 
             * (in this order).  
             *
             * \param H [out] will contain the evaluation of the Jacobian (more or less eq. 16,
             *                except for the order of the variables). Size of H is #joints x (#modes+2)
             *
             * \warning should only be called after a call to value(...) !
             */
            void jacobian( Eigen::MatrixXd& H );

    };

    inline MotionModel::Ptr create_motion_model( int type, unsigned int nrofjoints) {
        MotionModel::Ptr tmp( new MotionModel(type,nrofjoints));
        return tmp;
    }

} // namespace KDL

std::ostream& operator << (std::ostream& os, const KDL::MotionModel& m);


#endif
