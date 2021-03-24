#ifndef EXPRESSIONGRAPH_URDFEXPRESSIONS_HPP
#define EXPRESSIONGRAPH_URDFEXPRESSIONS_HPP


#include <string>
#include <set>
#include <map>
#include <vector>

#include <kdl/frames_io.hpp>
#include <kdl/expressiontree.hpp>

#include <expressiongraph/context.hpp>
#include <boost/tuple/tuple.hpp>
#include <stdexcept>

namespace KDL {

struct link_property {
    link_property() {
        name="";
        mass=0;
        Ixx=0;
        Ixy=0;
        Ixz=0;
        Iyy=0;
        Iyz=0;
        Izz=0;
        origin = Frame::Identity();
    }
    std::string name;
    double mass;
    double Ixx;
    double Ixy;
    double Ixz;
    double Iyy;
    double Iyz;
    double Izz;
    Frame  origin; 
};


class UrdfExpressions3;

typedef std::map<std::string, Expression<Frame>::Ptr> ExpressionMap;


class UrdfJointsAndConstraintsGenerator {
    public:
        typedef boost::shared_ptr<UrdfJointsAndConstraintsGenerator> Ptr;
        /** 
         * if vel_lower > vel_upper no velocity limits.
         * if pos_lower > pos_upper no position limits.
         */
        virtual boost::tuple< Expression<double>::Ptr, std::string>
        create(Context::Ptr& ctx, const std::string& name, double vel_lower, double vel_upper, double pos_lower, double pos_upper,
               double effort_lower, double effort_upper, double dt) = 0;

        virtual ~UrdfJointsAndConstraintsGenerator() {}
};


class DefaultUrdfJointsAndConstraintsGenerator :
    public UrdfJointsAndConstraintsGenerator {
public:
    double vel_scale;
    double K_limit;

    DefaultUrdfJointsAndConstraintsGenerator(double _vel_scale, double _K_limit);

    virtual boost::tuple< Expression<double>::Ptr, std::string> 
    create(Context::Ptr& ctx, const std::string& name, double vel_lower, double vel_upper, double pos_lower, double pos_upper,
            double effort_lower, double effort_upper, double dt);

    virtual ~DefaultUrdfJointsAndConstraintsGenerator() {}
}; 

/**
 * read a urdf expression, specifies what you want to get out and return a std::map of expressions.
 */
class UrdfExpr3 {
    protected:
        boost::shared_ptr<UrdfExpressions3> reader;        
    private:
        int                                count;
        std::map<std::string, int>         name_to_number;
        UrdfJointsAndConstraintsGenerator::Ptr jc_gen;
    public:
        double dt;
        typedef boost::shared_ptr<UrdfExpr3> Ptr;

        UrdfExpr3(double _dt=0.001):
                dt(_dt),
                count(0),
            jc_gen( new DefaultUrdfJointsAndConstraintsGenerator(1.0, 4.0)),
            poslimits(true),
            vellimits(true),
            effortlimits(true) {}

        /**
         * you can turn of position limits by setting K_limits==0
         * you can turn of velocity limits by setting velocity_scale==0
         */
        UrdfExpr3(double _dt, double K_limits, double velocity_scale):
            dt(_dt),
            count(0),
            jc_gen( new DefaultUrdfJointsAndConstraintsGenerator(velocity_scale,K_limits)),
            poslimits(true),
            vellimits(true),
            effortlimits(true) {
                if (velocity_scale==0) vellimits = false;
                if (K_limits==0) poslimits = false;
            }

        UrdfExpr3(double _dt, UrdfJointsAndConstraintsGenerator::Ptr _jc_gen):
            dt(_dt),
            count(0),
            jc_gen(_jc_gen),
            poslimits(true),
            vellimits(true),
            effortlimits(true) {}
        
        bool poslimits;             ///< modifies behavior of read... functions: (default true) true if you want to enforce position limits. 
        bool vellimits;             ///< modifies behavior of read... functions: (default true) true if you want to enforce position limits..
        bool effortlimits;             ///< modifies behavior of read... functions: (default true) true if you want to enforce position limits..

        void setJointsAndConstraintsCallback( UrdfJointsAndConstraintsGenerator::Ptr _jc_gen) {
            jc_gen  = _jc_gen;
        }
      
        /**
         * reads an URDF  specified by a filename
         */ 
        void readFromFile( const std::string& filename );

        /**
         * reads an URDF specified by a ros parameter name
         */
        void readFromParam(const std::string& parameter);

        /**
         * reads an URDF description in a string
         * \param [in] xmlstring URDF specification of a robot.
         */
        void readFromString(const std::string& xmlstring);

        /**
         * adds a transformation to return with getExpressions().
         * First collecting all necessary transformations to return, allows for an efficient implementation
         * for the returned expressions.
         * \param [in] name that will be used in the std::map that getExpressions() returns.
         * \param [in] an expression of this link
         * \param [in] expressed with respect to this link
         * can throw std::invalid_argument
         */
        void addTransform(const std::string& name, const std::string& frame, const std::string& base);

        /**
         * \param [in] ctx Context to add the specifications and constraints.  The robot description is returned as
         *             a set of constraints added to ctx and a series of expressions of interest for later
         *             task specification. 
         */
        ExpressionMap getExpressions(Context::Ptr& ctx);
       
        /**
         * gets a list of link names:
         */ 
        std::vector<std::string> getLinkNames();

        /**
         * get a list of link properties.
         */
        void getAllLinkProperties(std::vector<link_property>& props);

        /**
         * get all joint names in the URDF file
         */
        void getAllUrdfJointNames(std::vector<std::string>& names);


        /**
         * get all joint names (possibly modified by the callback function), only the
         * joints that are really used.
         */
        void getAllJointNames(std::vector<std::string>& names);

        void getAllJointExpressions(std::vector<Expression<double>::Ptr>& exprs);

        friend std::ostream& operator << ( std::ostream& os, const UrdfExpr3& u);
};



} // namespace
#endif
