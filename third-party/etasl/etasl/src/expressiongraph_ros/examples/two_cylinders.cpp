/**
 * An example of a task directly written in C++, without using the LUA scripting.
 * This example performs the same task as the two-cylinders2.lua example and generates
 * outputs for visualization using ROS RViz (leaving out Matlab output).
 * The structure of the LUA file is closely mapped, so, if some of the LUA libraries
 * for task definitions are used, this is mapped as a separate C++ function.
 */
#include <ros/ros.h>
#include <expressiongraph_tf/context.hpp>
#include <expressiongraph_tf/context_aux.hpp>
#include <expressiongraph_tf/qpoases_solver.hpp>
#include <expressiongraph_tf/outputs_matlab.hpp>
#include <expressiongraph_tf/outputs_ros.hpp>
#include <expressiongraph_tf/outputs_ros_lines.hpp>
#include <expressiongraph_tf/defaultobserver.hpp>
#include <expressiongraph_tf/urdfexpressions3.hpp>
#include <stdlib.h>

using namespace KDL;
using namespace Eigen;
using namespace std;

// these macros are not necessary, but they give back an error message if you make some
// errors during definition.
#define CHECK( a ) \
    { bool result = a; \
      if (!result) { \
            std::cerr << __FILE__ << " : " << __LINE__ << " : FAILED: " #a << std::endl;\
            exit(1);\
    } \
}



// define a desired position
std::map<std::string,double> getDesiredPosition() {
    std::map<std::string,double> despos;
    despos["l_wrist_roll_joint"]    =      0.1;
    despos["l_wrist_flex_joint"]    =      0.1;
    despos["l_forearm_roll_joint"]  =      0.0;
    despos["l_elbow_flex_joint"]    =     -1.0;
    despos["l_upper_arm_roll_joint"]=      1.0;
    despos["l_shoulder_lift_joint"] =      0.0;
    despos["l_shoulder_pan_joint"]  =      0.5;
    despos["r_wrist_roll_joint"]    =      -0.1;
    despos["r_wrist_flex_joint"]    =      -0.1;
    despos["r_forearm_roll_joint"]  =       0.0;
    despos["r_elbow_flex_joint"]    =      -1.0;
    despos["r_upper_arm_roll_joint"]=      -1.0;
    despos["r_shoulder_lift_joint"] =      0.01;
    despos["r_shoulder_pan_joint"]  =      -0.5;
    return despos;
}

// define a start position
std::map<std::string,double> getStartPosition() {
    std::map<std::string,double> pos;
    pos["l_wrist_roll_joint"]    =      0.1;
    pos["l_wrist_flex_joint"]    =      -0.1;
    pos["l_forearm_roll_joint"]  =      0.01;
    pos["l_elbow_flex_joint"]    =     -0.7;
    pos["l_upper_arm_roll_joint"]=      1.0;
    pos["l_shoulder_lift_joint"] =      0.01;
    pos["l_shoulder_pan_joint"]  =      0.5;
    pos["r_wrist_roll_joint"]    =      -0.1;
    pos["r_wrist_flex_joint"]    =      -0.1;
    pos["r_forearm_roll_joint"]  =       0.0;
    pos["r_elbow_flex_joint"]    =      -1.0;
    pos["r_upper_arm_roll_joint"]=      -1.0;
    pos["r_shoulder_lift_joint"] =      0.01;
    pos["r_shoulder_pan_joint"]  =      -0.5;
    return pos;
}

// for all the given joint values, define a constraint on each joint to the given value (with a given K and weight)
void desired_joint_positions(Context::Ptr ctx, const std::string& name, const std::map<std::string,double>& jval, double K, double weight) {
    for ( std::map<std::string,double>::const_iterator it = jval.begin(); it!= jval.end(); ++it) {
        Expression<double>::Ptr e = ctx->getScalarExpr(it->first);
        if (!e) {
            std::cerr << "could not find variable : " << it->first << std::endl;
            exit(1);
        }
        CHECK( addConstraint(ctx,name+":"+it->first, e, K, Constant(weight),2) == 0 ); 
    }
}


         
// defines a typical use of a pr2 robot (including its initial position, and a low weight constraint imposing some desired position):
ExpressionMap pr2_robot(Context::Ptr ctx, double velscale) {
    UrdfExpr3 urdf(10.0,velscale);
    urdf.readFromFile("pr2.urdf");
    /*addTransform gives a name (first arg.) to the transformations between base_link (3° arg.) and the
     * left/right gripper tool frames ("second arg")*/
    urdf.addTransform("leftarm","l_gripper_tool_frame","base_link");
    urdf.addTransform("rightarm","r_gripper_tool_frame","base_link");
    ExpressionMap r = urdf.getExpressions(ctx);
    // redefine the tool frames, such that z-axis point outwards:
    r["leftarm"]  = r["leftarm"] * Constant( Frame(Rotation::RotY(M_PI/2),Vector(0.01,0,0)));
    r["rightarm"] = r["rightarm"] * Constant( Frame(Rotation::RotY(M_PI/2),Vector(0.01,0,0)));

    //needed only in simulation, set initial position
    ctx->setInitialScalarValues( getStartPosition() );

    /*
     * very low weight task to avoid drifting of the robot, when other tasks
     * are not fully constraining the robot-> arms in front position
    */
    std::map<std::string,double> despos;
    desired_joint_positions(ctx,"despos", getDesiredPosition(), 1.0, 1E-4);
    return r;
}


Expression<Frame>::Ptr translate_z(double d) {
    return Constant( Frame(Vector(0,0,d)) ); 
} 

int main(int argc, char* argv[]) {
    // ===========================================================================================
    // setting up context and variables:
    // ===========================================================================================

    Context::Ptr ctx = create_context();

    // set-up necessary for robot control problems:
    // time and type time are defined by default.
    ctx->addType("robot");
    ctx->addType("feature");

    // ======================================================================================== 
    // Task level configuration (i.e. task definition)
    // ======================================================================================== 
  
    ExpressionMap r = pr2_robot(ctx,0.15);

    double radiusL = 0.03;
    double radiusR = 0.03;

    Expression<Frame>::Ptr cylR = r["leftarm"]*translate_z(0.15);
    Expression<Frame>::Ptr cylL = r["rightarm"]*translate_z(0.15);
  
    Expression<double>::Ptr f1 = 
        ctx->addScalarVariable("along cyl. axis 1","feature",0.1, Constant(1.0)); 
    Expression<double>::Ptr f2 = 
        ctx->addScalarVariable("along cyl. axis 2","feature",0.1, Constant(1.0)); 
    
    Expression<Vector>::Ptr p1 = cylR * KDL::vector( Constant(0.0), Constant(0.0), f1);
    Expression<Vector>::Ptr p2 = cylL * KDL::vector( Constant(0.0), Constant(0.0), f2);
    
    CHECK( addConstraint(ctx,"perpendicular L", dot(p2-p1,unit_z(rotation(cylL))), 10.0, 1.0, 0) ==0 );
    CHECK( addConstraint(ctx,"perpendicular R", dot(p2-p1,unit_z(rotation(cylR))), 10.0, 1.0, 0) ==0 );
    CHECK( addConstraint(ctx,"distance", norm(p2-p1) - Constant(radiusL+radiusR) , 10.0, 1.0, 2)  ==0 );
   
    // with a low weight constraint : keep the cylinders together:
    Expression<Vector>::Ptr v = origin(cylR)-origin(cylL);
    CHECK( addConstraint(ctx,"two_cylinder_together_x", coord_x(v), 10.0, 0.01, 2) ==0 );
    CHECK( addConstraint(ctx,"two_cylinder_together_y", coord_y(v), 10.0, 0.01, 2) ==0 );
    CHECK( addConstraint(ctx,"two_cylinder_together_z", coord_z(v), 10.0, 0.01, 2) ==0 );
    //put some excitation into the task:
    Expression<double>::Ptr left_shoulder_joint = ctx->getScalarExpr("l_shoulder_pan_joint");
    CHECK( addConstraint(ctx,"left_shoulder_joint", 
                       left_shoulder_joint+Constant(0.1)*sin(ctx->time*Constant(2.0*M_PI/3.0)) 
                       - Constant(0.5), 
                       10.0, 0.01, 2) ==0 );
    
    Expression<double>::Ptr right_shoulder_joint = ctx->getScalarExpr("r_elbow_flex_joint");
    CHECK( addConstraint(ctx,"right_shoulder_joint",
                       right_shoulder_joint+Constant(0.2)*sin(ctx->time*Constant(3.0*M_PI/4.0)) 
                        - Constant(-1.25), 
                       10.0, 0.01, 2) ==0);


    ctx->addMonitor( "fixed_duration",  ctx->time ,-1.0 ,30.0, "exit","");



    // ======================================================================================== 
    // Application level configuration:
    // ======================================================================================== 
    ros::init(argc,argv,"two_cylinders");
    ros::NodeHandle nh;

    double max_iterations      = 300;  
    double max_cpu_time        = 0.0;  
    double sample_time         = 0.01;  
    double regularization      = 0.001;
    double initialization_time = 3;
    qpOASESSolver solver(max_iterations,max_cpu_time, regularization);
    Observer::Ptr obs = create_default_observer(ctx,"exit"); 
    ctx->addDefaultObserver( obs );
    OutputGenerator::Ptr outgen =  create_ros_output("robot_description","ros_","base_link",ctx,nh);
    outgen =  create_ros_lines_output("roslines_","base_link",nh,outgen);


    // =========================================================================================
    // Task execution, with output and monitoring (boilerplate code, task independant)
    // =========================================================================================

    // very simple initialization, without stop criterion.
    cerr << "started initialization : " << endl;
    solver.prepareInitialization(ctx);
    for (double t=0;t<initialization_time;t+=sample_time) {
        int retval=solver.updateStep(sample_time);
        if (retval!=0) {
            cerr << "solved encountered error during initialization (t="<<t<< "):\n";
            cerr << solver.errorMessage(retval)  << endl;
            cerr << ctx << endl;
            return -3;
        }
    }
    solver.setInitialValues();
    cerr << "started execution  " << endl;
    solver.prepareExecution(ctx);
    outgen->init(ctx);
    ros::Rate loop_rate(1.0/sample_time);
    ctx->resetMonitors();
    ctx->clearFinishStatus();
    do {
        int retval = solver.updateStep(sample_time);
        ctx->checkMonitors();
        if (retval!=0) {
            cerr << "solved encountered error :\n";
            cerr << solver.errorMessage(retval)  << endl;
            cerr << ctx << endl;
            return -3;
        }
        if (ctx->getFinishStatus()) {
            cerr << "Program is finished "<< endl;
            break;
        }
        if (!ros::ok()) {
            break;
        }
        outgen->update(ctx);
        loop_rate.sleep();
    } while (true);
    outgen->finish(ctx);
    cerr << "finished execution  " << endl;
    return 0;
}

