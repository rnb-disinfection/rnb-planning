/**
 * An example of a task directly written in C++, without using the LUA scripting.
 * This example performs the same task as the two-cylinders2.lua example and generates
 * outputs for visualization using ROS RViz (leaving out Matlab output).
 * The structure of the LUA file is closely mapped, so, if some of the LUA libraries
 * for task definitions are used, this is mapped as a separate C++ function.
 */
#include <ros/ros.h>
#include <expressiongraph/context.hpp>
#include <expressiongraph/context_aux.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/outputs_matlab.hpp>
#include <expressiongraph/outputs_ros.hpp>
#include <expressiongraph/outputs_ros_lines.hpp>
//#include <expressiongraph_tf/context_scripting.hpp>
#include <expressiongraph/urdfexpressions.hpp>
#include <expressiongraph/defaultobserver.hpp>
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
  //  despos["arm_arm_3_joint"]    =      -0;
    despos["arm_arm_2_joint"]    =      0;
  // despos["hand_finger_12_joint"]  =      1;
 //  despos["hand_thumb_2_joint"]  =      1;
return despos;
}

// define a start position
std::map<std::string,double> getStartPosition()
{
	std::map<std::string,double> pos;
	pos["arm_arm_0_joint"]    =      0.7;
	pos["arm_arm_1_joint"]    =      -1.5;
	pos["arm_arm_2_joint"]  =      0.0;
	pos["arm_arm_3_joint"]    =     1.0;
	pos["arm_arm_4_joint"]=      0.0;
	pos["arm_arm_5_joint"] =      0.0;
	pos["arm_arm_6_joint"]  =      0;
	pos["hand_finger_12_joint"]    =   -1.5;
	pos["hand_finger_13_joint"]=        0.0;
	pos["hand_finger_22_joint"]  =      -0.5;
	pos["hand_finger_23_joint"] =       0.0;
	pos["hand_thumb_2_joint"]    =     -1.5;
	pos["hand_thumb_3_joint"]  =          0;

	return pos;
}

// for all the given joint values, define a constraint on each joint to the given value (with a given K and weight)
void desired_joint_positions(Context::Ptr ctx, const std::string& name, const std::map<std::string,double>& jval, double K, double weight) {
    for ( std::map<std::string,double>::const_iterator it = jval.begin(); it!= jval.end(); ++it) {
        Expression<double>::Ptr e = ctx->getScalarExpr(it->first);
        if (!e) {
            std::cerr << "could not find variable : " << it->first << std::endl;
           // exit(1);
        }
        else
        	CHECK( addConstraint(ctx, name+":"+it->first, e, K, Constant(weight),2) == 0 );
    }
}

// add a pointer to expression representing six joints of the hand
 bool get_hand_joint(Context::Ptr ctx,std::vector<Expression<double>::Ptr>& joint_hand_exp_vec)
{
	std::vector<std::string> jname(6);
	jname[2]="hand_finger_12_joint";
	jname[3]="hand_finger_13_joint";
	jname[0]="hand_finger_22_joint";
	jname[1]="hand_finger_23_joint";
	jname[4]="hand_thumb_2_joint";
	jname[5]="hand_thumb_3_joint";



	for (unsigned int i=0;i<jname.size();i++)
	{
		Expression<double>::Ptr e = ctx->getScalarExpr(jname[i]);
		if (!e)
		{
			std::cerr << "could not find variable : " << jname[i] << std::endl;
			// exit(1);
			return false;
		}
		else
			joint_hand_exp_vec.push_back(e) ;

	}
	return true;
}


         
// defines the robot, initial position, and constraints on joint space
ExpressionMap kuka_shunk_robot(Context::Ptr ctx, double velscale) {
    UrdfExpr3 urdf(10.0,velscale);
    
//DefaultUrdfJointsAndConstraintsGenerator::DefaultUrdfJointsAndConstraintsGenerator(double _vel_scale, double _K_limit) {
    urdf.readFromFile("pma-lwr-with-sch.urdf");

    /*addTransform gives a name (first arg.)
     *  to the transformations between base_link (3° arg.) and the
     * left/right gripper tool frames ("second arg")*/

    urdf.addTransform("Cc_T_tip","hand_tip_link","glass_link");
   // CHECK( urdf.addTransform("Cc_T_tip","hand_thumb_3_link","glass_link")  );
    urdf.addTransform("Cc_T_palm","hand_palm_link","glass_link");

    //adding transformation for all the kinematic tree, so that all the joints are added
    urdf.addTransform("Cc_T_finger1","hand_finger_13_link","glass_link");
    urdf.addTransform("Cc_T_finger2","hand_finger_23_link","glass_link");
    urdf.addTransform("Cc_T_finger3","hand_thumb_3_link","glass_link") ;
    ExpressionMap r = urdf.getExpressions(ctx);

    //needed only in simulation, set initial position
    ctx->setInitialScalarValues( getStartPosition() );

    /*
     * very low weight task to avoid drifting of the robot, when other tasks
     * are not fully constraining the robot-> arms in front position
    */
    std::map<std::string,double> despos;

   //desired_joint_positions(ctx,"despos", getDesiredPosition(), 1.0, 1E-6);

    return r;
}


Expression<Frame>::Ptr translate_z(double d) {
    return Constant( Frame(Vector(0,0,d)) ); 
} 

enum state_list {approacing, closing, closed};

int main(int argc, char* argv[]) {
    // ======================================================================================== 
    // Task level configuration (i.e. task definition)
    // ======================================================================================== 
    Context::Ptr ctx = create_context();

    // set-up necessary for robot control problems:
    // time and type time are defined by default.
    ctx->addType("robot");
    ctx->addType("feature");

    ExpressionMap r = kuka_shunk_robot(ctx,0.15);


    //double radius_cyl= 0.03;

   // Expression<Frame>::Ptr cylR = r["Cc_T_tip"]*translate_z(0.15);
   // Expression<Frame>::Ptr cylL = r["Cc_T_tip"]*translate_z(0.15);
  
    Expression<double>::Ptr f1 = 
        ctx->addScalarVariable("along cyl. axis","feature",0.1, Constant(1.0));

    Expression<Vector>::Ptr p_cyl =  KDL::vector(f1, Constant(0.0), Constant(0.0));
    Expression<Vector>::Ptr p_ee_or = origin( r["Cc_T_tip"]);
    Expression<Vector>::Ptr p_hand_palm = origin( r["Cc_T_palm"]);
    Expression<Vector>::Ptr p_tip = p_cyl-p_ee_or;
    Expression<Vector>::Ptr p_palm = p_cyl-p_hand_palm;
    Expression<Vector>::Ptr p_versor_tip = p_tip*(Constant(1.0)/norm(p_tip));
    Expression<Vector>::Ptr p_versor_palm = p_palm*(Constant(1.0)/norm(p_palm));



    //definition of features coordinate
    Expression<double>::Ptr dot_P_Z_cy
    = dot(p_versor_palm, KDL::vector(Constant(1.0), Constant(0.0),Constant(0.0) ));

    CHECK( addConstraint(ctx,"perpendicular(z axis cyl, distance vector)",
    		dot_P_Z_cy
    		, 10.0, 1.0, 0) ==0 );


    //monitored outputs
        Expression<double>::Ptr distance = norm(p_tip);
        CHECK( addConstraint(ctx,"distance",
        		distance,// -Constant(0.22),
        		0.5, 1.0, 2)  ==0 );


        //parallel(distance vector, palm z axis)
        Expression<double>::Ptr angle_z_palm__f1
        =acos(dot(unit_z( rotation(r["Cc_T_palm"])),p_versor_palm));//

        CHECK( addConstraint(ctx,"z_ee toward f1",
        		angle_z_palm__f1,
        		1.0, 1.0, 2) ==0 );



        //rot angle along z
          Expression<double>::Ptr angle_y_tip__x_cyl
    =dot(unit_y( rotation(r["Cc_T_palm"])),
    		KDL::vector( Constant(1.0), Constant(0.0), Constant(0.0)));
    CHECK( addConstraint(ctx,"y_ee up",
    		angle_y_tip__x_cyl-Constant(1.0),
       	1.0, 1.0, 2) ==0 );




    //desired f1 position

    Expression<double>::Ptr f1_ref=Constant(0.0);//*sin(ctx->time*Constant(2.0*M_PI/10.0));

  /*  CHECK( ctx->addConstraint("desired_height",
    		f1 - f1_ref,
    		10.0, 0.01, 2)  ==0 )*/
    CHECK( addInequalityConstraint(ctx,"ine_f1", f1,-0.1, 10.0,0.1, 10.0, 1, 2) ==0 )
    //put some excitation into the task:
    /*Expression<double>::Ptr a_joint = ctx->getScalarExpr("arm_arm_2_joint");
    CHECK( ctx->addConstraint("joint_some_mov",
    		a_joint+Constant(0.1)*sin(ctx->time*Constant(2.0*M_PI/3.0))
                       - Constant(0.5), 
                       10.0, 0.01, 2) ==0 );*/
    
    //SINERGY !!!!
   std::vector<Expression<double>::Ptr> joint_hand_exp_vec;
    CHECK( get_hand_joint(ctx, joint_hand_exp_vec));
 cerr<<"size must be six, and is: "<<joint_hand_exp_vec.size()<<endl;

    //Expression<double>::Ptr s_tip_des=sin(ctx->time*Constant(2.0*M_PI/3.0));

    std::vector<int> ndx;
     int time_ndx = ctx->getScalarNdx("time");
     ndx.push_back( time_ndx );
         // construct variable :
     VariableType<double>::Ptr s_tip_des = Variable<double>(ndx);
     //  or you can also write: VariableType<double>::Ptr my_input = Variable(time_ndx,1);
         // give the variable a value:
     s_tip_des->setValue(0);
         // and a derivative (towards time) :
         // this sets the 0-th column of the Jacobian to 0.0.  The first argument is NOT a variable index,
         // it corresponds to the ndx-variable you have given during construction:
     s_tip_des->setJacobian(0,0.0);
         // NOTE:  typically for input variables comming from sensors, you do not always have information on
         //        the derivatives.  In this case, you still make it dependant on time (because it changes
         //        when the time changes), but leave the Jacobian set to zero.

  CHECK( addConstraint(ctx,"joint_22_s",
    		joint_hand_exp_vec[0]-(s_tip_des*Constant(M_PI/2.0)-Constant(M_PI/2.0)),
                           10.0, 1, 2) ==0 );
       CHECK( addConstraint(ctx,"joint_12_s",
    		joint_hand_exp_vec[2]-(s_tip_des*Constant(M_PI/2.0)-Constant(M_PI/2.0)),
                           10.0, 0.01, 2) ==0 );
       CHECK( addConstraint(ctx,"joint_32_s",
           		joint_hand_exp_vec[4]-(s_tip_des*Constant(M_PI/2.0)-Constant(M_PI/2.0)),
                                  10.0, 0.01, 2) ==0 );
//pose of the elbow
       Expression<double>::Ptr j_1 = ctx->getScalarExpr("arm_arm_1_joint");
       Expression<double>::Ptr j_3 = ctx->getScalarExpr("arm_arm_3_joint");
       Expression<double>::Ptr elbow_pose= j_1-j_3;
       CHECK( addConstraint(ctx,"elbow_pose_cst",
    		   elbow_pose,10.0, 0.0001, 2) ==0 );

    ctx->addMonitor( "fixed_duration",  ctx->time ,-1.0 ,30.0, "exit","");

    cerr<<ctx<<endl;


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

    std::ofstream f("kuka_data.m");//it happens that the file is written in $HOME/.ros/data.m if lauchfile is used
     ctx->addOutput<double>("time", "matlab",ctx->time);
     ctx->addOutput<double>("f1","matlab",f1);
     ctx->addOutput<double>("f1_ref","matlab",f1_ref);
     ctx->addOutput<double>("distance","matlab",distance);
     ctx->addOutput<double>("dot_P_Z_cy","matlab",dot_P_Z_cy);
     ctx->addOutput<double>("angle_z_palm__f1","matlab",(angle_z_palm__f1));
     ctx->addOutput<double>("angle_y_tip__x_cyl","matlab",angle_y_tip__x_cyl);
     for(unsigned int i=0;i<6;i++)
     {
    	 std::ostringstream s;
    	 s << "joint" << i;
    	 ctx->addOutput<double>(s.str(),"matlab",joint_hand_exp_vec[i]);
     }
     ctx->addOutput<double>("s_tip_des","matlab",s_tip_des);
     ctx->addOutput<double>("elbow_pose","matlab",elbow_pose);
     OutputGenerator::Ptr outmat =  create_matlab_output(f,"matlab");


    // =========================================================================================
    // Task execution, with output and monitoring (boilerplate code, task independent)
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
    outmat->init(ctx);
    ros::Rate loop_rate(1.0/sample_time);
    state_list state=approacing;
    double new_s=0;
    //double new_s=0;
    ctx->resetMonitors();
    ctx->clearFinishStatus();
    do {
    	switch (state) {
    	case approacing:
//only checks leaving conditions
    		if (distance->value()<0.005)
    		{
    			state=closing;
    			cerr<<"Approaching is ended"<<endl;
    		}
    		break;
    	case closing:
    		 new_s=s_tip_des->value()+sample_time/10;
    		if (new_s>1)
    		{
    			new_s=1;
    			state=closed;
    			cerr<<"closing is ended"<<endl;
    		}
    	     s_tip_des->setValue(new_s);
    	     //s_tip_des->setJacobian(0,0.0);

    		break;
    	case closed:

    		break;
    	default:
    		break;
		}


        int retval = solver.updateStep(sample_time);
        ctx->checkMonitors();



        if (retval!=0) {
            cerr << "solved encountered error :\n";
            cerr << solver.errorMessage(retval)  << endl;
            cerr << ctx << endl;
            return -3;
        }
        if (ctx->getFinishStatus() ) {
            cerr << "Program is finished "<< endl;
            break;
        }
        if (!ros::ok()) {
            break;
        }
        outgen->update(ctx);
        outmat->update(ctx);
        loop_rate.sleep();
    } while (true);

    outgen->finish(ctx);
    outmat->finish(ctx);
    cerr << "finished execution  " << endl;
    return 0;
}

