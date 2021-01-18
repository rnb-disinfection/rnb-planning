#include <ros/ros.h>
#include <expressiongraph/context.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/outputs_matlab.hpp>
#include <expressiongraph/outputs_ros.hpp>
#include <expressiongraph/outputs_ros_lines.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/defaultobserver.hpp>
#include <stdlib.h>

using namespace KDL;
using namespace Eigen;
using namespace std;


int main(int argc, char* argv[]) {
    ros::init(argc,argv,"script_ros");
    ros::NodeHandle nh;
    std::string fname;
    if (argc!=2) {
        cerr << " This program requires a configuration script as argument"<< endl;
        return -1;
    } else {
        fname = argv[1]; 
    }
    Context::Ptr ctx = create_context();
    // set-up necessary for robot control problems:
    // time and type time are defined by default.
    ctx->addType("robot");
    ctx->addType("feature");
    
    LuaContext LUA; 
    LUA.initContext(ctx);
    int retval = LUA.executeFile(fname);
    if (retval !=0) {
        return -2;
    }

    //OutputGenerator::Ptr outgen =  create_matlab_output(cout,"matlab");

    OutputGenerator::Ptr outgen =  create_ros_output("robot_description","ros_","base_link",ctx,nh);
    outgen =  create_ros_lines_output("roslines_","base_link",nh,outgen);

    // =========================================================================================
    // execution, with output and monitoring
    // =========================================================================================
    double max_iterations      = floor(ctx->getSolverProperty("max_iterations",300));    //  
    double max_cpu_time        = ctx->getSolverProperty("max_cpu_time",0.0);    //  
    double sample_time         = ctx->getSolverProperty("sample_time",0.01);    //  
    double regularization      = ctx->getSolverProperty("regularization",1E-4); // regularization factor.
    double initialization_time = ctx->getSolverProperty("initialization_time",3); 
    qpOASESSolver solver(max_iterations,max_cpu_time, regularization);
    Observer::Ptr obs = create_default_observer(ctx,"exit"); 
    ctx->addDefaultObserver( obs );
    // very simple initialization, without stop criterion.
    cerr << "started initialization : " << endl;
    solver.prepareInitialization(ctx);
    for (double t=0;t<initialization_time;t+=sample_time) {
        solver.updateStep(sample_time);
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
    ros::Rate loop_rate(1/sample_time);
    ctx->resetMonitors();
    ctx->clearFinishStatus();
    do {
        int retval = solver.updateStep(sample_time);
        ctx->checkMonitors();
        if (retval!=0) {
            cerr << "solved encountered error :\n";
            cerr << solver.errorMessage(retval)  << endl;
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

