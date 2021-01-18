#include <ros/ros.h>
#include <expressiongraph/context.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/outputs_matlab.hpp>
#include <expressiongraph/outputs_ros.hpp>
#include <expressiongraph/outputs_ros_lines.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/groupingobserver.hpp>
#include <stdlib.h>
#include <boost/timer.hpp>

using namespace KDL;
using namespace Eigen;
using namespace std;


bool initialize_feature_variables( Context::Ptr ctx, qpOASESSolver& solver, double initialization_time, double sample_time, double convergence_crit) {
    boost::timer timer;
    // initialization:
    if (solver.getNrOfFeatureStates() > 0 ) {
        cerr << "started initialization : " << endl;
        solver.prepareInitialization(ctx);
        double t;
        for (t=0;t<initialization_time;t+=sample_time) {
            int retval = solver.updateStep(sample_time);
            if (retval!=0) {
                cerr << "solved encountered error during initialization (t="<<t<< "):\n";
                cerr << solver.errorMessage(retval)  << endl;
                cerr << ctx << endl;
                return false;
            }
            if (solver.getNormChange() < convergence_crit) break; 
        }
        cerr << "Initialization time("<< ceil(t/sample_time) <<"  iterations) : " << timer.elapsed() << "[s]" << endl;
    }
    solver.setInitialValues();
    return true;
}


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
    double max_iterations      = floor(ctx->getSolverProperty("max_terations",300));    //  
    double max_cpu_time        = ctx->getSolverProperty("max_cpu_time",0.0);    //  
    double sample_time         = ctx->getSolverProperty("sample_time",0.01);    //  
    double convergence_crit    = ctx->getSolverProperty("convergence_criterion",1E-4);
    double regularization      = ctx->getSolverProperty("regularization",1E-4); // regularization factor.
    double initialization_time = ctx->getSolverProperty("initialization_time",3); 
    qpOASESSolver solver(max_iterations,max_cpu_time, regularization);
    Observer::Ptr obs = create_grouping_observer( ctx );
    ctx->addDefaultObserver( obs );
    // initialization, with stop criterion.
    initialize_feature_variables(ctx,solver, initialization_time, sample_time, convergence_crit);
    cerr << "started execution  " << endl;
    solver.prepareExecution(ctx);
    outgen->init(ctx);
    ros::Rate loop_rate(1/sample_time);
    Eigen::VectorXd state;
    boost::timer timer;
    ctx->resetMonitors();
    ctx->clearFinishStatus();
    do {
        int retval = solver.updateStep(sample_time);
        ctx->checkMonitors();
        outgen->update(ctx);
        if (retval!=0) {
            cerr << "solved encountered error :\n";
            cerr << solver.errorMessage(retval)  << endl;
            return -3;
        }
        // no reinitialisation procedure (yet) !!!!!
        if (ctx->is_activity_changed()) {
            double time = solver.getTime();
            solver.getState(state);
            initialize_feature_variables(ctx,solver, initialization_time, sample_time, convergence_crit);
            solver.prepareExecution(ctx);
            solver.setTime(time);
        }
        if (ctx-> getFinishStatus() ) {
             cerr << "finishing..." << endl;
             break;
        }
        if (!ros::ok()) {
            break;
        }
        loop_rate.sleep();
    } while (true);
    outgen->finish(ctx);
    double exectime=timer.elapsed();
    cerr << "execution time("<< ceil(solver.getTime()/sample_time) << " iterations) : " << exectime << "[s], \nexecution time for 1 update : " 
        << (exectime/solver.getTime()*sample_time)*1000.0 << " [ms]"<< endl;
    cerr << "stated times includes writing output data \n";
    cerr << "finished execution  " << endl;
    return 0;
}

