#include <expressiongraph/context.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/outputs_matlab.hpp>
#include <expressiongraph/outputs_csv.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/defaultobserver.hpp>
#include <expressiongraph/breakobserver.hpp>
#include <boost/make_shared.hpp>
#include <stdlib.h>
#include <boost/timer.hpp>

using namespace KDL;
using namespace Eigen;
using namespace std;


int main(int argc, char* argv[]) {
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

    LuaContext::Ptr lua( boost::make_shared<LuaContext>() ); 
    // define a variable that only depends on time
    lua->initContext(ctx);
    int retval = lua->executeFile(fname);
    if (retval !=0) {
        return -2;
    }
    ofstream of_csv("output.csv");
    OutputGenerator::Ptr outgen2 = create_csv_output(of_csv,"csv");
    OutputGenerator::Ptr outgen =  create_matlab_output(cout,"matlab",outgen2);
    ofstream of("debug_init.m");
    OutputGenerator::Ptr outgen_init =  create_matlab_output(of,"matlab");

    // =========================================================================================
    // execution, with output and monitoring
    // =========================================================================================
    double max_iterations      = floor(ctx->getSolverProperty("max_terations",300));    //  
    double max_cpu_time        = ctx->getSolverProperty("max_cpu_time",0.0);    //  
    double sample_time         = ctx->getSolverProperty("sample_time",0.01);    //  
    double regularization      = ctx->getSolverProperty("regularization",1E-4); // regularization factor.
    double initialization_time = ctx->getSolverProperty("initialization_time",3); 
    qpOASESSolver solver(max_iterations,max_cpu_time, regularization);
    Observer::Ptr obs = create_break_observer(lua);
    obs = create_default_observer(ctx,"exit",obs); 
    ctx->addDefaultObserver( obs );
    // very simple initialization, without stop criterion.
    boost::timer timer;
    cerr << "started initialization : " << endl;
    solver.prepareInitialization(ctx);
    outgen_init->init(ctx);
    for (double t=0;t<initialization_time;t+=sample_time) {
        int retval = solver.updateStep(sample_time);
        if (retval!=0) {
            cerr << "solved encountered error during initialization (t="<<t<< "):\n";
            cerr << solver.errorMessage(retval)  << endl;
            cerr << ctx << endl;
            return -3;
        }
        outgen_init->update(ctx);
    }
    outgen_init->finish(ctx);
    solver.setInitialValues();
    cerr << "Initialization time("<< ceil(initialization_time/sample_time) <<"  iterations) : " << timer.elapsed() << "[s]" << endl;
    cerr << "started execution  " << endl;
    solver.prepareExecution(ctx);
    outgen->init(ctx);
    double t=0;
    timer.restart();
    ctx->resetMonitors();
    ctx->clearFinishStatus();
    do {
        int retval = solver.updateStep(sample_time);
        ctx->checkMonitors();
        if (retval!=0) {
            cerr << "solved encountered error (t=" << t << "):\n";
            cerr << solver.errorMessage(retval)  << endl;
            cerr << ctx << endl;
            return -3;
        }
        if (ctx->getFinishStatus()) {
            cerr << "Program is finished "<< endl;
            break;
        }
        outgen->update(ctx);
        t+=sample_time;
    } while (true);
    outgen->finish(ctx);
    double exectime=timer.elapsed();
    cerr << "execution time("<< ceil(t/sample_time) << " iterations) : " << exectime << "[s], \nexecution time for 1 update : " << (exectime/t*sample_time)*1000.0 << " [ms]"<< endl;
    cerr << "stated times includes writing output data \n";
    cerr << "finished execution  " << endl;
    return 0;
}

