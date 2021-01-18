#include <expressiongraph/context.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/outputs_matlab.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/rfsm_observer.hpp>
#include <expressiongraph/breakobserver.hpp>
#include <stdlib.h>
#include <boost/timer.hpp>
#include <boost/make_shared.hpp>

using namespace KDL;
using namespace Eigen;
using namespace std;
using namespace luabind;


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
        }
        cerr << "Initialization time("<< ceil(t/sample_time) <<"  iterations) : " << timer.elapsed() << "[s]" << endl;
    }
    solver.setInitialValues();
    return true;
}



int main(int argc, char* argv[]) {
    std::string fname("");
    std::string fsm_fname("");
    if (argc==3) {
        fname     = argv[1]; 
        fsm_fname = argv[2];
        cerr << "specification file  : " << fname << endl;
        cerr << "fsm file            : " << fsm_fname << endl;
    } else {
        cerr << " This program requires a configuration script and possibly a fsm script as argument"<< endl;
        return -1;
    }

    // ========================================================================================
    // Loading specification of constraints:
    // ========================================================================================
 
    // set-up the context for a robot control problem: 
    Context::Ptr ctx = create_context();
    ctx->addType("robot");
    ctx->addType("feature");

    // set-up two different LUA context to prohibit sharing of variables
    // outside the context.
    LuaContext::Ptr lua_spec = boost::make_shared<LuaContext>(); 
    lua_spec->initContext(ctx);
    int retval = lua_spec->executeFile(fname);
    if (retval !=0) {
        cerr << "Error loading specification file" << endl;
        return -2;
    }
    

    // ========================================================================================
    // Preparing lua:
    // ========================================================================================
    LuaContext::Ptr lua_fsm  = boost::make_shared<LuaContext>();
    lua_fsm->initContext(ctx);
    // execute preample:
    retval = lua_fsm->executeString(
        "require('rfsm');\n"
        "require('rfsmpp');\n"
        "function pp(fsm) \n"
        "    print(rfsmpp.fsm2str(fsm)) \n"
        "end; \n"
        "function showeq(fsm)\n"
        "   res = rfsm.check_events(fsm)\n"
        "   if res>0 then\n"
        "       print(\"queue:  \" .. table.concat(utils.map(tostring, fsm._intq), ', '))\n"
        "   end\n"
        "end"
    );
    if (retval !=0) {
        cerr << "Error loading rfsm library" << endl;
        return -2;
    }
    object model,fsm,load,init, step,showeq,cstate,prettyprint,check_events;
    try {
        load  = globals(lua_fsm->L)["rfsm"]["load"];
        init  = globals(lua_fsm->L)["rfsm"]["init"];
        step  = globals(lua_fsm->L)["rfsm"]["step"];
        check_events  = globals(lua_fsm->L)["rfsm"]["check_events"];
    } catch( error& err) {
        cerr << "Error setting up rFSM: " << err.what() << endl;
        object error_msg( from_stack(err.state(), -1) );
        cerr << "error message : " << error_msg << endl;
        return -4;
    }
    try {
        model       = call_function<object>(load,fsm_fname);
        fsm         = call_function<object>(init,model);
        globals(lua_fsm->L)["fsm"] = fsm;
        prettyprint = globals(lua_fsm->L)["pp"];
        showeq  = globals(lua_fsm->L)["showeq"];
    } catch( error& err ) {
        cerr << "Error loading and initializing state machine : " << err.what() << endl;
        object error_msg( from_stack(err.state(), -1) );
        cerr << "error message : " << error_msg << endl;
        return -4;
    }
 
    // =========================================================================================
    // execution, with output and monitoring:
    // =========================================================================================
    double max_iterations      = floor(ctx->getSolverProperty("max_terations",300));    //  
    double max_cpu_time        = ctx->getSolverProperty("max_cpu_time",0.0);    //  
    double sample_time         = ctx->getSolverProperty("sample_time",0.01);    //  
    double convergence_crit    = ctx->getSolverProperty("convergence_criterion",1E-4);
    double regularization      = ctx->getSolverProperty("regularization",1E-4); // regularization factor.
    double initialization_time = ctx->getSolverProperty("initialization_time",3); 
    qpOASESSolver solver(max_iterations,max_cpu_time, regularization);

    Observer::Ptr observers = create_break_observer(lua_spec,create_rfsm_observer(ctx,lua_fsm));
    ctx->addDefaultObserver( observers );

    ofstream matlab_file("output.m");
    OutputGenerator::Ptr outgen =  create_matlab_output(matlab_file,"matlab");
    // =========================================================================================
    // scheduling  initialization and loop:
    // =========================================================================================
    initialize_feature_variables(ctx,solver, initialization_time, sample_time,convergence_crit);
    cerr << "started execution  " << endl;
    solver.prepareExecution(ctx);
    outgen->init(ctx);
    double t=0;
    boost::timer timer;
    Eigen::VectorXd state;
    ctx->clearFinishStatus();
    ctx->resetMonitors();
    while (true) {
        
        int retval = solver.updateStep(sample_time);
        if (retval!=0) {
            cerr << "solved encountered error (t=" << t << "):\n";
            cerr << solver.errorMessage(retval)  << endl;
            cerr << ctx << endl;
            return -3;
        }
        outgen->update(ctx);
        //lua_fsm->call_console();
        //call_function<void>(showeq,fsm);
        try {
            call_function<void>(showeq,fsm);
        } catch( error& err ) {
            cerr << "Error showing queue : " << err.what() << endl;
            object error_msg( from_stack(err.state(), -1) );
            cerr << "error message : " << error_msg << endl;
            return -4;
        }
 
        ctx->checkMonitors();
        bool idle = call_function<bool>(step,fsm,1);
        if (!idle) {
            call_function<void>(prettyprint, fsm);
        }
        if (ctx->getFinishStatus()) break;

        // no reinitialisation procedure (yet) !!!!!
        if (ctx->is_activity_changed() ) {
            ctx->update_active();
            double time = solver.getTime();
            solver.getState(state);
            initialize_feature_variables(ctx,solver, initialization_time, sample_time, convergence_crit);
            solver.prepareExecution(ctx);
            solver.setTime(time);
            ctx->resetMonitors();
            cerr << "change of group activation" << endl;
        }
        t+=sample_time;
    }
    outgen->finish(ctx);
    double exectime=timer.elapsed();
    cerr << "execution time("<< ceil(t/sample_time) << " iterations) : " << exectime << "[s], \nexecution time for 1 update : " << (exectime/t*sample_time)*1000.0 << " [ms]"<< endl;
    cerr << "stated times includes writing output data \n";
    cerr << "finished execution  " << endl;
    return 0;
}

