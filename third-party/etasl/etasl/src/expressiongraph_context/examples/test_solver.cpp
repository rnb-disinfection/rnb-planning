#include <expressiongraph_tf/context.hpp>
#include <expressiongraph_tf/qpoases_solver.hpp>
#include <expressiongraph_tf/outputs_matlab.hpp>
#include <expressiongraph_tf/controller.hpp>

using namespace KDL;
using namespace Eigen;
using namespace std;

void update_and_check( qpOASESSolver& s, double dt ) {
    int c=s.updateStep(dt);
    //s.printMatrices(cout);
    if (c!=0) {
        cerr << "solved encountered error : \n";
        cerr << s.errorMessage(c)  << endl;
    }
}



int main(int argc, char* argv[]) {
    Context::Ptr ctx = create_context();

    // set-up necessary for robot control problems:
    ctx->addType("robot");
    ctx->addType("feature");
    ctx->addType("time");
    ctx->addScalarVariable("time","time",0.0, Constant(1.0) );

    // define the problem: a 1-dof robot in 2d-space
    // not the best way, just an illustration
    double K = 4;
    Expression<double>::Ptr q     = ctx->addScalarVariable("q","robot", 0.0, Constant(1.0));
    Expression<double>::Ptr f     = ctx->addScalarVariable("f","feature",0.0, Constant(1.0));
    Expression<double>::Ptr L     = Constant(0.2);
    Expression<double>::Ptr ee_x  = (L+f)*cos(q);
    Expression<double>::Ptr ee_y  = (L+f)*sin(q);
    Expression<double>::Ptr des_x = Constant(0.2);
    Expression<double>::Ptr des_y = Constant(0.2);
    //ctx->addConstraint("tracking_x",  ee_x - des_x, K, 1.0, 1); 
    //ctx->addConstraint("tracking_y",  ee_y - des_y, K, 1.0, 1); 

    ControllerRegistry reg;
    reg.register_controller(create_controller_proportional());
    reg.register_controller(create_controller_proportional_saturated());
    Controller::Ptr c = reg.lookupPrototype("proportional");
    c->setParameter("K", Constant<double>(10.0));
    ctx->addInequalityConstraint("tracking_x", 
            ee_x - des_x,    ee_x - des_x, 
            0.0, 0.0, c->clone(), c->clone(), 
            Constant<double>(1.0), 1);

    ctx->addInequalityConstraint("tracking_y", 
            ee_y - des_y,    ee_y - des_y, 
            0.0, 0.0, c->clone(), c->clone(), 
            Constant<double>(1.0), 1);
    // specification of the output:
    ctx->addOutput<double>("q","matlab",q); 
    ctx->addOutput<double>("f","matlab",f); 
    ctx->addOutput<double>("dx","matlab",ee_x - des_x); 
    ctx->addOutput<double>("dy","matlab",ee_y - des_y); 



    OutputGenerator::Ptr outgen =  create_matlab_output(cout,"matlab");

    //cout << ctx << endl;
    
    qpOASESSolver solver(100,0.0, 1E-8);
    solver.prepareInitialization(ctx);
    for (int i=0;i<100;++i) {
        solver.updateStep(0.01);
    }
    solver.prepareExecution(ctx);
    outgen->init(ctx);
    for (int i=0;i<500;++i) {
        update_and_check(solver,0.01);
        outgen->update(ctx);
    }
    outgen->finish(ctx);
    return 0;
}

