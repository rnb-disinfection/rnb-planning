#include <expressiongraph_tf/context.hpp>
#include <expressiongraph_tf/context_aux.hpp>
#include <expressiongraph_tf/urdfexpressions3.hpp>
#include <expressiongraph_tf/qpoases_solver.hpp>
#include <expressiongraph_tf/outputs_matlab.hpp>

using namespace KDL;
using namespace Eigen;
using namespace std;




int main(int argc, char* argv[]) {
    Context::Ptr ctx = create_context();

    // set-up necessary for robot control problems:
    ctx->addType("robot");
    ctx->addType("feature");

    // define the problem: a 1-dof robot in 2d-space
    // not the best way, just an illustration
    double K = 4;
    Expression<double>::Ptr q     = ctx->addScalarVariable("q","robot",0.0, Constant(1.0));
    Expression<double>::Ptr f     = ctx->addScalarVariable("f","feature",0.0, Constant(1.0));
    Expression<double>::Ptr L     = Constant(0.2);
    Expression<double>::Ptr ee_x  = (L+f)*cos(q);
    Expression<double>::Ptr ee_y  = (L+f)*sin(q);
    Expression<double>::Ptr des_x = Constant(0.2);
    Expression<double>::Ptr des_y = Constant(0.2);
    addConstraint(ctx,"tracking_x",  ee_x - des_x, K, 1.0, 1); 
    addConstraint(ctx,"tracking_y",  ee_y - des_y, K, 1.0, 1); 

    // specification of the output:
    ctx->addOutput<double>("q","matlab",q); 
    ctx->addOutput<double>("f","matlab",f); 
    ctx->addOutput<double>("dx","matlab",ee_x - des_x); 
    ctx->addOutput<double>("dy","matlab",ee_y - des_y); 

    UrdfExpr3 ue;
    ue.readFromFile("pr2.urdf");
    ue.addTransform("t1","r_gripper_palm_link","base_link");
    ue.addTransform("t2","l_gripper_palm_link","base_link");
    ExpressionMap m = ue.getExpressions(ctx);
    for (ExpressionMap::iterator it = m.begin(); it!=m.end(); ++it) {
        cout << it->first << "\t";
        it->second->print(cout);
        cout << endl;
    }
   return 0;
}

