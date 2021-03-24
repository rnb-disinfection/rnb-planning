#include <expressiongraph_tf/context.hpp>
#include <iostream>

using namespace KDL;
using namespace std;


int main(int argc, char* argv[]) {
    Context::Ptr ctx = create_context(); 
    ctx->addType("feature");
    Expression<double>::Ptr f1=ctx->addScalarVariable("tst","feature",0.1, Constant(1.0));
    if (!f1) {
        cout << "could not add scalar variable" << endl;
        return -1;
    }
    Expression<double>::Ptr r = f1*f1;
    cout << "r = " << r->value() << endl;
    Expression<double>::Ptr tst = ctx->createInputChannel<double>("tst");
    

    VariableType<double>::Ptr var;
    var = ctx->getInputChannel<double>("global.tst");
    if (var) {
        cout << "variable found" << endl;
        var->setValue(3.1415);
        cout << "value " << tst->value() << endl;
    } else {
        cout << "could not find input variable" << endl;
    }
    return 0;
}
