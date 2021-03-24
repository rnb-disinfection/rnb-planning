#include <expressiongraph_tf/outputs.hpp>

using namespace KDL;
using namespace Eigen;
using namespace std;


/**
 * example that generates a matlab output using an Outputs specification.
 */
int main(int argc,char* argv[]) {

    // definition: 
    Outputs op;
    op.addOutput<double>("A", "matlab", sin(input(1)) );           // later on we will choose to only output outputs with type "matlab"
    op.addOutput<double>("B", "matlab", input(1) ); 
    op.addOutput<double>("B", "other", input(1)*Constant(10.0) );   // this has the type "other" and will not be used in the output.

    // declaration of a state variable q:
    std::vector<int> ndx;
    ndx.push_back(1);
    Eigen::VectorXd q(1);

/*    
    // execution:
    op.outputMatlabPrefix("matlab",cout);
    for (double t=0.0;t<10.0;t+=0.01) {
        q(0) = t;
        op.setInputValues(ndx,q);
        op.outputToStream("matlab", cout, "\t","\n");
    }    
    op.outputMatlabPostfix(cout);
*/
    return 0;
}
