#include <expressiongraph_tf/variables.hpp>
/**
 * small test program to show the functionality of Variables.
 */
int main(int argc, char* argv[]) {
    using namespace KDL;
    using namespace std;
    
    Variables v;
    // declare relevant types and variables for the solver at hand:
    v.addType("time");
    v.addType("robot");
    v.addType("feature");
    Expression<double>::Ptr t  = v.addScalarVariable("time","time",0.0, Constant(1.0)); 


    // declare some application variables:
    Expression<double>::Ptr q1 = v.addScalarVariable("q1","robot",0.0, Constant(1.0));
    Expression<double>::Ptr q2 = v.addScalarVariable("q2","robot",0.0, Constant(1.0));
    Expression<double>::Ptr f1 = v.addScalarVariable("f1","feature",1.0, Constant(1.0));
    
    // get a list of the types: 
    set<string> s;  v.getTypes(s);
    for (set<string>::iterator it=s.begin();it!=s.end();it++) {
        cout << *it << "  ";
    } 
    cout << endl;
    
    // get all robot and feature variables:
    vector<int> ndx;
    v.getScalarsOfType("robot",ndx);
    v.getScalarsOfType("feature",ndx);
    
    // do something with it:
    for (size_t i=0; i < ndx.size(); ++i) {
        VariableScalar* p = v.getScalarStruct(ndx[i]);
       cout << p->name << "\t:\t" << p->type << "\t:\t" << p->number << "\t:\t" << p->initial_value << "\t";
       p->weight->print(cout);
       cout  << "\t" ;
       p->expr->print(cout);
       cout << endl;
    }
    Expression<double>::Ptr r = v.getScalarExpr("q1");
    r->print(cout); cout << endl;
    cout << v.getScalarNdx("q1") << endl;
    int n = v.getScalarNdx("q1");
    cout << v.getType(n) << endl;
    cout << v.getName(n) << endl;

    // robust code should check the output results:
    VariableScalar *p = v.getScalarStruct("q12");
    if (p!=0) {
        cout << p->name << endl;
    } else {
        cout << "variable not found" << endl;
    }
}
