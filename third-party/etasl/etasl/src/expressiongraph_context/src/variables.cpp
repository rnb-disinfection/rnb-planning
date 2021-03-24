#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>

namespace KDL {


Expression<double>::Ptr Context::addScalarVariable(
                        const std::string& name,
                        const std::string& type,
                        double initial_value,
                        Expression<double>::Ptr weight){
    if (var_types.find(type) == var_types.end()) {
        //return Expression<double>::Ptr();
        throw "invalid variable type";
    }
    if (var_names.find(name) != var_names.end()) {
        //return getScalarExpr(name);
        throw "duplicate variable name";
    }
    VariableScalar v;
    v.name          = name;
    v.type          = type;
    v.number        = next_varnr++;
    v.expr          = input(v.number);
    v.initial_value = initial_value;
    v.expr->setInputValue(v.number,initial_value);
    v.weight        = weight;
    var_scalar.push_back(v);
    var_names.insert(name);
    return v.expr;
}

Expression<double>::Ptr Context::addDerivedVariable(
        const std::string& name,
        const std::string& type,
        double initial_value,
        int derivation_number,
        double dt,
        Expression<double>::Ptr weight){
    if (var_types.find(type) == var_types.end()) {
        //return Expression<double>::Ptr();
        throw "invalid variable type";
    }
    if (var_names.find(name) != var_names.end()) {
        //return getScalarExpr(name);
        throw "duplicate variable name";
    }
    VariableScalar v;
    v.name          = name;
    v.type          = type;
    v.number        = next_varnr++;
    v.expr          = derivation(v.number,derivation_number, dt);
    v.initial_value = initial_value;
    v.expr->setInputValue(v.number,initial_value);
    v.weight        = weight;
    var_scalar.push_back(v);
    var_names.insert(name);
    return v.expr;
}

void Context::addType(const std::string& type){
    var_types.insert(type);
}

void Context::getTypes(std::set<std::string>& types){
    types = this->var_types;
} 

void Context::getScalarsOfType(const std::string& type, std::vector<int>& ndx){
    for (size_t i = 0; i < var_scalar.size();++i)  {
        if (var_scalar[i].type == type) {
            ndx.push_back(i);
        }
    }
}


std::string Context::getName(int ndx){
    for (size_t i = 0; i < var_scalar.size();++i)  {
        if (var_scalar[i].number==ndx) {       
            return var_scalar[i].name;
        }
    }
    return "";
}

std::string Context::getType(int ndx){
    for (size_t i = 0; i < var_scalar.size();++i)  {
        if (var_scalar[i].number==ndx) {       
            return var_scalar[i].type;
        }
    }
    return "";
}

int Context::getScalarNdx(const std::string& name){
    VariableScalar* p = getScalarStruct(name);
    if (p==0) {
        return -1;
    } else {
        return p->number;
    }
}

//int Context::getRotationNdx( const std::string& name){}

Expression<double>::Ptr   Context::getScalarExpr(const std::string& name){
    VariableScalar* p = getScalarStruct(name);
    if (p==0) {
        return Expression<double>::Ptr();
    } else {
        return p->expr;
    }

}

//Expression<Rotation>::Ptr   Context::getRotationExpr(const std::string& name){}

VariableScalar* Context::getScalarStruct(const std::string& name) {
     for (size_t i = 0; i < var_scalar.size();++i)  {
        if (var_scalar[i].name==name) {       
            return &var_scalar[i];
        }
    }
    return 0;
}
 
VariableScalar* Context::getScalarStruct(int ndx) {
     for (size_t i = 0; i < var_scalar.size();++i)  {
        if (var_scalar[i].number==ndx) {       
            return &var_scalar[i];
        }
    }
    return 0;
}
void Context::printVariables( std::ostream& os ) {
    using namespace std;
    using boost::format;
    using boost::io::group; 
    os << "Variables("<< var_scalar.size()  << "):\n";
    os << format("\t%|124T-|\n");
    os << format("\t%1$=30s|%2$=16s|%3$=16s|%4$=20s|%5$=20s|%6$=20s\n") % "Name" % "Type" % "Number" % "Init.Value" % "Current Value" % "Weight";
    os << format("\t%|124T-|\n");
    for (size_t i =0; i < var_scalar.size();++i) {
        os << format("\t%1$=30s|%2$=16s|%3$=16d|%4$=20d|%5$=20d|%6$=20d\n") 
             % var_scalar[i].name 
             % var_scalar[i].type
             % var_scalar[i].number
             % var_scalar[i].initial_value
             % var_scalar[i].expr->value()
             % var_scalar[i].weight->value();
    }
    os << format("\t%|124T-|\n");
}

void Context::setInitialScalarValues(const std::map<std::string,double>& iv) {
    typedef std::map<std::string,double> NamedValues;
    for (NamedValues::const_iterator it=iv.begin();it!=iv.end();++it) {
        VariableScalar* s = getScalarStruct(it->first);
        if (s!=0) {
            s->initial_value = it->second; 
            s->expr->setInputValue(s->number,s->initial_value);
        }
    }
}

void Context::setInputValues_variables(const std::vector<int>& ndx,const Eigen::VectorXd& values) {
    for (size_t i=0;i<var_scalar.size();++i) {
        var_scalar[i].weight->setInputValues(ndx,values);
        var_scalar[i].expr->setInputValues(ndx,values);
    }
}

void 
Context::setInputValues_variables(
    const std::vector<int>& ndx,
    const std::vector<double>& values
) {
    for (size_t i=0;i<var_scalar.size();++i) {
        var_scalar[i].weight->setInputValues(ndx,values);
        var_scalar[i].expr->setInputValues(ndx,values);
    }
}

void 
Context::addToOptimizer_variables(ExpressionOptimizer& opt) {
    for (size_t i=0;i<var_scalar.size();++i) {
        var_scalar[i].weight->addToOptimizer(opt);
        var_scalar[i].expr->addToOptimizer(opt);
    }
}
 


} // namespace KDL
