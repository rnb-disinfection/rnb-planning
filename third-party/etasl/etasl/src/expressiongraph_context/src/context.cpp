#include <string>
#include <vector>
#include <algorithm>
#include <expressiongraph/context.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/format.hpp>

namespace KDL {



std::string Context::group_name() {
    return current_group_stack.back();
}

void Context::push_group(const std::string& name){
    std::string cname = current_group_stack.back()+"."+name;
    current_group_stack.push_back(cname);
    
}

void Context::pop_group() {
    current_group_stack.pop_back();
}

Expression<double>::Ptr Context::get_start_time(const std::string& name) {
    StartTimeMap::iterator it = start_times.find( name );
    VariableType<double>::Ptr tmp;
    if (it==start_times.end()) {
        VariableType<double>::Ptr tmp = Variable<double>(getScalarNdx("time"),1);  // variable dependend on time
        tmp->setValue( time->value() );
        tmp->setJacobian(0, 0.0);
        start_times[name] = tmp;
        return tmp;
    } else {
        return it->second;
    }
}


void Context::update_active() {
    if (!activity_has_changed) return;
    for (size_t i=0;i<cnstr_scalar.size();++i) {
        if (active_groups.find( cnstr_scalar[i].groupname )!= active_groups.end()) {
            cnstr_scalar[i].active = true;
        } else {
            cnstr_scalar[i].active = false;
        }
    }
    for (size_t i=0;i<cnstr_box.size();++i) {
        if (active_groups.find( cnstr_box[i].groupname )!= active_groups.end()) {
            cnstr_box[i].active = true;
        } else {
            cnstr_box[i].active = false;
        }
    }
    for (size_t i=0;i<mon_scalar.size();++i) {
        if (active_groups.find( mon_scalar[i].groupname )!= active_groups.end()) {
            mon_scalar[i].active = true;
        } else {
            mon_scalar[i].active = false;
        }
    }
    activity_has_changed = false; 
}


void Context::activate(const std::string& name) {
    active_groups.insert(name);
    StartTimeMap::iterator it = start_times.find(name);
    if (it!=start_times.end()) {
        it->second->setValue( time->value());
    }
    activity_has_changed = true;
}

std::string Context::active_group_names() {
    std::string result = "";
    for ( std::set<std::string>::iterator it = active_groups.begin(); it!= active_groups.end(); ++it) {
        result = result + *it + " "; 
    }
    return result;
}
void Context::deactivate(const std::string& name) {
    active_groups.erase(name);
    activity_has_changed = true;
}


void Context::deactivate_all(const std::string& name) {
    active_groups.clear();
    activity_has_changed = true;
}

bool Context::activate_cmd(const std::string& cmd) {
    std::set<std::string> ag = active_groups; 
    std::vector<std::string> cmds;
    boost::split( cmds, cmd, boost::is_any_of(",; "), boost::token_compress_on );
    for (unsigned int i=0;i<cmds.size();++i) {
        if (cmds[i].size()!=0) {
            std::string name;
            StartTimeMap::iterator it;
            switch (cmds[i][0]) {
                case '+':
                    name = cmds[i].substr(1);
                    ag.insert(name);
                    it = start_times.find(name);
                    if (it!=start_times.end()) {
                        it->second->setValue( time->value());
                    }
                    break;
                case '-':
                    if (cmds[i][cmds[i].size()-1]=='*') {
                        std::string s = cmds[i].substr(1,cmds[i].size()-2);
                        //std::cerr << "looking up " << s << std::endl;
                        // a little bit tricky, because erase invalidates the current pointer.
                        for ( std::set<std::string>::iterator it = ag.begin(); it!= ag.end(); ) {
                            //std::cerr << "checking " << *it << std::endl;
                            if (it->compare(0,s.size(),s)==0) {
                                //std::cerr << "removing " << *it << std::endl;
                                ag.erase(it++);
                            } else {
                                ++it;
                            }
                        }
                    } else {
                        ag.erase(cmds[i].substr(1));
                    }
                    break;
                default:
                    return false;
            }
        }
    }
    active_groups = ag;
    activity_has_changed = true;
    return true;
}

template <typename T>
void call_setInputValues(
    boost::any& obj, 
    const std::vector<int>& ndx,
    const Eigen::VectorXd& values ) {
        try {
            boost::any_cast< typename Expression<T>::Ptr >( obj ) -> setInputValues(ndx,values);
        } catch(boost::bad_any_cast& ) {
        }
}

template <typename T>
void call_setInputValues(
    boost::any& obj, 
    const std::vector<int>& ndx,
    const std::vector<double>& values) {
        try {
            boost::any_cast< typename Expression<T>::Ptr >( obj ) -> setInputValues(ndx,values);
        } catch(boost::bad_any_cast& ) {
        }
}



void Context::setInputValues_outputvars(
    const std::vector<int>& ndx,
    const Eigen::VectorXd& values
) {
    for (OutputVarMap::iterator it = output_vars.begin(); it!=output_vars.end();++it) {
        it->second->setInputValues(ndx,values);
    }
}


void Context::setInputValues_outputvars(
    const std::vector<int>& ndx,
    const std::vector<double>& values)
{
    for (OutputVarMap::iterator it = output_vars.begin(); it!=output_vars.end();++it) {
        it->second->setInputValues(ndx,values);
    }
}

void Context::addToOptimizer_outputvars(ExpressionOptimizer& opt) {
    update_active();
    for (OutputVarMap::iterator it = output_vars.begin(); it!=output_vars.end();++it) {
        it->second->addToOptimizer(opt); 
    }
}


 
/***********************************************************************************
 * GENERAL
***********************************************************************************/
double Context::getSolverProperty(const std::string& name, double defaultval) {
    StringMap::iterator it = solver_property.find(name);
    if (it !=solver_property.end()) {
        return it->second;
    } else {
        return defaultval;
    } 
}
std::string Context::getSolverStringProperty(const std::string& name, const std::string& defaultvalue) {
    std::map<std::string,std::string>::iterator it = solver_string_property.find(name);
    if (it !=solver_string_property.end()) {
        return it->second;
    } 
    return defaultvalue;
}


void Context::setSolverProperty(const std::string& name, double value) {
    solver_property[name] = value;
}

void Context::setSolverStringProperty(const std::string& name, const std::string& value) {
    solver_string_property[name] = value;
}


Context::Context() {
    next_varnr = 0;
    addType("time");
    time    = addScalarVariable("time","time",0.0,Constant(1.0)); 

    //reltime = addScalarVariable("reltime","time",0.0,Constant(1.0)); 
    current_group_stack.push_back("global");
    activate("global");
    // just for the ones that forget to call this to initialize:
    resetMonitors();
    // just for the ones that forget to call this to initialize:
    clearFinishStatus();
    ctrl_registry = boost::make_shared< ControllerRegistry > (); 
    default_ctrl  = create_controller_proportional();
    default_ctrl->setParameter("K",Constant<double>(4));
    ctrl_registry->register_controller( default_ctrl );
    ctrl_registry->register_controller( create_controller_proportional_saturated() );
}

void Context::setInputValues(const std::vector<int>& ndx,const Eigen::VectorXd& values) {
    update_active();
    setInputValues_constraints(ndx,values);
    setInputValues_monitors(ndx,values);
    setInputValues_outputs(ndx,values);
    setInputValues_outputvars(ndx,values);
    setInputValues_variables(ndx,values);
}

void Context::setInputValues(const std::vector<int>& ndx,const std::vector<double>& values) {
    update_active();
    setInputValues_constraints(ndx,values);
    setInputValues_monitors(ndx,values);
    setInputValues_outputs(ndx,values);
    setInputValues_outputvars(ndx,values);
    setInputValues_variables(ndx,values);
}

void Context::addToOptimizer(ExpressionOptimizer& opt) {
    update_active();
    addToOptimizer_constraints(opt);
    addToOptimizer_monitors(opt);
    addToOptimizer_outputs(opt);
    addToOptimizer_outputvars(opt);
    addToOptimizer_variables(opt);
}

std::string getVarTypeString( boost::any& arg ) {
     if (arg.type() == typeid(typename VariableType<double>::Ptr) ) {                         
        return "double";
     }
     if (arg.type() == typeid(typename VariableType<Vector>::Ptr) ) {                         
        return "Vector";
     }
     if (arg.type() == typeid(typename VariableType<Rotation>::Ptr) ) {                         
        return "Rotation";
     }
     if (arg.type() == typeid(typename VariableType<Frame>::Ptr) ) {                         
        return "Frame";
     }
     if (arg.type() == typeid(typename VariableType<Twist>::Ptr) ) {                         
        return "Twist";
     }
     if (arg.type() == typeid(typename VariableType<Wrench>::Ptr) ) {                         
        return "Wrench";
     }
}

void Context::printInputChannels(std::ostream& os) {
    using namespace std;
    using boost::format;
    using boost::io::group; 
    os << "input channels ("<< input_vars.size()  << "):\n";
    os << format("\t%|57T-|\n");
    os << format("\t%1$=40s|%2$=16s\n") % "Name" % "Type";
    os << format("\t%|57T-|\n");
    for (InputVarMap::iterator it = input_vars.begin(); it!=input_vars.end();++it) {
        os << format("\t%1$=40s|%2$=16s\n") % it->first % getVarTypeString( it->second );
    }
}

std::string getExprTypeString(ExpressionBase::Ptr arg  ) {
    if (boost::dynamic_pointer_cast< Expression<double> >( arg )!=0) {
        return "double";
    }
    if (boost::dynamic_pointer_cast< Expression<Vector> >( arg )!=0) {
        return "Vector";
    }
    if (boost::dynamic_pointer_cast< Expression<Rotation> >( arg )!=0) {
        return "Rotation";
    }
    if (boost::dynamic_pointer_cast< Expression<Frame> >( arg )!=0) {
        return "Frame";
    }
    if (boost::dynamic_pointer_cast< Expression<Twist> >( arg )!=0) {
        return "Twist";
    }
    if (boost::dynamic_pointer_cast< Expression<Wrench> >( arg )!=0) {
        return "Wrench";
    }
    return "unknown";
}

void Context::printOutputExpressions(std::ostream& os) {
    using namespace std;
    using boost::format;
    using boost::io::group; 
    os << "output expressions ("<< output_vars.size()  << "):\n";
    os << format("\t%|57T-|\n");
    os << format("\t%1$=40s|%2$=16s\n") % "Name" % "Type";
    os << format("\t%|57T-|\n");
    for (OutputVarMap::iterator it = output_vars.begin(); it!=output_vars.end();++it) {
        os << format("\t%1$=40s|%2$=16s\n") % it->first % getExprTypeString(it->second);
    }
}



std::ostream& operator << ( std::ostream& os, const Context::Ptr v ) {
    os << "active group names : " << v->active_group_names();
    os << std::endl;
    v->printVariables(os);
    v->printConstraints(os);
    v->printOutputs(os);
    v->printMonitors(os);
    v->printInputChannels(os);
    v->printOutputExpressions(os);
    return os;
}

} // namespace KDL

