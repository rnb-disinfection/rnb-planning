#include <expressiongraph/context.hpp>
#include <expressiongraph/luabind_util.hpp>
#include <expressiongraph/context_scripting.hpp>

#include <lua.hpp>
#include <luabind/luabind.hpp>
#include <luabind/operator.hpp>
#include <luabind/class_info.hpp>
#include <luabind/out_value_policy.hpp>
#include <expressiongraph_lua/bind_kdl.hpp>
#include <expressiongraph/urdfexpressions.hpp>
#include <luabind/shared_ptr_converter.hpp>
#include <luabind/adopt_policy.hpp>
#include <ros/package.h>
#include <boost/tuple/tuple.hpp>
#include <stdexcept>
#include <string>

using namespace KDL;
using namespace luabind;
using namespace std;

#include <map>

typedef std::map< std::string,    KDL::Expression<double>::Ptr > ExpressionDoubleMap;

bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

std::string withoutEnding(std::string const& fullString, int n) {
    return fullString.substr(fullString.length() - n,n);
}


// had to add meas and model parameters to controller_param because luabind
// only allows 10 parameters.
// parameters for both lower and upper are specified together,
// if you only want to specify lower append _lower (identical for _upper)
void addInequalityConstraintLua(
                lua_State*                  L,
                Context*                    ctx,
                const std::string&          cname,
                double                      target_lower,
                double                      target_upper,
                const std::string&          controller_lower,
                const std::string&          controller_upper,
                luabind::object             const & controller_param,
                int                         priority,
                Expression<double>::Ptr     weight
)
{
    Expression<double>::Ptr     model;
    Expression<double>::Ptr     meas;

    Controller::Ptr ctrl_lower = ctx->getControllerRegistry()->create(controller_lower);
    if (!ctrl_lower) {
    	throw std::invalid_argument("invalid controller name for controller_lower");
    }
    Controller::Ptr ctrl_upper = ctx->getControllerRegistry()->create(controller_upper);
    if (!ctrl_upper) {
    	throw std::invalid_argument("invalid controller name for controller_upper");
    }
    if (type(controller_param) == LUA_TTABLE) {
        for (luabind::iterator i( controller_param), end; i!=end; ++i) {
            std::string name = object_cast<std::string>(i.key());
            Expression<double>::Ptr arg = object_cast< Expression<double>::Ptr >(controller_param[ i.key()]);
            //cout << "parameter " << name << " = ";arg->print(cout);cout << endl;
            std::string name_lower;
            std::string name_upper;
            if (name=="model") {
                model = arg; 
            } else if (name=="meas") {
                meas = arg; 
            } else if (hasEnding(name, "_lower")) {
                std::string parametername= withoutEnding(name,6);
                //cout << cname << " parameter for lower : " << parametername << " = ";arg->print(cout);cout << endl;
                if (!ctrl_lower->setParameter(parametername, arg)) {
                    throw std::invalid_argument("invalid controller parameter for controller_lower");
                }
            } else if (hasEnding(name, "_upper")) {
                std::string parametername= withoutEnding(name,6);
                //cout << cname << " parameter for upper : " << parametername << " = ";arg->print(cout);cout << endl;
                if (!ctrl_upper->setParameter(parametername, arg)) {
                    throw std::invalid_argument("invalid controller parameter for controller_upper");
                }
            } else {
                //cout << cname << " parameter for upper and upper : " << name << " = ";arg->print(cout);cout << endl;
                if (!ctrl_lower->setParameter(name, arg)) {
                    throw std::invalid_argument("invalid controller parameter for controller_lower");
                }
                if (!ctrl_upper->setParameter(name, arg)) {
                    throw std::invalid_argument("invalid controller parameter for controller_upper");
                }
            }
        }
    } else {
        throw std::invalid_argument("controller_param argument is not a table"); 
    }
   if (!model) {
        throw std::invalid_argument("controller_param should contain 'model' key");
    }
   if (!meas) {
        throw std::invalid_argument("controller_param should contain 'meas' key");
    }
    //cout << "name " << name << endl;
    //cout << "model "; model->print(cout); cout << endl;
    //cout << "meas ";  meas->print(cout); cout << endl;
    //cout << "target_lower " << target_lower << endl;
    //cout << "target_upper " << target_upper << endl;
    //cout << "controller_lower " << controller_lower << endl;
    //cout << "controller_upper " << controller_upper << endl;
    //cout << "weight "; weight->print(cout); cout << endl;
    //cout << "priority " << priority << endl;
 
   ctx->addInequalityConstraint(cname, model, meas, target_lower, target_upper, 
                                 ctrl_lower,  ctrl_upper, weight, priority);
}

struct null_deleter {
   void operator() (void const*) const {}
};
 
std::ostream& operator << ( std::ostream& os, const Context& v ) {
   using namespace boost;
   boost::shared_ptr<Context> vp(const_cast<Context*>(&v),null_deleter());
   return KDL::operator << (os, vp);
}
 
void getExpressions(lua_State* L, UrdfExpr3* ue, Context::Ptr& ctx) {
    lua_newtable(L);
    //lua_pushstring(L, "naam" );
    //Expression<double>::Ptr p = input(1);
    //luabind::detail::push(L, p );
    //lua_settable(L, -3);
    ExpressionMap m = ue->getExpressions(ctx);
    for (ExpressionMap::iterator it = m.begin(); it!= m.end(); ++it) {
            lua_pushstring(L,it->first.c_str());
            luabind::detail::push(L,it->second);   // why is such a useful function in detail ?
            //it->second->print(cout);cout << endl;
            lua_settable(L, -3);
    }
}
void getAllJointNames(lua_State* L, UrdfExpr3* ue) {
    lua_newtable(L);
    //lua_pushstring(L, "naam" );
    //Expression<double>::Ptr p = input(1);
    //luabind::detail::push(L, p );
    //lua_settable(L, -3);
    std::vector<string> names;
    ue->getAllJointNames(names);
    for (size_t i=0;i<names.size();++i) {
            lua_pushnumber(L,i+1);
            luabind::detail::push(L,names[i]);   // why is such a useful function in detail ?
            //it->second->print(cout);cout << endl;
            lua_settable(L, -3);
    }
}

void getAllJointExpressions(lua_State* L, UrdfExpr3* ue) {
    lua_newtable(L);
    std::vector<Expression<double>::Ptr> exprs;
    ue->getAllJointExpressions(exprs);
    for (size_t i=0;i<exprs.size();++i) {
            lua_pushnumber(L,i+1);
            luabind::detail::push(L,exprs[i]);   // why is such a useful function in detail ?
            //it->second->print(cout);cout << endl;
            lua_settable(L, -3);
    }
}


void getAllLinkProperties(lua_State* L, UrdfExpr3* ue) {
    std::vector<link_property> props;
    ue->getAllLinkProperties(props);
    lua_newtable(L);
    for (size_t i=0;i<props.size();++i) {
            lua_pushstring(L,props[i].name.c_str());
            lua_newtable(L);
            lua_pushnumber(L, props[i].mass);
            lua_setfield(L,-2,"mass");
            lua_pushnumber(L, props[i].Ixx);
            lua_setfield(L,-2,"Ixx");
            lua_pushnumber(L, props[i].Ixy);
            lua_setfield(L,-2,"Ixy");
            lua_pushnumber(L, props[i].Ixz);
            lua_setfield(L,-2,"Ixz");
            lua_pushnumber(L, props[i].Iyy);
            lua_setfield(L,-2,"Iyy");
            lua_pushnumber(L, props[i].Iyz);
            lua_setfield(L,-2,"Iyz");
            lua_pushnumber(L, props[i].Izz);
            lua_setfield(L,-2,"Izz");
            luabind::detail::push(L,props[i].origin);   // why is such a useful function in detail ?
            lua_setfield(L,-2,"origin");
            lua_settable(L, -3);
    }
}

/*
void setInitialScalarValues(lua_State* L, Context* ctx) {
    if (!lua_istable(L,-1)) {
        lua_pushstring(L,"expects a named values table as input");
        lua_error(L);
    }
    std::map<std::string,double> results;
    lua_pushnil(L);
    while (lua_next(L, -3) != 0) {
        results[lua_tostring(L, -2)] = lua_tonumber(L, -1);
        std::cerr << lua_tostring(L,-2) << std::endl;
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    ctx->setInitialScalarValues(results);
}
void my_function(object const& table)
{
    if (type(table) == LUA_TTABLE)
    {
        table["time"] = std::clock();
        table["name"] = std::rand() < 500 ? "unusual" : "usual";

        std::cout << object_cast<std::string>(table[5]) << "\n";
    }
}
*/
void setInitialScalarValues(Context* ctx,object const& table) {
    if (type(table) == LUA_TTABLE) {
        std::map<std::string,double> results;
        for (luabind::iterator i( table), end; i!=end; ++i) {
            std::string k = object_cast<std::string>(i.key());
            results[k] = object_cast<double>(table[ i.key() ]);
        }
        ctx->setInitialScalarValues(results);
    } else {
        throw "argument should be a table";
    }
}

template <typename T>
bool is_non_null(T const& p)
{
    return p.get();
}

//…
//

    static void stackDump (lua_State *L) {
      int i;
      int top = lua_gettop(L);
      for (i = 1; i <= top; i++) {  /* repeat for each level */
        int t = lua_type(L, i);
        switch (t) {
    
          case LUA_TSTRING:  /* strings */
            printf("`%s'", lua_tostring(L, i));
            break;
    
          case LUA_TBOOLEAN:  /* booleans */
            printf(lua_toboolean(L, i) ? "true" : "false");
            break;
    
          case LUA_TNUMBER:  /* numbers */
            printf("%g", lua_tonumber(L, i));
            break;
    
          default:  /* other values */
            printf("%s", lua_typename(L, t));
            break;
    
        }
        printf("  ");  /* put a separator */
      }
      printf("\n");  /* end the listing */
    }


class UrdfJointsAndConstraintsGeneratorLua: public UrdfJointsAndConstraintsGenerator {
        luabind::object fn;
        std::string error_msg;
    public:
        UrdfJointsAndConstraintsGeneratorLua(luabind::object const& _fn):fn(_fn) {
        }
        /** 
         * if vel_lower > vel_upper no velocity limits.
         * if pos_lower > pos_upper no position limits.
         */
        virtual boost::tuple< Expression<double>::Ptr, std::string> create(Context::Ptr& ctx, const std::string& name, double vel_lower, double vel_upper, double pos_lower, double pos_upper ,
                                                                           double effort_lower, double effort_upper, double dt) {
            Expression<double>::Ptr jvar;
            std::string full_name;
            boost::tuple< Expression<double>::Ptr,std::string> result;
            lua_State* L;
            try{ 
                //boost::tie(jvar,full_name) = ( 
                //jvar = luabind::call_function< Expression<double>::Ptr >(fn,ctx, name, vel_lower, vel_upper, pos_lower, pos_upper);
                // manually calling callback function because I need multiple outputs:
                lua_State* L = fn.interpreter();
                fn.push( L );
                luabind::detail::push( L,ctx );
                luabind::detail::push( L,name );
                luabind::detail::push( L,vel_lower );
                luabind::detail::push( L,vel_upper );
                luabind::detail::push( L,pos_lower );
                luabind::detail::push( L,pos_upper );
                luabind::detail::push( L,effort_lower );
                luabind::detail::push( L,effort_upper );
                luabind::detail::push( L,dt );
                if (lua_pcall(L, 9, 2, 0)!=0) {
                    error_msg = lua_tostring(L, -1);
                    lua_pop(L,1);
                    throw error_msg.c_str();
                }
                jvar      = object_cast<Expression<double>::Ptr>(object(from_stack(L,-2)));
                full_name = object_cast<std::string>(object(from_stack(L,-1)));
                lua_pop(L, 2);
            } catch(std::exception& e) {
                error_msg = "Error calling feedback function in UrdfExpr: ";
                error_msg += e.what();
                throw error_msg.c_str();
            }
            /*if (!jvar) {
                throw "call back function for UrdfExpr returned wrong type of variable";
            }*/
            return boost::make_tuple(jvar,full_name);
        }
        virtual ~UrdfJointsAndConstraintsGeneratorLua() {}
};

void setCB( UrdfExpr3* urdfexpr,luabind::object const& fn) {
    if (!fn.is_valid()) {
            throw "callback function for UrdfExpr is not valid or is NULL";
    }
    UrdfJointsAndConstraintsGenerator::Ptr u(new UrdfJointsAndConstraintsGeneratorLua(fn));
    urdfexpr->setJointsAndConstraintsCallback(u);
}


std::string getScalarName(Context::Ptr ctx, int ndx) {
    VariableScalar* v = ctx->getScalarStruct(ndx);
    if (v==0) {
        throw "integer index number is not known to this context";
    } else {
        return v->name;
    }
}

std::string getScalarType(Context::Ptr ctx, int ndx) {
    VariableScalar* v = ctx->getScalarStruct(ndx);
    if (v==0) {
        throw "integer index number is not known to this context";
    } else {
        return v->type;
    }
}

double getScalarInitialValue(Context::Ptr ctx, int ndx) {
    VariableScalar* v = ctx->getScalarStruct(ndx);
    if (v==0) {
        throw "integer index number is not known to this context";
    } else {
        return v->initial_value;
    }
}

Expression<double>::Ptr getScalarWeight(Context::Ptr ctx, int ndx) {
    VariableScalar* v = ctx->getScalarStruct(ndx);
    if (v==0) {
        throw "integer index number is not known to this context";
    } else {
        return v->weight;
    }
}


void register_context(lua_State* L) {
    module(L)
    [
        class_<Context,Context::Ptr>("Context")
            .def(constructor<>())
            .def("pushGroup", &Context::push_group)
            .def("popGroup", &Context::pop_group)
            .def("groupName", &Context::group_name)
            .def("get_start_time",  
                    ( 
                        Expression<double>::Ptr (Context::*) (const std::string&) 
                    )& Context::get_start_time
                ) 
            .def("get_start_time",  
                    ( 
                        Expression<double>::Ptr (Context::*) () 
                    )& Context::get_start_time
                ) 
            .def("setFinishStatus",&Context::setFinishStatus)
            .def("getFinishStatus",&Context::getFinishStatus)
            .def("clearFinishStatus",&Context::clearFinishStatus)
            .def("activeGroupNames", &Context::active_group_names)
            .def("activate", &Context::activate)
            .def("activate_cmd", &Context::activate_cmd)
            .def("deactivate", &Context::deactivate)
            .def("deactivate_all", &Context::deactivate_all)
            .def("addScalarVariable",  
                    ( Expression<double>::Ptr (Context::*)
                        (const std::string&, const std::string&, double, Expression<double>::Ptr) 
                    )& Context::addScalarVariable
                ) 
            .def("addScalarVariable",  
                    ( Expression<double>::Ptr (Context::*)
                        (const std::string&, const std::string&, double, double) 
                    )& Context::addScalarVariable
                ) 
            .def("addScalarVariable",  
                    ( Expression<double>::Ptr (Context::*)
                        (const std::string&, const std::string&, double) 
                    )& Context::addScalarVariable
                ) 
            .def("addScalarVariable",  
                    ( Expression<double>::Ptr (Context::*)
                        (const std::string&, const std::string&, double) 
                    )& Context::addScalarVariable
                ) 
            .def("addType",  &Context::addType)

            .def("addInequalityConstraint",&addInequalityConstraintLua)
            /*.def("addInequalityConstraint",
                    (int (Context::*) (const std::string &, Expression<double>::Ptr, 
                              double, double, double,double,
                              Expression<double>::Ptr, int, const std::string&))& Context::addInequalityConstraint )
            .def("addInequalityConstraint",
                    (int (Context::*) (const std::string &, Expression<double>::Ptr, 
                              double, double, double,double,
                              double, int, const std::string&))& Context::addInequalityConstraint )
            .def("addConstraint",
                    (int (Context::*) (const std::string &,Expression<double>::Ptr, 
                              double, Expression<double>::Ptr, int, const std::string&) ) &Context::addConstraint )
            .def("addConstraint",
                    (int (Context::*) (const std::string &,Expression<double>::Ptr, 
                              double, double, int, const std::string&) ) &Context::addConstraint )
            .def("addConstraint",
                    (int (Context::*) (const std::string &,Expression<Rotation>::Ptr, 
                              double, Expression<double>::Ptr, int, const std::string&) ) &Context::addConstraint )
            .def("addConstraint",
                    (int (Context::*) (const std::string &,Expression<Rotation>::Ptr, 
                              double, double, int, const std::string&) ) &Context::addConstraint )*/
            .def("addBoxConstraint",
                    ( int (Context::*) (const std::string &, int, double, double, const std::string&)) &Context::addBoxConstraint)
            .def("addOutput", (void (Context::*) (
                                    const std::string&, 
                                    const std::string&,
                                    Expression<double>::Ptr e ))& Context::addOutput<double>)
            .def("addOutput", (void (Context::*) (
                                    const std::string&, 
                                    const std::string&,
                                    Expression<Vector>::Ptr e ))& Context::addOutput<Vector>)
            .def("addOutput", (void (Context::*) (
                                    const std::string&, 
                                    const std::string&,
                                    Expression<Rotation>::Ptr e ))& Context::addOutput<Rotation>)
            .def("addOutput", (void (Context::*) (
                                    const std::string&, 
                                    const std::string&,
                                    Expression<Frame>::Ptr e ))& Context::addOutput<Frame>)
            .def("addOutput", (void (Context::*) (
                                    const std::string&, 
                                    const std::string&,
                                    Expression<Twist>::Ptr e ))& Context::addOutput<Twist>)
            .def("addOutput", (void (Context::*) (
                                    const std::string&, 
                                    const std::string&,
                                    Expression<Wrench>::Ptr e ))& Context::addOutput<Wrench>)
            .def("addMonitor",(void (Context::*) (
                                    const std::string&, Expression<double>::Ptr,
                                     double, double, const std::string&, const std::string&) ) &Context::addMonitor)
            .def("getScalarExpr",&Context::getScalarExpr)
            .def("getScalarNdx",&Context::getScalarNdx)
            .def("setInitialScalarValues",&setInitialScalarValues)
            .def("getSolverProperty",&Context::getSolverProperty)
            .def("getSolverStringProperty",&Context::getSolverStringProperty)
            .def("setSolverProperty",&Context::setSolverProperty)
            .def("setSolverStringProperty",&Context::setSolverStringProperty)

            /*.def("createInputChannelScalar",&Context::createInputChannel<double>)
            .def("createInputChannelVector",&Context::createInputChannel<Vector>)
            .def("createInputChannelRotation",&Context::createInputChannel<Rotation>)
            .def("createInputChannelFrame",&Context::createInputChannel<Frame>)
            .def("createInputChannelTwist",&Context::createInputChannel<Twist>)
            .def("createInputChannelWrench",&Context::createInputChannel<Wrench>)*/

            .def("createInputChannelScalar",  (Expression<double>::Ptr   (Context::*)(const std::string&)) &Context::createInputChannel<double>)
            .def("createInputChannelVector",  (Expression<Vector>::Ptr   (Context::*)(const std::string&)) &Context::createInputChannel<Vector>)
            .def("createInputChannelRotation",(Expression<Rotation>::Ptr (Context::*)(const std::string&)) &Context::createInputChannel<Rotation>)
            .def("createInputChannelFrame",   (Expression<Frame>::Ptr    (Context::*)(const std::string&)) &Context::createInputChannel<Frame>)
            .def("createInputChannelTwist",   (Expression<Twist>::Ptr    (Context::*)(const std::string&)) &Context::createInputChannel<Twist>)
            .def("createInputChannelWrench",  (Expression<Wrench>::Ptr   (Context::*)(const std::string&)) &Context::createInputChannel<Wrench>)



            .def("createInputChannelScalar",  (Expression<double>::Ptr   (Context::*)(const std::string&, const double&))   &Context::createInputChannel<double>)
            .def("createInputChannelVector",  (Expression<Vector>::Ptr   (Context::*)(const std::string&, const Vector&))   &Context::createInputChannel<Vector>)
            .def("createInputChannelRotation",(Expression<Rotation>::Ptr (Context::*)(const std::string&, const Rotation&)) &Context::createInputChannel<Rotation>)
            .def("createInputChannelFrame",   (Expression<Frame>::Ptr    (Context::*)(const std::string&, const Frame&))    &Context::createInputChannel<Frame>)
            .def("createInputChannelTwist",   (Expression<Twist>::Ptr    (Context::*)(const std::string&, const Twist&))    &Context::createInputChannel<Twist>)
            .def("createInputChannelWrench",  (Expression<Wrench>::Ptr   (Context::*)(const std::string&, const Wrench&))   &Context::createInputChannel<Wrench>)

            .def("setOutputExpression",&Context::setOutputExpression<double>)
            .def("setOutputExpression",&Context::setOutputExpression<Vector>)
            .def("setOutputExpression",&Context::setOutputExpression<Rotation>)
            .def("setOutputExpression",&Context::setOutputExpression<Frame>)
            .def("setOutputExpression",&Context::setOutputExpression<Twist>)
            .def("getOutputExpressionScalar",&Context::getOutputExpression<double>)
            .def("getOutputExpressionVector",&Context::getOutputExpression<Vector>)
            .def("getOutputExpressionRotation",&Context::getOutputExpression<Rotation>)
            .def("getOutputExpressionFrame",&Context::getOutputExpression<Frame>)
            .def("getOutputExpressionTwist",&Context::getOutputExpression<Twist>)
            .def("getInputChannelScalar",&Context::getInputChannel<double>)
            .def("getInputChannelVector",&Context::getInputChannel<Vector>)
            .def("getInputChannelRotation",&Context::getInputChannel<Rotation>)
            .def("getInputChannelFrame",&Context::getInputChannel<Frame>)
            .def("getInputChannelTwist",&Context::getInputChannel<Twist>)
            .def("getScalarName",&getScalarName)
            .def("getScalarType",&getScalarType)
            .def("getScalarInitialValue",&getScalarInitialValue)
            .def("getScalarWeight",&getScalarWeight)

            .def(tostring(self)),
        class_<UrdfExpr3,UrdfExpr3::Ptr>("UrdfExpr")
            .def(constructor<double>())
            .def(constructor<double,double,double>())
            //.def_readwrite("K_limits", &UrdfExpr3::K_limits)
            //.def_readwrite("velocity_scale", &UrdfExpr3::velocity_scale)
            .def_readwrite("poslimits",&UrdfExpr3::poslimits)
            .def_readwrite("vellimits",&UrdfExpr3::vellimits)
            .def_readwrite("effortlimits",&UrdfExpr3::effortlimits)
            .def("readFromFile",   &UrdfExpr3::readFromFile)
            .def("readFromParam",  &UrdfExpr3::readFromParam)
            .def("readFromString", &UrdfExpr3::readFromString)
            .def("addTransform",   &UrdfExpr3::addTransform)
            .def("getExpressions", &getExpressions)
            .def("getAllJointNames", &getAllJointNames)
            .def("getAllJointExpressions", &getAllJointExpressions)
            .def("getAllLinkProperties", &getAllLinkProperties)
            .def("setJointsAndConstraintsCallback",&setCB)
            .def(tostring(self)),
        def("make_identifier",&make_identifier),
        def("is_non_null", &is_non_null<Expression<double>::Ptr>),
        def("is_non_null", &is_non_null<Expression<Vector>::Ptr>),
        def("is_non_null", &is_non_null<Expression<Rotation>::Ptr>),
        def("is_non_null", &is_non_null<Expression<Frame>::Ptr>),
        def("is_non_null", &is_non_null<Expression<Twist>::Ptr>),
        def("rospack_find", &ros::package::getPath)
    ];
}


namespace KDL {

LuaContext::LuaContext() {
    L = luaL_newstate();
    luaL_openlibs(L);
    open(L);
    register_kdl(L);
    register_context(L);
    bind_class_info(L);
}

LuaContext::~LuaContext() {
    lua_close(L); 
}

static void l_message (const char *pname, const char *msg) {
  if (pname) fprintf(stderr, "%s: ", pname);
  fprintf(stderr, "%s\n", msg);
  fflush(stderr);
}


static int report (lua_State *L, int status) {
  if (status && !lua_isnil(L, -1)) {
    const char *msg = lua_tostring(L, -1);
    if (msg == NULL) msg = "(error object is not a string)";
    l_message("LuaContext::", msg);
    lua_pop(L, 1);
  }
  return status;
}


static int dolibrary (lua_State *L, const char *name) {
  lua_getglobal(L, "require");
  lua_pushstring(L, name);
  return report(L, lua_pcall(L, 1, 0, 0));
}




bool LuaContext::call_console() {
    int error = dolibrary(L,"ilua");
    if (error !=0) {
        cerr << "could not find the ilua library" << endl;
        return false;
    }
    return true;
}



void LuaContext::initContext(Context::Ptr ctx) {
    this->ctx = ctx; // keep reference.
    luabind::globals(L)["time"] = ctx->getScalarExpr("time");
    luabind::globals(L)["ctx"]  = ctx; 
}

int LuaContext::executeFile(const std::string& name) {
   int error = luaL_loadfile(L,name.c_str());
    if (error != 0) {
        cerr <<  "executeFile : error during loading/compiling :\n " << lua_tostring(L, -1) << endl << endl;
        lua_pop(L, 1);  /* pop error message from the stack */
        return 1;
    }
    error = lua_pcall(L,0,0,0);
    if (error != 0) {
        cerr <<  "executeFile : error during execution : \n" << lua_tostring(L, -1) << endl << endl;
        lua_pop(L, 1);  /* pop error message from the stack */
        return 2;
    }
    return 0;
}

int LuaContext::executeString(const std::string& str) {
   const char* buf = str.c_str();
   int error = luaL_loadbuffer(L,buf,str.length(),"executeString");
    if (error != 0) {
        cerr <<  "executeString : error during loading/compiling :\n " << lua_tostring(L, -1) << endl << endl;
        lua_pop(L, 1);  /* pop error message from the stack */
        return 1;
    }
    error = lua_pcall(L,0,0,0);
    if (error != 0) {
        cerr <<  "executeString : error during execution : \n" << lua_tostring(L, -1) << endl << endl;
        lua_pop(L, 1);  /* pop error message from the stack */
        return 2;
    }
    return 0;
}

} // namespace


