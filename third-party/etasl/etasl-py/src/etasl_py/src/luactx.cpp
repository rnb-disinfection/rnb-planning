#include "luactx.hpp"

#include <lua.hpp>
#include <luabind/luabind.hpp>
//#include <luabind/operator.hpp>
#include <luabind/class_info.hpp>
//#include <luabind/out_value_policy.hpp>
#include <expressiongraph_lua/bind_kdl.hpp>
#include <expressiongraph/context_scripting.hpp>
//#include <expressiongraph/urdfexpressions.hpp>
//#include <luabind/shared_ptr_converter.hpp>
//#include <luabind/adopt_policy.hpp>
//#include <ros/package.h>
//#include <boost/tuple/tuple.hpp>
//#include <stdexcept>
//#include <string>


namespace KDL {

LuaCtx::LuaCtx() {
    L = luaL_newstate();
    luaL_openlibs(L);
    luabind::open(L);
    register_kdl(L);
    register_context(L);
    luabind::bind_class_info(L);
}

LuaCtx::~LuaCtx() {
    lua_close(L); 
}

void LuaCtx::report_error (const char* operation, int status) {
  error_str.str("");error_str.clear();
  if (status && !lua_isnil(L, -1)) {
    const char *msg = lua_tostring(L, -1);
    if (msg == NULL) msg = "(error object is not a string)";
    error_str << "Lua Error during " << operation << ":\n" << msg << std::endl;
    lua_pop(L, 1);
    throw LuaException(error_str.str());
  }
}

std::string LuaCtx::describeContext() {
    lua_getfield(L, LUA_GLOBALSINDEX,"ctx");
    lua_getfield(L, -1,"__tostring");
    lua_remove(L,-2);
    lua_getfield(L, LUA_GLOBALSINDEX,"ctx");
    lua_call(L,1,1);  // with 1 argument and 1 result
    return lua_tostring(L,1);
}

void LuaCtx::initContext(Context::Ptr ctx) {
    this->ctx = ctx; // keep reference.
    luabind::globals(L)["time"] = ctx->getScalarExpr("time");
    luabind::globals(L)["ctx"]  = ctx; 
}

void LuaCtx::executeFile(const std::string& name ) {
    report_error("executeFile: loading/compiling", luaL_loadfile(L,name.c_str()) );
    report_error("executeFile: execution", lua_pcall(L,0,0,0));
}

void LuaCtx::executeString(const std::string& str) {
   const char* buf = str.c_str();
   report_error("executeString: loading/compiling", luaL_loadbuffer(L,buf,str.length(),"executeString"));
   report_error("executeString: execution", lua_pcall(L,0,0,0));
}

}

