#include <expressiongraph_lua/bind_kdl.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <luabind/luabind.hpp>
#include <luabind/class_info.hpp>


// this allows to open the dynamic library directly into a
// lua interpreter, as long it is in the lua search path.
// very handy for testing purposes.

// the name of this subroutine depends on the name of the shared library that will
// be created.

//.def("debug_printtree",&Expression<double>::debug_printtree)
extern "C" int luaopen_libexpressiongraph_context_lua(lua_State* L)
{
    using namespace luabind;

    open(L);
    register_kdl(L);
    register_context(L);
    bind_class_info(L);
    return 0;
}
  
