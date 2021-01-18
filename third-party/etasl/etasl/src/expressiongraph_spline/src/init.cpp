#include <luabind/luabind.hpp>
#include <luabind/class_info.hpp>
#include <expressiongraph/spline_scripting.hpp>


// this allows to open the dynamic library directly into a
// lua interpreter, as long it is in the lua search path.
// very handy for testing purposes.

//.def("debug_printtree",&Expression<double>::debug_printtree)
extern "C" int luaopen_libexpressiongraph_spline(lua_State* L)
{
    using namespace luabind;

    open(L);
    //register_kdl(L); expressiongraphs and kdl should already be registered. 
    register_spline(L);
    return 0;
}

