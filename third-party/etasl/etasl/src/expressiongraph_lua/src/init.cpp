#include <expressiongraph_lua/bind_kdl.hpp>
#include <luabind/luabind.hpp>
#include <luabind/class_info.hpp>



//.def("debug_printtree",&Expression<double>::debug_printtree)
extern "C" int luaopen_libexpressiongraph_lua(lua_State* L)
{
    using namespace luabind;

    open(L);
    register_kdl(L);
    bind_class_info(L);
    return 0;
}
