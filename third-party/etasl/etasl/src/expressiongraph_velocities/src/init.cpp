#include <lua.hpp>
#include <luabind/luabind.hpp>
#include <luabind/shared_ptr_converter.hpp>
#include <luabind/operator.hpp>
#include <expressiongraph/expressiongraph_velocities.hpp>
#include <luabind/adopt_policy.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <luabind/class_info.hpp>
#include <expressiongraph/expressiongraph_velocities.hpp>

using namespace KDL;
using namespace luabind;
using namespace std;


void register_expressiongraph_velocities(lua_State* L) {
    module(L)
    [
        def("previous_velocity",&previous_velocity<double>),
        def("previous_velocity",&previous_velocity<Vector>),
        def("previous_velocity",&previous_velocity<Rotation>),
        def("previous_velocity",&previous_velocity<Frame>),
        def("previous_velocity",&previous_velocity<Twist>),
        def("previous_velocity",&previous_velocity<Wrench>)
    ];
}





//.def("debug_printtree",&Expression<double>::debug_printtree)
extern "C" int luaopen_libexpressiongraph_velocities(lua_State* L)
{
    using namespace luabind;

    open(L);
    register_expressiongraph_velocities(L);
    return 0;
}


