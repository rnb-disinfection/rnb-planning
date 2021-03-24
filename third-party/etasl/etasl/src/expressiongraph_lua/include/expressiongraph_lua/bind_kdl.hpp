#include <luabind/luabind.hpp>

/**
 * registers a binding to KDL classes and functions under a given namespace.
 * \param [in] L lua_State
 * \param nspace namespace to register the binding under.
 * \return an error code is returned.  If successfull, the error code is 0.  If the
 *         pre-amble could not be loaded, the error code is 1.  If an error code occurred
 *         during the execution of the pre-amble, the error code is 2.
 */
extern void register_kdl(lua_State* L);

