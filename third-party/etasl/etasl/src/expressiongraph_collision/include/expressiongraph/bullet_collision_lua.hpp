#ifndef EXPRESSIONGRAPH_COLLISION_COLLISION_LUA_HPP
#define EXPRESSIONGRAPH_COLLISION_COLLISION_LUA_HPP
/**
 * lua-binding for collision expressiongraph node and geometric shapes
 *
 * (c) 2015, Erwin Aertbelien, Dep. Of Mech. Eng., KULeuven.
 */

#include <luabind/luabind.hpp>

/**
 * registers a binding to KDL classes and functions that implement collision checking (
 * using the FCL library) 
 * \param [in] L lua_State
 */
extern void register_bullet_collision(lua_State* L);

#endif
