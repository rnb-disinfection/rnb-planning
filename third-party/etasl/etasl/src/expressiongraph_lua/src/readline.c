// build@ cc -shared readline.c -o readline.so
// Completely and absolutely minimal binding to the readline library
// Steve Donovan, 2007
#include <stdlib.h> 
#include <string.h> 
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include <readline/readline.h>
#include <readline/history.h>

#if LUA_VERSION_NUM!=501
    #define lua_strlen lua_rawlen
#endif

static int f_readline(lua_State* L)
{
    const char* prompt = lua_tostring(L,1);
    const char* line = readline(prompt);
    lua_pushstring(L,line);
    (void)free((void *)line); // Lua makes a copy...
    return 1;
}

static int f_add_history(lua_State* L)
{
    //if (lua_strlen(L,1) > 0)
    if (lua_strlen(L,1) > 0)
        add_history(lua_tostring(L, 1));
    return 0;
}

static const struct luaL_Reg lib[3] = {
	{"readline", f_readline},
    {"add_history",f_add_history},
	{NULL, NULL},
};

int luaopen_libreadline_c (lua_State *L) {
    //#if LUA_VERSION_NUM==501
	    luaL_openlib (L, "readline", lib, 0);
    //#else
    //    lua_newtable(L);
    //    luaL_setfuncs(L, lib, 0);
    //    lua_setglobal(L,"readline");
    //#endif
	return 1;
}
