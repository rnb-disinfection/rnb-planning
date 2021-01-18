#ifndef EXPRESSIONGRAPH_CONTEXT_SCRIPTING_HPP
#define EXPRESSIONGRAPH_CONTEXT_SCRIPTING_HPP
#include <expressiongraph/context.hpp>
#include <luabind/luabind.hpp>

namespace KDL {

/**
 * class that maintains the state of the LUA interpreter.
 * This class should not be deleted or go out of scope as long as the
 * context and the expression graphs defined in this class are used.
 */
class LuaContext {
    /**
     * pointer to the context given by init(ctx), just to be sure
     * to keep a reference to ctx, such that it is not destroyed.
     */
    Context::Ptr ctx; 
public:
    typedef boost::shared_ptr<LuaContext> Ptr;

    /**
     * the LUA scripting environment.
     */ 
    lua_State *L;


    LuaContext();

    /**
     * executes the commands in the striing.
     * \return 0 if succesful, non-zero if there is an error. 
     */
    int executeString(const std::string& cmds);
 
    /**
     * executes the commands in the file with the given filename.
     * \return 0 if succesful, non-zero if there is an error. 
     */
    int executeFile(const std::string& filename);

    /** 
     * initialize the LUA environment with a time and ctx variable.
     * \warning typically called before executeFile/executeString
     */    
    void initContext(Context::Ptr ctx);

    /**
     * calls a LUA console for interactive sessions in your program.
     * returns false if error occurred
     * returns true if no error occurred.
     */
    bool call_console();

    /**
     * define a VariableType expression that is accessible in the script and 
     * from the C++ side.
     * \param [in] name specifies the name used in LUA
     * \param [in] depend specifies what dependencies the variable has. (See VariableType)
     */
    template<typename R>
    typename VariableType<R>::Ptr define_var(const std::string& name, const std::vector<int>& depend) {
        typename VariableType<R>::Ptr var = Variable<R>(depend);
        typename Expression<R>::Ptr tmp = var;
        luabind::globals(L)[name] = tmp;
        return var;
    }

    /**
     * define a VariableType expression that is accessible in the script and 
     * from the C++ side.
     * \param [in] name specifies the name used in LUA
     * \param [in] depend specifies what dependencies the variable has. (See VariableType)
     */
    template<typename R>
    typename VariableType<R>::Ptr define_var(const std::string& name, int startndx, int nrofderiv) {
        typename VariableType<R>::Ptr var = Variable<R>(startndx,nrofderiv);
        typename Expression<R>::Ptr tmp = var;
        luabind::globals(L)[name] = tmp;
        return var;
    }


    ~LuaContext();
};


} // namespace

void register_context(lua_State* L);

#endif
