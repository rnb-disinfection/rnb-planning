#ifndef PY_ETASL_LUACTX 
#define PY_ETASL_LUACTX 
#include <expressiongraph/context.hpp>
#include <luabind/luabind.hpp>
#include <sstream>
#include <exception>

namespace KDL {


class LuaException:public std::exception {
        std::string msg;
    public:
        LuaException(const std::string& _msg):msg(_msg) {}
        virtual const char* what() const throw() {
            return msg.c_str();
        }
        virtual ~LuaException() throw() {}
};


/**
 * class that maintains the state of the LUA interpreter.
 * This class should not be deleted or go out of scope as long as the
 * context and the expression graphs defined in this class are used.
 */
class LuaCtx {
    /**
     * pointer to the context given by init(ctx), just to be sure
     * to keep a reference to ctx, such that it is not destroyed.
     */
    Context::Ptr ctx; 

    std::stringstream error_str;

    void report_error (const char* operation, int status);
public:
    typedef boost::shared_ptr<LuaCtx> Ptr;

    /**
     * the LUA scripting environment.
     */ 
    lua_State *L;


    LuaCtx();

    /**
     * executes the commands in the striing.
     * \return 0 if succesful, non-zero if there is an error. 
     */
    void executeString(const std::string& cmds);
 
    /**
     * executes the commands in the file with the given filename.
     * \return 0 if succesful, non-zero if there is an error. 
     */
    void executeFile(const std::string& filename);

    /** 
     * initialize the LUA environment with a time and ctx variable.
     * \warning typically called before executeFile/executeString
     */    
    void initContext(Context::Ptr ctx);


    std::string describeContext();

    std::string error_message() {
        return error_str.str();
    }

    ~LuaCtx();
};


} // namespace


#endif
