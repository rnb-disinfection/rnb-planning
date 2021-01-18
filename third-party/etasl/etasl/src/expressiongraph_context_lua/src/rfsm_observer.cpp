#include <string>
#include <vector>
#include <algorithm>
#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>
#include <expressiongraph/rfsm_observer.hpp>
#include <luabind/luabind.hpp>

namespace KDL {


/**
 * Observer that puts events on an rFSM event queue 
 */
class RFSMObserver: public Observer {
    Context::Ptr    ctx;                  ///< context to operate on
    Observer::Ptr   next;                 ///< the next observer you want to react to monitors.
    LuaContext::Ptr lua_fsm;              ///< refers to the Lua stack for the fsm execution 
                                          ///<( different from Lua stack for constraint specification )
    luabind::object fsm;
    luabind::object se;
public:
    typedef boost::shared_ptr< RFSMObserver > Ptr;

    /**
     * \param _ctx  context to operate on
     * \param _next next observer to check.  
     */
    RFSMObserver(Context::Ptr _ctx, LuaContext::Ptr _lua_fsm,Observer::Ptr next );

    /**
     * The solver will call this when MonitoringScalar is activated.
     * \param [in] mon the monitor that was activated.
     */
    virtual void monitor_activated(const  MonitorScalar& mon);

    virtual ~RFSMObserver();
};



Observer::Ptr create_rfsm_observer(Context::Ptr _ctx, LuaContext::Ptr lua_fsm, Observer::Ptr next  ) {
    RFSMObserver::Ptr r( new RFSMObserver(_ctx, lua_fsm,next) );
    return r;
}


RFSMObserver::RFSMObserver( 
    Context::Ptr _ctx, 
    LuaContext::Ptr _lua_fsm,
    Observer::Ptr _next 
):
    ctx(_ctx), next(_next),lua_fsm(_lua_fsm) 
{
    fsm = luabind::globals(lua_fsm->L)["fsm"];
    se  = luabind::globals(lua_fsm->L)["rfsm"]["send_events"];
}

void RFSMObserver::monitor_activated( const MonitorScalar& mon ) {
    //std::cerr << "monitor activated " << mon.action_name << std::endl;
    if (mon.action_name.compare("event")==0) {
        std::cerr << "triggering event : " << mon.argument << std::endl;
        luabind::call_function<void>(se,fsm, mon.argument);
    } else {
        if (next) {
            next->monitor_activated(mon);
        }
    }
}

RFSMObserver::~RFSMObserver( ) {
}


} // namespace KDL

