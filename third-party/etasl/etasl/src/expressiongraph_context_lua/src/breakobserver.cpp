#include <string>
#include <vector>
#include <algorithm>
#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>
#include <expressiongraph/breakobserver.hpp>

#include "lua.h"

#include "lauxlib.h"
#include "lualib.h"


namespace KDL {


/**
 * An observer that when triggered falls back to a lua console
 */
class BreakObserver: public Observer {
    LuaContext::Ptr luactx;
    Observer::Ptr next;                 ///< the next observer you want to react to monitors.
public:
    typedef boost::shared_ptr< BreakObserver > Ptr;
    /**
     * \param lc lua context 
     * \param _next next observer to check.  
     */
    BreakObserver(LuaContext::Ptr lc, Observer::Ptr next );

    /**
     * The solver will call this when MonitoringScalar is activated.
     * \param [in] mon the monitor that was activated.
     */
    virtual void monitor_activated(const  MonitorScalar& mon);
    virtual ~BreakObserver();
};



BreakObserver::BreakObserver( LuaContext::Ptr lc, Observer::Ptr _next ):
    luactx(lc), next(_next) {}

void BreakObserver::monitor_activated( const MonitorScalar& mon ) {
    //std::cerr << mon.action_name << std::endl;
    if ( (mon.action_name.compare("break")==0)  ) {
        if (!luactx->call_console() ) {
            std::cerr << "Could not call console for LUA" << std::endl;
        }
    } else { 
        if (next) {
            next->monitor_activated(mon);
        }
    }
}

BreakObserver::~BreakObserver( ) {
}



Observer::Ptr create_break_observer(LuaContext::Ptr lc, Observer::Ptr next ) {
    BreakObserver::Ptr r( new BreakObserver(lc, next) );
    return r;
}



} // namespace KDL

