#include <string>
#include <vector>
#include <algorithm>
#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>
#include <expressiongraph/groupingobserver.hpp>
namespace KDL {

class GroupingObserver: public Observer {
    Context::Ptr  ctx;                  ///< context to operate on
    Observer::Ptr next;                 ///< the next observer you want to react to monitors.
    std::vector<std::string> cmd_queue; ///< queue of commands 
public:
    typedef boost::shared_ptr< GroupingObserver > Ptr;
    /**
     * \param _ctx  context to operate on
     * \param _next next observer to check.  
     */
    GroupingObserver(Context::Ptr _ctx, Observer::Ptr next );

  
    /**
     * The solver will call this when MonitoringScalar is activated.
     * \param [in] mon the monitor that was activated.
     */
    virtual void monitor_activated(const  MonitorScalar& mon);

    virtual ~GroupingObserver();
};




GroupingObserver::GroupingObserver( const Context::Ptr _ctx, Observer::Ptr _next ):
    ctx(_ctx), next(_next) {}

void GroupingObserver::monitor_activated( const MonitorScalar& mon ) {
    //std::cerr << "monitor activated " << mon.action_name << std::endl;
    if (mon.action_name.compare("activate")==0) {
        //std::cerr << mon.action_name << std::endl;
        ctx->activate_cmd( mon.argument );
    } else {
        if (next) {
            next->monitor_activated(mon);
        }
    } 
    if (ctx->active_groups.size()==0) {
        ctx->setFinishStatus(); 
    }
}

GroupingObserver::~GroupingObserver( ) {
}

Observer::Ptr create_grouping_observer(const Context::Ptr _ctx, Observer::Ptr next ) {
    GroupingObserver::Ptr r( new GroupingObserver(_ctx, next) );
    return r;
}



} // namespace KDL

