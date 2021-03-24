#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <expressiongraph/defaultobserver.hpp>
#include <boost/make_shared.hpp>

namespace KDL {


/**
 * An observer that provokes an exit of the execution loop. 
 */
class DefaultObserver: public Observer {
    Context::Ptr  ctx;
    Observer::Ptr next;                 ///< the next observer you want to react to monitors.
    std::string   action_name;      ///< only listen to monitors with this action_name.
public:
    typedef boost::shared_ptr< DefaultObserver > Ptr;
    DefaultObserver(Context::Ptr _ctx, 
                    Observer::Ptr _next,
                    const std::string& _action_name ):
        ctx(_ctx), 
        next(_next),
        action_name(_action_name) {}


    virtual void monitor_activated(const  MonitorScalar& mon) {
        using namespace std;
        if (action_name.size() !=0) {
              if (action_name == mon.action_name) {
               ctx->setFinishStatus();
            } else {
                if (next) {
                    next->monitor_activated(mon);
                }
            }
       } else {
            cerr << "Monitor trigger reached DefaultObserver"<<endl;
            cerr << "Monitor name        " << mon.name << endl;
            cerr << "Monitor value       " << mon.expr->value() << endl;
            cerr << "Monitor action_name " << mon.action_name << endl;
            cerr << "Monitor argument    " << mon.argument << endl;
            cerr << "\nSetting finish flag"<<endl;
            ctx->setFinishStatus();
        }
    }

    virtual ~DefaultObserver() {}
};


Observer::Ptr create_default_observer(
        Context::Ptr ctx,
        const std::string& action_name, 
        Observer::Ptr next 
) {
    return boost::make_shared<DefaultObserver>(ctx,next, action_name);
}

Observer::Ptr create_default_observer(
        Context::Ptr ctx,
        Observer::Ptr next 
) {
    return boost::make_shared<DefaultObserver>(ctx, next,"");
}




} // namespace KDL

