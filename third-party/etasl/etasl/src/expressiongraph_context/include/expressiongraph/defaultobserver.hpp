#ifndef EXPRESSIONGRAPH_DEFAULTOBSERVER_HPP
#define EXPRESSIONGRAPH_DEFAULTOBSERVER_HPP
#include <expressiongraph/context.hpp>

namespace KDL {

/**
 * creates a default observer with an empty action_name.
 *
 * This observer will react to any trigger of a monitor that reaches it,
 * display information on the monitor and set the finish flag of the context.
 */
Observer::Ptr create_default_observer(Context::Ptr, Observer::Ptr next = Observer::Ptr() );


/**
 * creates a default observer with an action_name.
 *
 * This observer will react to a trigger of a monitor with the geven action_name and
 * set the finish flag of the context. 
 */
Observer::Ptr create_default_observer(Context::Ptr, const std::string& action_name,Observer::Ptr next = Observer::Ptr() );

};// end of namespace KDL
#endif
