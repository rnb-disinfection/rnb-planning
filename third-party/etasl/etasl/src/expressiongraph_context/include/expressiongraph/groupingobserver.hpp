#ifndef EXPRESSIONGRAPH_MONITOR_GROUPING_HPP
#define EXPRESSIONGRAPH_MONITOR_GROUPING_HPP
#include <expressiongraph/context.hpp>

namespace KDL {

/**
 * Observer that can change the active groups
 * The action name follows a "mini-language"
 *
 *  "active: -a.b +a.c"
 *
 *  "active:" is a fixed prefix, "-a.b" removes group a.b,
 *            "+a.c" adds group a.c to the active list.
 * 
 * This serves two purposes: simple sequences can be handled with this.
 * Secondly it allows constraints to automatically remove themself from the list of active groups
 * when they are finished
 */
Observer::Ptr create_grouping_observer(const Context::Ptr _ctx, Observer::Ptr next = Observer::Ptr() );


};// end of namespace KDL
#endif
