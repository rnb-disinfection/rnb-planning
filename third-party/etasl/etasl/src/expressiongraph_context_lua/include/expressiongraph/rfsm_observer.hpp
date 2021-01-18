#ifndef EXPRESSIONGRAPH_MONITOR_RFSM_HPP
#define EXPRESSIONGRAPH_MONITOR_RFSM_HPP
#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>

namespace KDL {
/**
 * creates an observer that pushes an event on the rFSM state machine.
 *
 * if an event is triggered, it checks for the type name "event", if the
 * type matches, it pushes the name of the monitor to the rFSM event queue.
 *
 * \param _ctx context where the monitors are specified
 * \param lua_fsm  lua stack where the finite state machine is defined (using rFSM)
 * \param next     the next observer that has to be called (daisy chaining)
 *
 * \warning assumes that a variable fsm is available such that 
 *          "rfsm.send_events(fsm, ...)" succeeds.
 */
Observer::Ptr create_rfsm_observer(Context::Ptr _ctx, LuaContext::Ptr lua_fsm, Observer::Ptr next = Observer::Ptr() );



};// end of namespace KDL
#endif
