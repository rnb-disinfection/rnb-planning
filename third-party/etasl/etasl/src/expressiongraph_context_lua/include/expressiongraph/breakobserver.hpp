#ifndef EXPRESSIONGRAPH_BREAKOBSERVER_HPP
#define EXPRESSIONGRAPH_BREAKOBSERVER_HPP
#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>

namespace KDL {

Observer::Ptr create_break_observer(LuaContext::Ptr lc, Observer::Ptr next = Observer::Ptr() );

};// end of namespace KDL
#endif
