#!/bin/bash
cat > tmp.lua
#`rospack find expressiongraph_context_lua`/bin/script_solver tmp.lua
rosrun expressiongraph_context_lua script_solver tmp.lua
