
# adapt this script to your directory configuration :
LUA=`rospack find expressiongraph_lua`
TF=`rospack find expressiongraph_tf`

# export variables :
export LUA_CPATH="./?.so;/usr/local/lib/lua/5.1/?.so;/usr/lib/x86_64-linux-gnu/lua/5.1/?.so;/usr/lib/lua/5.1/?.so;/usr/local/lib/lua/5.1/loadall.so;$TF/lib/?.so;$LUA/lib/?.so"

export LUA_PATH="./?.lua;/usr/local/share/lua/5.1/?.lua;/usr/local/share/lua/5.1/?/init.lua;/usr/local/lib/lua/5.1/?.lua;/usr/local/lib/lua/5.1/?/init.lua;/usr/share/lua/5.1/?.lua;/usr/share/lua/5.1/?/init.lua;$TF/scripts/?.lua;%TF/scripts/?;$TF/scripts/lib/?.lua;$TF/scripts/lib/?;$LUA/scripts/?.lua;$LUA/scripts/?"

# add romeo_description package to urdf path if it exists: 
ROMEO=':'`rospack find romeo_description`'/urdf' || ROMEO=''

LWR=':'`rospack find kuka_lwr_description`'/urdf' || LWR=''

export URDF_PATH="$TF/robots$ROMEO$LWR"
export PATH="$LUA/scripts":${PATH}

#
# add path to rFSM (repository: https://github.com/kmarkus/rFSM.git)
# change this to your local paths:
#
export LUA_PATH=${LUA_PATH}";"`rospack find rFSM`"/?.lua"
export PATH="${PATH}:"`rospack find rFSM`/tools


