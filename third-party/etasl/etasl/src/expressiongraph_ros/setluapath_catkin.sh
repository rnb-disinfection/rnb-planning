PACKAGE="expressiongraph_lua"

RESETC="\e[0m\e[39m"
BOLD="\e[1m\e[31m"

# $1 pathvar as string $2 addition using seperation char $3
prepend_to_var() {
    if [[ ":${!1}:" != *":$2:"* ]]; then
        export $1="$2${!1:+$3${!1}}"
    fi
}

prepend_existing_to_var() {
    if [ -d "$2" ] && [[ ":${!1}:" != *":$2:"* ]]; then
        export $1="$2${!1:+$3${!1}}"
    fi
}

echo -e "${BOLD}Adding paths to PATH, LUA_PATH, LUA_CPATH and URDF_PATH according to dependencies of $PACKAGE${RESETC}\n"

# default lua paths and lua cpaths:
export LUA_PATH=""
export LUA_CPATH=""


prepend_to_var "PATH" `rospack find expressiongraph_lua`"/scripts" ":"
prepend_to_var "LUA_PATH" `rospack find expressiongraph_lua`"/scripts/?.lua" ":"
echo "LUA_PATH="$LUA_PATH
#prepend_to_var "URDF_PATH" `rospack find expressiongraph_lua`"/robots" ":"
#prepend_to_var "URDF_PATH" `rospack find expressiongraph_lua`"/urdf" ":"

LIBDIRS=`catkin_find --lib`

for libd in $LIBDIRS; do
    echo "adding library directory $libd"
    prepend_to_var LUA_CPATH "$libd/?.so" ";"
done

for pkg in `rospack depends $PACKAGE`; do
    echo "adding scripts directory of $pkg"
    fullpkg=`rospack find $pkg`
    if [ -e "${fullpkg}/scripts/lib" ] 
    then
        echo "added ${fullpkg}/scripts/lib" 
        prepend_to_var LUA_PATH "${fullpkg}/scripts/lib/?.lua" ";"
    fi
    #if [ -e "${fullpkg}/robots" ] 
    #then
    #    echo "added ${fullpkg}/robots" 
    #    prepend_to_var URDF_PATH "${fullpkg}/robots" ":"
    #fi
    #if [ -e "${fullpkg}/urdf" ] 
    #then
    #    echo "added ${fullpkg}/urdf" 
    #    prepend_to_var URDF_PATH "${fullpkg}/urdf" ":"
    #fi
done

prepend_to_var LUA_CPATH "./?.lua" ";"
prepend_to_var LUA_CPATH ";;" ";" prepend_to_var LUA_PATH "./?.so" ";"
prepend_to_var LUA_PATH ";;" ";"

echo -e "\n${BOLD}PATH${RESETC}:\n$PATH"
echo -e "\n${BOLD}LUA_PATH${RESETC}:\n$LUA_PATH"
echo -e "\n${BOLD}LUA_CPATH${RESETC}:\n$LUA_CPATH"
