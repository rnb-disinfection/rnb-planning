expressiongraph binding
-----------------------

all operations on expressiongraph are defined in lua.


ilua.lua
--------

Some adaptations to the ilua.lua script from Steve Donovan 2007, Chris Hixon, 2010.

you can call the operating system shell using a "!" command.

<TAB> completion and history is available and stored in a "." file in the directory
where ilua is called.

special imports: ilua, Ilua, Pretty, p, ls, dir, slice

there is another environent _G


command line options:
    -g global_env
    -H inject helpers
    -s strict false 
    -v verbose
    -p postpone is true


Package
-------

catkinized package, based upon lua 5.1 (for compatibility with orocos-rtt)
luabind in ubuntu 14.04 only supports lua5.2, therefore a seperately compiled
luabind package is introduced.
Catkinized the package

To be verified, especially wrt "install"

Also uses catkins env-hooks, such that the lua path is automatically
set, and also the path itself.

License
-------

GNU LGPL v3, see LICENSE

Author
------

Erwin Aertbeliën
`
