-- type checking utilities for use in rttlua.
-- these routine recognize rtt components and types.

require("string")


all_packages          = {}
all_packages.help     = {}
all_packages.contents = {}

function _contents(ft, libname)
    local msg=[[
        _contents(ft,libname)

            gives back a function that prints a table of contents for 
            a module.

            INPUT:
                ft : a function table containing all functions
                libname : the library name to be used.
    ]]
    if ft=='help' then
        print(msg)
        return
    end
    namedcheck({"ft","libname"},{ft,libname},{"table","string"}, msg)
    local function contentsfun()
        print(libname)
        print(string.rep('-',#libname))
        for k,v in pairs(ft) do
            print("  "..k)
        end
    end
    all_packages.contents[libname] = contentsfun
    return contentsfun
end

function _help(ft, libname)
    local msg=[[
        _help(ft, libname)

            gives back a function that prints all the help messages for
            the declared functions in the function table 

            INPUT:
                ft : a function table containing all functions
                libname : the library name to be used.
    ]]
    if ft=='help' then
        print(msg)
        return
    end
    namedcheck({"ft","libname"},{ft,libname},{"table","string"}, msg)
    local function helpfun()
        print(libname)
        print(string.rep('-',#libname))
        for k,v in pairs(ft) do
            if k~='contents' and k~='help' then 
                print("\n")
                print(k..":")
                local status,err = pcall( function() v('help') end )
            end
        end
    end
    all_packages.help[libname] = helpfun
    return helpfun
end

function help(arg)
    printed=false
    if arg==nil or arg=="" then
        for k,v in pairs(all_packages.help) do
            v()
        end
        printed=true
    else
       for k,v in pairs(all_packages.help) do
            if k==arg then
                v()
                printed=true
            end
       end
    end
    if not printed then
        print([[
            help() without argument prints all help, help(name-of-package) prints the help 
            for that package
        ]])
    end
end

function contents(arg)
    printed=false
    if arg==nil or arg=="" then
        for k,v in pairs(all_packages.contents) do
            v()
        end
        printed=true
    else
       for k,v in pairs(all_packages.contents) do
            if k==arg then
                v()
                printed=true
            end
       end
    end
    if not printed then
        print([[
            contents() without argument prints all help, contents(name-of-package) prints the 
            contents for that package
        ]])
    end
end

function rtt_type( obj )
    -- infers some rtt_type based on the availability of some operations.
    local m = getmetatable(obj)
    if m~=nil then
        if m.start~=nil and m.stop~=nil and m.configure~=nil 
               and m.cleanup~=nil and m.getProperties~=nil then
            return "rtt_component"
        end
        if m.getOperationNames~=nil and m.getAttributeNames~=nil
           and m.getPropertyNames~=nil and m.getPortNames~=nil then
            return "rtt_service"
        end
        if m.getType~=nil and m.getTypeName~=nil and m.getTypeIdName~=nil then
            return obj:getTypeName()
        end
        --[[if m.getType~=nil then
            return obj:getType()
        end]]
    end
    return ""
end

-- gives the type, even if the object is of type userdata (cf. luabind):
function extendedtype(c)
    local msg=[[
        extendedtype(c)

        gives back an extended type description of its input.
        
        extendedtype recognizes luabind classes (eg. the expression_.. objects)
        but also RTT components and services
    ]]
    if c=='help' then
        print(msg)
        return
    end
    local t = type(c)
    if t=="userdata" then
        t2=rtt_type(c)
        if t2~="" then
            return t2
        end
        if class_info~=nil then 
            local ci=class_info(c)
            t = ci.name
            return t
        end
    end
    return t
end


-- you can specify the type or for userdata the class_info.name
-- "?" means that the argument is optional (can have nil value).
function check(argument,arg)
    local i
    local v
    for i,typedescr in ipairs(arg) do
        local t=extendedtype(argument[i])
        local typeerror = true;
        for v in string.gmatch(typedescr,"[^|]+")  do
            if v:sub(1,1)=="?" then
                if t=="nil" then 
                    typeerror = false 
                else
                    v = v:sub(2)
                end
            end
            if t==v  then
                typeerror = false 
            end
            if v=="namedvalues" then
                if (t=="table") then
                    typeerror = false
                    local pk
                    local pv
                    for pk,pv in pairs(argument[i]) do
                        if type(pk)~="string" or type(pv)~="number" then
                            typeerror = true
                        end
                    end
                end
            end
            if not typeerror then break end
        end
        if typeerror then
            error("type checking error : argument " .. tostring(i) .. " is of type \"" .. t .. "\" but should be of type \"" .. typedescr .. "\"",3)
        end
    end
end

-- you can specify the type or for userdata the class_info.name
-- "?" means that the argument is optional (can have nil value).
function namedcheck(names, argument ,checks, msg, errorlevel)
    local i
    local v
    local my_msg=[[
        namedcheck( names, argument, checks, msg, errorlevel)
        
        where
            names : a list of variable names to communicate to the user in
                    case of type errors.
            argument:  the variable to be checked
            checks: a type check specification.  This consists of a type names 
                   separated by '|'.  You can prepend a typename with '?' to indicate
                   that the parameter is optional (i.e. the function will deal with == nil).

                   additionally: 

                   "namedvalues" indicates a table with string keys and number values
                   "table_<type>" indicates a table with values of <type>.

            msg:    the message to give in case of error. Typically a description of the input
                    parameters
            errorlevel: (default value 3):
                    The level in the call stack where you want to report the error:
                        1: inside the namedcheck routine
                        2: inside the routine that wants to perform type checking
                        3: inside the routine that calls the function that performs typechecking.
    ]] 

    if msg==nil then
        msg=""
    end
    if errorlevel==nil then
        errorlevel=3
    end
    if names=='help' then
        print("\n")
        print(my_msg)
        print("\n")
        error('help message requested')
    end
    for i,typedescr in ipairs(checks) do
        local t=extendedtype(argument[i])
        local typeerror = true;
        for v in string.gmatch(typedescr,"[^|]+")  do
            if v:sub(1,1)=="?" then
                if t=="nil" then 
                    typeerror = false 
                else
                    v = v:sub(2)
                end
            end
            if t==v  then
                typeerror = false 
            end
            if v=="namedvalues" then
                if (t=="table") then
                    typeerror = false
                    local pk
                    local pv
                    for pk,pv in pairs(argument[i]) do
                        if type(pk)~="string" or type(pv)~="number" then
                            typeerror = true
                        end
                    end
                end
            end
            if v:sub(1,6)=="table_" then
                if (t=="table") then
                    typeerror = false
                    local pk
                    local pv
                    local eltype = v:sub(7)
                    for pk,pv in pairs(argument[i]) do
                        if extendedtype(pv)~=eltype then
                            typeerror = true
                            error("type checking error : argument " .. names[i] .. " has elements of type \"" .. extendedtype(pv) .. "\" but the array should be of type \"" .. typedescr .. "\"\n"..msg,errorlevel)
                        end
                    end
                end
            end
            if not typeerror then break end
        end
        if typeerror then
            --print(names[i])
            --print(t)
            --print(typedescr)
            error("type checking error : argument " .. names[i] .. " is of type \"" .. t .. "\" but should be of type \"" .. typedescr .. "\"\n"..msg,errorlevel)
            --error("type checking error : argument " .. names[i] .. " is of type \"" .. t .. "\" but should be of type \"" .. typedescr .. "\"",3)
        end
    end
end


function convert_to_expression(a)
    local msg=[[
        convert_to_expression(a)
    
        convert a value a to a constant expression if possible
    ]]
    if a=='help' then
        print(msg)
        return
    end
    if extendedtype(a)=="number" or 
       extendedtype(a)=="Vector" or
       extendedtype(a)=="Rotation" or
       extendedtype(a)=="Frame" or 
       extendedtype(a)=="Twist" then
        a=constant(a)
    end
    return a
end


function decorate_obj(obj, mtable, msg)
    local local_msg=[[
        decorate_obj

        decorates an object with the information given in the method table;
        returns a function that gives help  (for the function table) with
        a method summary.  Also declares a new method obj.help() in the object 
        to give detailed help for all methods.

        INPUT:
            obj:    object to decorate (not class!)
            mtable: a table with as key the name of an object method, and as
                    value a new function that replaces the method, the new
                    function should typecheck and call the original function
                    (and be able to react to the 'help' argument in the first 
                     OR THE SECOND argument)
            msg:    help description for this object
        OUTPUT:
            the adapted object
    ]]
    if obj=='help' then
        print(local_msg)
        return
    end
    namedcheck({"mtable","msg"},{mtable,msg},{"table","string"},local_msg)
    if obj==nil then
        obj={}
    end
    for k,v in pairs(mtable) do
        obj["_"..k] = obj[k]
        obj[k] = v
    end
    -- add help() method
    local function full_help(arg)
        print(msg)
        print("    MEMBERS:\n")
        for k,v in pairs(mtable) do
                print("  "..k..":\n")
                local status,err = pcall( function() v('help') end )
 
        end
    end 
    obj.help = full_help
    -- add contents() method
    local function my_contents(arg)
        print(msg)
        print("    MEMBERS:\n")
        for k,v in pairs(mtable) do
                print("  "..k)
        end
    end 
    obj.contents = my_contents
    return obj
end


local ftable={
    convert_to_expression   =   convert_to_expression,
    namedcheck              =   namedcheck,
    extendedtype            =   extendedtype,
    _help                   =   _help,
    _contents               =   _contents
}

ftable['contents'] = _contents(ftable,"type_checking")
ftable['help']     = _help(ftable,"type_checking")

return ftable
