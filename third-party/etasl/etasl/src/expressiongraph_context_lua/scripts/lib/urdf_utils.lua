--
-- library to facilitate the use of URDF-files.
--
-- It allows you to read URDF-files and customize how it is translated into eTaSL expressions and 
-- constraints
--
require("context")
require("type_checking")


local function contains(t, e)
  for i = 1,#t do
    if t[i] == e then return true end
  end
  return false
end

local function table_or_number(argname, jointnames, tbl, defval)
    if tbl==nil then
        tbl=defval
    end
    local returntbl={}
    for k,v in pairs(jointnames) do
        returntbl[v] = defval 
    end 
    if type(tbl)~="table" then
        for k,v in pairs(jointnames) do
           returntbl[v] = tbl
        end 
    end
    if type(tbl)=="table" then     
        for k,v in pairs(tbl) do
            if type(k)=='number' then
                if 1<=k and k<= #jointnames then
                    returntbl[ jointnames[k] ] = v
                else
                    error(argname..": table contains an index outside the range given by jointnames (1.."..#jointnames..")",2)
                end
            elseif type(k)=='string' then
                if contains(jointnames,k) then
                    returntbl[k] = v 
                else
                    error(argname..": table contains an key that is not in jointnames",2)
                end
            end 
        end 
    end
    return returntbl;
end

local function robot_cb(arg)
    local msg = [[

        robot_cb: a call back function to construct a virtual robot with
                          customized constraints.

           some of the arguments can be a number or a table. If it is a table, it is
           indexed using an index number or the name of the variable (given by jointnames). 
           The meaning of the index is given by the jointnames table. If a number is given 
           instead of a table,  the value is valid for all joints.  You can only specify
           some of the values for a table, the values for the rest of the joints will be
           the default value.

        INPUT:
            context:  
                context used for defining constraints and variables
            prefix:
                prefix to the names of all entities defined.
            jointnames:
                table of joint names (without prefix), used to define the order of the
                joints.
            vartype [number, optional, default='robot']:  
                the created variables will be of this type ('robot' or 'feature')
            
            jointexpr:
                table of expressions for the joint variable. If not given a variable will
                be created of variable type given by vartype

            pos_enable [boolean or table, optional, default=true]:
                position constraint is enabled for this variable
            pos_margin [number or table, optional, default=0]:  
                stay away from the limits by this margin. 
            pos_K [number or table, optional, default=0]:  
                position constraints are enforced using this gain. 
            pos_priority[number of table, optional, default=0]: 
                priority of position constraints.
            pos_weight[number of table, optional, default=0]: 
                weight of position constraints (when priority >= 2)

            vel_enable [boolean or table, optional, default=true]:
                velocity constraints are enabled for this variable
            vel_scale [number or table, optional, default=1]:
                the velocity constraints are scaled by this number.            
            vel_priority[number of table, optional, default=0]: 
                priority of velocity constraints.
            vel_weight[number of table, optional, default=0]: 
                weight of velocity constraints (when priority >= 2)

        RETURN
            callback function
            table of joint expressions
        EXAMPLE:

            require("context")
            require("geometric")

            urdf_utils=require("urdf_utils")

            local pref = ""
            local jname =
            { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" }
            local cb = urdf_utils.robot_cb{ 
                context    = ctx, 
                prefix     = pref, 
                jointnames = jname, 
                vartype    = 'feature',
                pos_margin = 0.05,
                vel_scale  = 0.5
            }

            local u=UrdfExpr()
            u:setJointsAndConstraintsCallback(cb)
            -- callback must be set before reading file!
            u:readFromFile(rospack_find("etasl_rtt_examples").."/robots/ur10_robot.urdf")
            u:addTransform("ee","ee_link","base_link")
            local r = u:getExpressions(ctx)

    ]]
    if arg==nil or arg=='help' then 
        print(msg)
        return
    end
    namedcheck({"context","prefix","jointnames","vartype","jointexpr"},
               {arg.context, arg.prefix, arg.jointnames,arg.vartype,arg.jointexpr},
               {"Context","string","table_string","?string|table_string","?table_expression_double"},
               msg)
    arg.vartype = table_or_number('vartype', arg.jointnames, arg.vartype, 'robot')
    if arg.jointexpr==nil then
        arg.jointexpr={}
    end
    local jointexpr={}
    for k,v in pairs(arg.jointexpr) do
        if type(k)=='number' then
            if 1<=k and k<= #arg.jointnames then
                jointexpr[ arg.jointnames[k] ] = v
            else
                error(argname..": table contains an index outside the range given by jointnames (1.."..#jointnames..")",1)
            end
        elseif type(k)=='string' then
            if contains(arg.jointnames,k) then
                jointexpr[k] = v 
            else
                error(argname..": table contains an key that is not in jointnames",1)
            end
        end
    end
    arg.jointexpr = jointexpr
    namedcheck({"pos_enable","pos_margin","pos_K","pos_priority","pos_weight"},
               {arg.pos_enable, arg.pos_margin, arg.pos_K, arg.pos_priority, arg.pos_weight},
               {"?boolean|table_boolean","?number|table_number","?number|table_number","?number|table_number","?number|table_number"},
               msg)
    arg.pos_enable   = table_or_number('vartype', arg.jointnames, arg.pos_enable, true)
    arg.pos_margin   = table_or_number('vartype', arg.jointnames, arg.pos_margin, 0.0)
    arg.pos_K        = table_or_number('vartype', arg.jointnames, arg.pos_K, 4)
    arg.pos_priority = table_or_number('vartype', arg.jointnames, arg.pos_priority, 0)
    arg.pos_weight   = table_or_number('vartype', arg.jointnames, arg.pos_weight, 1.0)
    namedcheck({"vel_enable","vel_scale","vel_priority","vel_weight"},
               {arg.vel_enable, arg.vel_scale, arg.vel_priority, arg.vel_weight},
               {"?boolean|table_boolean", "?number|table_number", "?number|table_number","?number|table_number"},
               msg)
    arg.vel_enable    = table_or_number('vartype', arg.jointnames, arg.vel_enable, true)
    arg.vel_scale     = table_or_number('vartype', arg.jointnames, arg.vel_scale, 1.0)
    arg.vel_priority  = table_or_number('vartype', arg.jointnames, arg.vel_priority, 0)
    arg.vel_weight    = table_or_number('vartype', arg.jointnames, arg.vel_weight, 1.0)
   
    -- for the joints that are not given, create a variable: 
    arg.jexpr={}
    for i = 1,#arg.jointnames, 1 do
        if arg.jointexpr[ arg.jointnames[i] ] == nil then
            arg.jexpr[ arg.jointnames[i] ] = 
                Variable{
                    context=ctx, 
                    vartype=arg.vartype[ arg.jointnames[i] ], 
                    name = arg.prefix..arg.jointnames[i] 
                }
        else
            arg.jexpr[ arg.jointnames[i] ] = arg.jointexpr[ arg.jointnames[i] ]
        end
    end
    local cb = function(ctx,name,vL,vU,pL,pU)
        local jv=arg.jexpr[name]
        if jv~= nil then
            if arg.pos_enable[ name ] then
                Constraint{
                    context=ctx,
                    name=arg.prefix.."posconstraint_on_"..name,
                    expr=jv,
                    target_lower=pL+arg.pos_margin[name],
                    target_upper=pU-arg.pos_margin[name],
                    K           = arg.pos_K[name],
                    priority    = arg.pos_priority[name],
                    weight      = arg.pos_weight[name] 
                }
            end
            if arg.vel_enable[ name ] then
                if arg.vel_priority[name] < 2 then
                    BoxConstraint{
                        context=ctx,
                        var_name=arg.prefix..name,
                        lower=vL*arg.vel_scale[name],
                        upper=vU*arg.vel_scale[name]
                    }
                else
                    Constraint{
                        context=ctx,
                        name=arg.prefix.."uppervelconstraint_on_"..name,
                        expr= jv - time*constant(vU*arg.vel_scale[name]),
                        target_upper=0,
                        K           = 0,
                        priority    = arg.pos_priority[name],
                        weight      = arg.pos_weight[name] 
                    }
                    Constraint{
                        context=ctx,
                        name=arg.prefix.."lowervelconstraint_on_"..name,
                        expr= jv - time*constant(vL*arg.vel_scale[name]),
                        target_lower= 0,
                        K           = 0,
                        priority    = arg.pos_priority[name],
                        weight      = arg.pos_weight[name] 
                    }
                end
            end
            -- you can also define a velocity constraint...
            return jv, name;
        else
            error("unknown joint name")
        end
    end
    return cb
end

local ftable = {
    robot_cb = robot_cb
}

ftable['help'] = _help(ftable, "urdf_utils")
ftable['contents'] = _contents(ftable,"urdf_utils")
return ftable
