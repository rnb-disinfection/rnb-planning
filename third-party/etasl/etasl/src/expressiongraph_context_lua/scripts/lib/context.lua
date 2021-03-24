-- use require("context") to define a context
--

-- in case it is called from plain lua, we create the ctx object ourself
-- (and therefore will not be used by a solver=, but this is handy for
-- checking the correct definitions in eTaSL)

require("type_checking")
if ctx==nil then
    require("libexpressiongraph_context_lua")
    ctx=Context()
    ctx:addType("robot")
    ctx:addType("feature")
    time = ctx:getScalarExpr("time")
    start_time = constant(0.0) 
end

if ilua~=nil then
    print( [[ 
        for help: store the return value of require and call help() or contents():
        e.g.  c=require("context")
              c.help() or c.contents()
        The global help() and contents() call the help and contents of all libraries
        ]]
    )
end


-- union of namedvalues :
function union(a,b)
    local k
    local v
    local res={}
    for k,v in pairs (a) do res[k]=v end
    for k,v in pairs (b) do res[k]=v end
    return res
end

function chk_add( a )
    if a == -1 then
        error("control constant should be >= 0",3)
    end
    if a == -1 then
        error("control constant should be >= 0",3)
    end
    if a == -2 then
        error("priority should be >= 0",3)
    end
    if a == -3 then
        error("name already exists",3)
    end
    if a == -4 then
        error("target_lower should be <= compared to target_upper",3)
    end
    if a == -5 then
        error("variable number should be >= 0",3)
    end
    if a == -10 then
        error("one of the tags in the list is not recognized",3)
    end

end


function filter_out_controllerparam( table )
    local retval={}
    for k,v in pairs(table) do
        if k ~= 'name' and
           k ~= 'context' and 
           k ~= 'model' and
           k ~= 'meas'  and
           k ~= 'expr'  and
           k ~= 'target' and
           k ~= 'target_lower' and
           k ~= 'target_upper' and
           k ~= 'weight'       and
           k ~= 'controller'   and
           k ~= 'controller_lower' and
           k ~= 'controller_upper' and
           k ~= 'priority' then
                if extendedtype(v)=='number' then
                    retval[k]= constant(v)
                elseif extendedtype(v)=='expression_double' then
                    retval[k]= v
                else
                    error("field " .. k .. " : additional fields of constraints are controller parameters and should be a number or a expression_double",3)
                end 
        end
    end
    return retval
end


function Constraint(arg)
    local HUGE_VALUE = 1E20
    local msg = "   Adaptation of the original Constraint(...) function that takes into account\n"..
                "   The original function is still available as Constraint_old (now obsolete) \n"..
                "   changes of the weight variables in its feedforward\n"..
                "   This function is called as follows ( not all combinations of arguments are possible ):\n" ..
                "   Constraint{\n"..
                "       context = ... \n" ..
                "       name = ...         [optional, default_name<nr>]\n" ..
                "       model = ...  (expression) \n"..
                "       meas = ...   (expression)\n"..
                "       expr = ...[ compatibility, if expr is used then model==expr and meas==expr ] \n"..
                "       target = ...       [optional, 0.0] ( can be expression )\n" ..
                "       target_lower = ... [optional, 0.0] ( can be expression )\n" ..
                "       target_upper = ... [optional, 0.0] ( can be expression )\n" ..
                "       weight = ...       [optional, defaultweight, >= 0] ( can be expression )\n"..
                "       priority = ...     [optional, defaultpriority, 0..2]\n"..
                "       controller_lower = ....  [optional, 'proportional']\n"..
                "       controller_upper = ....  [optional, 'proportional']\n"..
                "       controller       = ....  [optional, 'proportional']\n"..
                "       <controllerparameter> =... (can be expressions) \n"..
                "       <controllerparameter>_lower or <controllerparameter>_upper (can be expressions) \n"..
                "   }\n The xxx_lower and xxx_upper variables can only be used when the type of expr is expression_double; they\n"..
                "   can be scalars or scalar expressions.\n" ..
                "   If omitted context.defaultweight, context.defaultpriority, context.defaultK are used\n"..
                "   For other variables, the most evident default is taken or an error message is given if this does not exists.\n\n"..
                "   target_lower==-1E20 or target_upper==1E20 is used as a flag to indicate that this boundary is inactive, so\n"..
                "   setting these specific values will still work when K==0! \n"
    if arg=='help' then
        print(msg)
        return
    end

    if arg==nil then
        error( "Constraint expects a table as argument\n"..msg)
    end
    if arg.context==nil then
        error( "Constraint expects 'context' (Context) to be specified\n"..msg)
    end
    if arg.expr==nil then
        if arg.model==nil and arg.meas==nil then
            error( "Constraint expects either 'expr' or both 'model' and 'meas' to be specified\n"..msg)
        end
    end

    namedcheck({"context","expr"},{arg.context,arg.expr},
               {"Context","?expression_double|expression_vector|expression_rotation|expression_frame"},msg)
    namedcheck({"context","meas"},{arg.context,arg.meas},
               {"Context","?expression_double|expression_vector|expression_rotation|expression_frame"},msg)
    namedcheck({"context","model"},{arg.context,arg.model},
               {"Context","?expression_double|expression_vector|expression_rotation|expression_frame"},msg)

    namedcheck({"weight","name","priority","target"},
               {arg.weight,arg.name,arg.priority}, 
               {"?number|?expression_double", "?string","?number"},msg)
    namedcheck({"controller","controller_lower","controller_upper"},
               {arg.controller, arg.controller_lower, arg.controller_upper},
               {"?string","?string","?string"})

    if arg.priority == nil then
        arg.priority = ctx.defaultpriority
    end
    --if arg.priority~=0 and arg.priority~=1 and arg.priority~=2 then
    --    error("priority has a wrong value ( ".. arg.priority..")" ,2)
    --end
    if arg.weight == nil then
        arg.weight = ctx.defaultweight 
    end
    if arg.name == nil then
        arg.name = ctx.defaultname .. tostring(ctx.defaultname_counter)
        ctx.defaultname_counter = ctx.defaultname_counter + 1
    end  
    -- related to target:
    if arg.target_lower ~=nil and arg.target~=nil then
        error("you cannot specify target_lower or target_upper and at the same time specify target \n"..msg,2)
    end 
    if arg.target_upper ~=nil and arg.target~=nil then
        error("you cannot specify target_lower or target_upper and at the same time specify target  \n"..msg,2)
    end 
     -- related to controller:
    if (arg.controller_lower ~=nil or arg.controller_upper~=nil) then
        if arg.controller ~=nil then
            error("you cannot specify controller_lower or controller_upper and at the same time specify controller \n"..msg,2)
        end 
    else 
        arg.controller_lower=arg.controller
        arg.controller_upper=arg.controller
    end 
    if (arg.controller_lower==nil) then
        arg.controller_lower="proportional"
    end
    if (arg.controller_upper==nil) then
        arg.controller_upper="proportional"
    end



    -- related to meas, model and expr
    if arg.expr~=nil then
        if ( arg.model~=nil or arg.meas~=nil) then
            error("when the expr is specified, you cannot specify model or meas"..msg,2) 
        end
        arg.model = arg.expr
        arg.meas  = arg.expr
    else
        if arg.model==nil then
            error("the 'model' expression for the constraint should be specified"..msg,2) 
        end
        if arg.meas==nil then
            error("the 'meas' expression for the constraint should be specified"..msg,2) 
        end
    end  
    if not(  extendedtype(arg.model)==extendedtype(arg.meas) ) then
        error('model and meas should be of the same type (expression...)',2)
    end

    local param  = filter_out_controllerparam( arg);

    local t= extendedtype(arg.model)
    
    local function weight_ff(w)
        local eps = constant(1e-14)
        --if is_constant(w) then
        --    return w
        --else
            return cached(w/make_constant(w+eps))
        --end
    end

    if t~="expression_double" then
       if arg.target_upper ~=nil or arg.target_lower ~= nil then
           error("when an expression_rotation/frame/vector model is used, you can only use target and not target_lower or target_upper")
       end
       if arg.controller_upper ~= arg.controller_lower then
            error("if the constraint is not scalar, only equality constraints are allowed and controller_upper == controller_lower");
       end
       if t=="expression_rotation" then  
            if extendedtype(arg.weight) ~= "expression_double" then
               arg.weight = constant( arg.weight ) 
            end
            local root_weight = weight_ff(arg.weight)
            if arg.target == nil then
                arg.target = Rotation.Identity()
            end
            local model
            local meas
            if extendedtype(arg.target)=="Rotation" then
                model          = cached( getRotVec(arg.model*inv(constant(arg.target))))
                meas           = cached( getRotVec(arg.meas*inv(constant(arg.target))))
            elseif extendedtype(arg.target)=="expression_rotation" then
                model          = cached( getRotVec(arg.model*inv(arg.target)))
                meas           = cached( getRotVec(arg.meas*inv(arg.target)))
            else
                error("when an expression_rotation model is used, target should be of type Rotation or expression_rotation\n"..msg,2)
            end
            param['model'] = coord_x(model)*root_weight
            param['meas']  = coord_x(meas)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotx",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_y(model)*root_weight
            param['meas']  = coord_y(meas)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_roty",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_z(model)*root_weight
            param['meas']  = coord_z(meas)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotz",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
        end
        if t=="expression_vector" then
           if extendedtype(arg.weight) ~= "expression_double" then
               arg.weight = constant( arg.weight ) 
           end
           local root_weight = weight_ff(arg.weight)
           if arg.target == nil then
                arg.target = Vector.Zero()
            end
            local model
            local meas
            if extendedtype(arg.target)=="Vector" then
                model = cached(arg.model-constant(arg.target))
                meas  = cached(arg.meas -constant(arg.target))
            elseif extendedtype(arg.target)=="expression_vector" then
                model = cached(arg.model - arg.target)
                meas  = cached(arg.meas  - arg.target)
            else
                error("when an expression_vector expression is used, target should be of type Vector or expression_vector\n"..msg,2)
            end
            param['model'] = coord_x(model)*root_weight
            param['meas']  = coord_x(meas)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_x",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_y(model)*root_weight
            param['meas']  = coord_y(meas)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_y",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_z(model)*root_weight
            param['meas']  = coord_z(meas)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_z",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
        end 
        if t=="expression_frame" then
           if extendedtype(arg.weight) ~= "expression_double" then
               arg.weight = constant( arg.weight ) 
           end
           local root_weight = weight_ff(arg.weight)
           if arg.target == nil then
                arg.target = Frame.Identity()
            end
            local model
            local meas
            if extendedtype(arg.target)=="Frame" then
                model = cached(arg.model*inv(constant(arg.target)))
                meas  = cached(arg.meas*inv(constant(arg.target)))
            elseif extendedtype(arg.target)=="expression_frame" then
                model = cached(arg.model*inv(arg.target))
                meas  = cached(arg.meas *inv(arg.target))
            else
                error("when an expression_frame expression is used, target should be of type Frame or expression_frame\n"..msg,2)
            end
            param['model'] = coord_x(origin(model))*root_weight
            param['meas']  = coord_x(origin(meas))*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_x",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_y(origin(model))*root_weight
            param['meas']  = coord_y(origin(meas))*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_y",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_z(origin(model))*root_weight
            param['meas']  = coord_z(origin(meas))*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_z",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )

            local modelr = getRotVec(rotation(model))
            local measr  = getRotVec(rotation(meas))
            param['model'] = coord_x(modelr)*root_weight
            param['meas']  = coord_x(measr)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotx",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_y(modelr)*root_weight
            param['meas']  = coord_y(measr)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_roty",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
            param['model'] = coord_z(modelr)*root_weight
            param['meas']  = coord_z(measr)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotz",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight ) )
        end 
        return
    end
    -- t == expression_double
    -- double: the case of using target (with number/expr) 
    if arg.target_lower==nil and arg.target_upper==nil then
        if arg.target==nil then
            arg.target = 0.0
        end
        if extendedtype(arg.target)=="number" then
            arg.target = constant( arg.target )
        end
        if extendedtype(arg.weight)=="number" then
            param['model'] = arg.model - arg.target
            param['meas']  = arg.meas  - arg.target
            chk_add( arg.context:addInequalityConstraint(arg.name,0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, constant(arg.weight)) )
            return
        else
            local root_weight = weight_ff(arg.weight)
            param['model'] = (arg.model - arg.target) * root_weight
            param['meas']  = (arg.meas  - arg.target)* root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name,0.0, 0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            return
        end
    end
    if (arg.target_lower~=nil or arg.target_upper~=nil) and arg.target~=nil then
        error("cannot specify target_lower/target_upper AND target "..msg,2)
    end
    -- double: the case of using target_lower/target_upper with numbers for target and weight:
    if extendedtype(arg.target_lower)=="number" and arg.target_upper==nil and extendedtype(arg.weight)=="number" then
        arg.target_upper = HUGE_VALUE
    end
    if extendedtype(arg.target_upper)=="number" and arg.target_lower==nil and extendedtype(arg.weight)=="number" then
        arg.target_lower = -HUGE_VALUE
    end
    if extendedtype(arg.target_lower)=="number" and extendedtype(arg.target_upper)=="number" and extendedtype(arg.weight)=="number" then
            param['model'] = arg.model 
            param['meas']  = arg.meas 
            chk_add( arg.context:addInequalityConstraint(arg.name,arg.target_lower,arg.target_upper, arg.controller_lower, arg.controller_upper, param, arg.priority, constant(arg.weight)) )
            return
    end
    -- double: the general case, we'll need a seperate constraint for each upper/lower limit:
    if extendedtype(arg.weight)=="number" then
        arg.weight = constant(arg.weight)
    end
    if extendedtype(arg.target_lower)=="number" then
        arg.target_lower = constant(arg.target_lower)
    end
    if extendedtype(arg.target_upper)=="number" then
        arg.target_upper = constant(arg.target_upper)
    end

    local root_weight = weight_ff(arg.weight)
    if arg.target_lower~=nil then
            param['model'] = (arg.model - arg.target_lower)*root_weight
            param['meas']  = (arg.meas  - arg.target_lower)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_lower",  0.0,   HUGE_VALUE, arg.controller_lower, arg.controller_upper, 
                                                     param, arg.priority, arg.weight ) )
    end
    if arg.target_upper~=nil then 
            param['model'] = (arg.model - arg.target_upper)*root_weight
            param['meas']  = (arg.meas  - arg.target_upper)*root_weight
            chk_add( arg.context:addInequalityConstraint(arg.name.."_upper",  -HUGE_VALUE,   0.0, arg.controller_lower, arg.controller_upper, 
                                                         param, arg.priority, arg.weight ) )
    end 
    return
end 

--this is the old constraint routine:
function Constraint_old(arg)
    local HUGE_VALUE = 1E20
    local msg = "This function is called as follows ( not all combinations of arguments are possible ):\n" ..
                "Constraint{\n"..
                "   context = ... \n" ..
                "   name = ...         [optional, default_name<nr>]\n" ..
                "   model = ...  (expression) \n"..
                "   meas = ...   (expression)\n"..
                "   expr = ...[ compatibility, if expr is used then model==expr and meas==expr ] \n"..
                "   target = ...       [optional, 0.0] ( can be expression )\n" ..
                "   target_lower = ... [optional, 0.0] ( can be expression )\n" ..
                "   target_upper = ... [optional, 0.0] ( can be expression )\n" ..
                "   weight = ...       [optional, defaultweight, >= 0] ( can be expression )\n"..
                "   priority = ...     [optional, defaultpriority, 0..2]\n"..
                "   controller_lower = ....  [optional, 'proportional']\n"..
                "   controller_upper = ....  [optional, 'proportional']\n"..
                "   controller       = ....  [optional, 'proportional']\n"..
                "   <controllerparameter> =... (can be expressions) \n"..
                "   <controllerparameter>_lower or <controllerparameter>_upper (can be expressions) \n"..
                "}\n The xxx_lower and xxx_upper variables can only be used when the type of expr is expression_double; they\n"..
                "can be scalars or scalar expressions.\n" ..
                "If omitted context.defaultweight, context.defaultpriority, context.defaultK are used\n"..
                "For other variables, the most evident default is taken or an error message is given if this does not exists.\n\n"..
                "target_lower==-1E20 or target_upper==1E20 is used as a flag to indicate that this boundary is inactive, so\n"..
                "setting these specific values will still work when K==0! \n"

    if arg==nil then
        error( "Constraint expects a table as argument\n"..msg)
    end
    if arg.context==nil then
        error( "Constraint expects 'context' (Context) to be specified\n"..msg)
    end
    if arg.expr==nil then
        if arg.model==nil and arg.meas==nil then
            error( "Constraint expects either 'expr' or both 'model' and 'meas' to be specified\n"..msg)
        end
    end

    namedcheck({"context","expr"},{arg.context,arg.expr},
               {"Context","?expression_double|expression_vector|expression_rotation|expression_frame"},msg)
    namedcheck({"context","meas"},{arg.context,arg.meas},
               {"Context","?expression_double|expression_vector|expression_rotation|expression_frame"},msg)
    namedcheck({"context","model"},{arg.context,arg.model},
               {"Context","?expression_double|expression_vector|expression_rotation|expression_frame"},msg)

    namedcheck({"weight","name","priority","target"},
               {arg.weight,arg.name,arg.priority}, 
               {"?number|?expression_double", "?string","?number"},msg)
    namedcheck({"controller","controller_lower","controller_upper"},
               {arg.controller, arg.controller_lower, arg.controller_upper},
               {"?string","?string","?string"})

    if arg.priority == nil then
        arg.priority = ctx.defaultpriority
    end
    --if arg.priority~=0 and arg.priority~=1 and arg.priority~=2 then
    --    error("priority has a wrong value ( "+tostring(arg.priority),2)
    --end
    if arg.weight == nil then
        arg.weight = ctx.defaultweight 
    end
    if extendedtype(arg.weight) ~= "expression_double" then
       arg.weight = constant( arg.weight ) 
    end
    if arg.name == nil then
        arg.name = ctx.defaultname .. tostring(ctx.defaultname_counter)
        ctx.defaultname_counter = ctx.defaultname_counter + 1
    end  
    -- related to target:
    if arg.target_lower ~=nil and arg.target~=nil then
        error("you cannot specify target_lower or target_upper and at the same time specify target \n"..msg,2)
    end 
    if arg.target_upper ~=nil and arg.target~=nil then
        error("you cannot specify target_lower or target_upper and at the same time specify target  \n"..msg,2)
    end 
     -- related to controller:
    if (arg.controller_lower ~=nil or arg.controller_upper~=nil) then
        if arg.controller ~=nil then
            error("you cannot specify controller_lower or controller_upper and at the same time specify controller \n"..msg,2)
        end 
    else 
        arg.controller_lower=arg.controller
        arg.controller_upper=arg.controller
    end 
    if (arg.controller_lower==nil) then
        arg.controller_lower="proportional"
    end
    if (arg.controller_upper==nil) then
        arg.controller_upper="proportional"
    end



    -- related to meas, model and expr
    if arg.expr~=nil then
        if ( arg.model~=nil or arg.meas~=nil) then
            error("when the expr is specified, you cannot specify model or meas"..msg,2) 
        end
        arg.model = arg.expr
        arg.meas  = arg.expr
    else
        if arg.model==nil then
            error("the 'model' expression for the constraint should be specified"..msg,2) 
        end
        if arg.meas==nil then
            error("the 'meas' expression for the constraint should be specified"..msg,2) 
        end
    end  
    if not(  extendedtype(arg.model)==extendedtype(arg.meas) ) then
        error('model and meas should be of the same type (expression...)',2)
    end

    local param  = filter_out_controllerparam( arg);

    local t= extendedtype(arg.model)
    if t~="expression_double" then
       if arg.target_upper ~=nil or arg.target_lower ~= nil then
           error("when an expression_rotation/frame/vector model is used, you can only use target and not target_lower or target_upper")
       end
       if arg.controller_upper ~= arg.controller_lower then
            error("if the constraint is not scalar, only equality constraints are allowed and controller_upper == controller_lower");
       end
       if t=="expression_rotation" then  
            if arg.target == nil then
                arg.target = Rotation.Identity()
            end
            local model
            local meas
            if extendedtype(arg.target)=="Rotation" then
                model          = cached( getRotVec(arg.model*inv(constant(arg.target))))
                meas           = cached( getRotVec(arg.meas*inv(constant(arg.target))))
            elseif extendedtype(arg.target)=="expression_rotation" then
                model          = cached( getRotVec(arg.model*inv(arg.target)))
                meas           = cached( getRotVec(arg.meas*inv(arg.target)))
            else
                error("when an expression_rotation model is used, target should be of type Rotation or expression_rotation\n"..msg,2)
            end
            param['model'] = coord_x(model)
            param['meas']  = coord_x(meas)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotx",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_y(model)
            param['meas']  = coord_y(meas)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_roty",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_z(model)
            param['meas']  = coord_z(meas)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotz",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
        end
        if t=="expression_vector" then
           if arg.target == nil then
                arg.target = Vector.Zero()
            end
            local model
            local meas
            if extendedtype(arg.target)=="Vector" then
                model = cached(arg.model-constant(arg.target))
                meas  = cached(arg.meas -constant(arg.target))
            elseif extendedtype(arg.target)=="expression_vector" then
                model = cached(arg.model - arg.target)
                meas  = cached(arg.meas  - arg.target)
            else
                error("when an expression_vector expression is used, target should be of type Vector or expression_vector\n"..msg,2)
            end
            param['model'] = coord_x(model)
            param['meas']  = coord_x(meas)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_x",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_y(model)
            param['meas']  = coord_y(meas)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_y",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_z(model)
            param['meas']  = coord_z(meas)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_z",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
        end 
        if t=="expression_frame" then
           if arg.target == nil then
                arg.target = Frame.Identity()
            end
            local model
            local meas
            if extendedtype(arg.target)=="Frame" then
                model = cached(arg.model*inv(constant(arg.target)))
                meas  = cached(arg.meas*inv(constant(arg.target)))
            elseif extendedtype(arg.target)=="expression_frame" then
                model = cached(arg.model*inv(arg.target))
                meas  = cached(arg.meas *inv(arg.target))
            else
                error("when an expression_frame expression is used, target should be of type Frame or expression_frame\n"..msg,2)
            end
            param['model'] = coord_x(origin(model))
            param['meas']  = coord_x(origin(meas))
            chk_add( arg.context:addInequalityConstraint(arg.name.."_x",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_y(origin(model))
            param['meas']  = coord_y(origin(meas))
            chk_add( arg.context:addInequalityConstraint(arg.name.."_y",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_z(origin(model))
            param['meas']  = coord_z(origin(meas))
            chk_add( arg.context:addInequalityConstraint(arg.name.."_z",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )

            local modelr = getRotVec(rotation(model))
            local measr  = getRotVec(rotation(meas))
            param['model'] = coord_x(modelr)
            param['meas']  = coord_x(measr)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotx",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_y(modelr)
            param['meas']  = coord_y(measr)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_roty",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
            param['model'] = coord_z(modelr)
            param['meas']  = coord_z(measr)
            chk_add( arg.context:addInequalityConstraint(arg.name.."_rotz",0.0,0.0, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
        end 
        return
    end
    -- t == expression_double
    if arg.target_lower ~=nil and arg.target_upper==nil then
        arg.target_upper = HUGE_VALUE
    end 
    if arg.target_lower ==nil and arg.target_upper~=nil then
        arg.target_lower = -HUGE_VALUE
    end 
    if (arg.target~=nil) then
        arg.target_lower = arg.target
        arg.target_upper = arg.target
    end
    if arg.target==nil and arg.target_lower==nil and arg.target_upper==nil then
        arg.target_lower = 0.0
        arg.target_upper = 0.0
    end

    if extendedtype(arg.target_lower)~="number" and extendedtype(arg.target_lower)~="expression_double" then           
        error("when an expression_double expression is used, target,target_lower or target_upper should be of type number or expression_double\n"..msg,2)
    end
    if extendedtype(arg.target_upper)~="number" and extendedtype(arg.target_upper)~="expression_double" then
        error("when an expression_double expression is used, target,target_lower or target_upper should be of type number or expression_double\n"..msg,2)
    end

    if extendedtype(arg.target_lower)=="number" and extendedtype(arg.target_upper)=="number" then 
        param['model'] = arg.model 
        param['meas']  = arg.meas 
        chk_add( arg.context:addInequalityConstraint(arg.name,arg.target_lower,arg.target_upper, arg.controller_lower, arg.controller_upper, param, arg.priority, arg.weight) )
    else
        if extendedtype(arg.target_lower)=="number" then
            arg.target_lower = constant( arg.target_lower)
        end
        if extendedtype(arg.target_upper)=="number" then
            arg.target_upper = constant( arg.target_upper)
        end
        param['model'] = arg.model - arg.target_lower
        param['meas']  = arg.meas  - arg.target_lower
        chk_add( arg.context:addInequalityConstraint(arg.name.."_lower",  0.0,   HUGE_VALUE, arg.controller_lower, arg.controller_upper, 
                                                     param, arg.priority, arg.weight) )
        param['model'] = arg.model - arg.target_upper
        param['meas']  = arg.meas  - arg.target_upper
        chk_add( arg.context:addInequalityConstraint(arg.name.."_upper",  -HUGE_VALUE,   0.0, arg.controller_lower, arg.controller_upper, 
                                                     param, arg.priority, arg.weight) )
    end
end 




function BoxConstraint(arg)
    local HUGE_VALUE = 1E20
    local msg=  "   BoxConstraint{\n"..
                "       context=...,   [context to use]\n"..
                "       var_name=..,    [variable of which you want to constrain the velocity]\n"..
                "       lower=..,       [optional, default -large_value][lower velocity bound]\n".. 
                "       upper=..,       [optional, default +large_value][upper velocity bound]\n".. 
                "       tags=..         [optional, default ''][tags for this constraint]\n"..
                "   }\n"
    if arg=='help' then
        print(msg);
        return
    end
    namedcheck({"context","var_name","lower","upper","tags"},
               {arg.context, arg.var_name, arg.lower, arg.upper,arg.tags},
               {"Context","string","?number","?number","?string"},msg)
    if arg.lower == nil and arg.upper == 0 then
        error("lower and upper cannot be both unspecified\n\n"..msg,2)
    end
    if arg.lower == nil then
        arg.lower = -HUGE_VALUE
    end
    if arg.upper == nil then
        arg.upper = HUGE_VALUE
    end
    if arg.tags == nil then
        arg.tags = ''
    end
    local ndx = ctx:getScalarNdx(arg.var_name)
    if ndx==-1 then
        error("unknown variable name("..arg.var_name.." given\n\b"..msg,2)
    end
    local retval
    retval = ctx:addBoxConstraint(arg.var_name, ndx, arg.lower, arg.upper,arg.tags)
    if retval~=0 then
        error("addBoxConstraint failed: could not add the constraint\n\n"..msg)
    end
end

--[[

-- not working because of missing diff/addDelta for the expressiongraphs.

function VelocityConstraint(arg)
    local msg = "\nThis function is called as follows ( not all combinations of arguments are possible ):\n" ..
                "VelocityConstraint{\n"..
                "   context         = ... \n" ..
                "   name            = ...    [optional, default_name<nr>]\n" ..
                "   expr            = ... (! position-level expression !)\n" ..
                "   target_velocity = ...       [optional, 0.0]\n" ..
                "   weight          = ...       [optional, defaultweight] (scalar)\n" ..
                "   priority        = ...     [optional, defaultpriority]\n"
                "}\n ";
    if arg==nil or arg.context==nil or arg.expr==nil then
        error( "VelocityConstraint expects at least context and  expr"..msg)
    end
    namedcheck({"context","name","expr","target_velocity","weight","priority"},
               {arg.context,arg.name,arg.expr, arg.target_velocity, arg.weight, arg.priority},
               {"Context","?string","expression_double|expression_vector|expression_rotation|expression_frame",
                "?number|?Vector|?Twist",
                "?number|?expression_double","?number"},msg)
    if arg.target_velocity==nil then
        if (extendedtype(arg.expr) =="expression_double") then
            arg.target_velocity = 0.0;
        end
        if (extendedtype(arg.expr) =="expression_vector")  then
            arg.target_velocity = Vector.Zero();
        end
        if (extendedtype(arg.expr) =="expression_rotation") then
            arg.target_velocity = Vector.Zero();
        end
        if (extendedtype(arg.expr) =="expression_frame")  then
            arg.target_velocity = Twist.Zero();
        end
    end
    if arg.weight==nil then
        arg.weight = 1
    end
    if arg.priority==nil then
        arg.priority = 2
    end 
    if arg.name==nil then
        if velconstraint_counter==nil then
            velconstraint_counter=0
        end
        arg.name = "velocityconstraint_" .. tostring(monitor_counter)
        velconstraint_counter = velconstraint_counter + 1
    end 
    if (extendedtype(arg.expr) =="expression_double") and (extendedtype(arg.target_velocity)~="number") then
        error('type of target_velocity should be of the derivative type of expr\n'..msg)
    end
    if (extendedtype(arg.expr) =="expression_vector") and (extendedtype(arg.target_velocity)~="Vector") then
        error('type of target_velocity should be of the derivative type of expr\n'..msg)
    end
    if (extendedtype(arg.expr) =="expression_rotation") and (extendedtype(arg.target_velocity)~="Vector") then
        error('type of target_velocity should be of the derivative type of expr\n'..msg)
    end
    if (extendedtype(arg.expr) =="expression_frame") and (extendedtype(arg.target_velocity)~="Twist") then
        error('type of target_velocity should be of the derivative type of expr\n'..msg)
    end

    Constraint{
        context = arg.context,
        name    = arg.name,
        expr    = arg.expr - arg.target_velocity*time,
        K       = 0.0,
        weight  = arg.weight,
        priority= arg.priority
    }
end
--]]

function VelocityConstraint(arg)
    local msg = "\nThis function is called as follows ( not all combinations of arguments are possible ):\n" ..
                "VelocityConstraint{\n"..
                "   context         = ... \n" ..
                "   name            = ...    [optional, default_name<nr>]\n" ..
                "   expr            = ... (! scalar position-level expression !)\n" ..
                "   target_velocity = ...       [optional, 0.0]\n" ..
                "   weight          = ...       [optional, defaultweight] (scalar)\n" ..
                "   priority        = ...     [optional, defaultpriority]\n"..
                "}\n ";
    if arg=='help' then
        print(msg)
        return;
    end 
    if arg==nil or arg.context==nil or arg.expr==nil then
        error( "VelocityConstraint expects at least context and  expr"..msg)
    end
    namedcheck({"context","name","expr","target_velocity","weight","priority"},
               {arg.context,arg.name,arg.expr, arg.target_velocity, arg.weight, arg.priority},
               {"Context","?string","expression_double", "?number","?number|?expression_double","?number"},msg)
    if arg.target_velocity==nil then
        arg.target_velocity = 0.0;
    end
    if arg.weight==nil then
        arg.weight = 1
    end
    if arg.priority==nil then
        arg.priority = 2
    end 
    if arg.name==nil then
        if velconstraint_counter==nil then
            velconstraint_counter=0
        end
        arg.name = "velocityconstraint_" .. tostring(velconstraint_counter)
        velconstraint_counter = velconstraint_counter + 1
    end 

    Constraint{
        context = arg.context,
        name    = arg.name,
        expr    = arg.expr - constant(arg.target_velocity)*time,
        K       = 0.0,
        weight  = arg.weight,
        priority= arg.priority
    }
end

function Monitor(arg)
    local msg = "This function is called as follows :\n" ..
                "Monitor {\n" ..
                "   context = \n" ..
                "   name    = [optional, default_name<nr>]\n"..
                "   expr    = ...\n"..
                "   lower   = ... [optional, default -infinity]\n"..
                "   upper   = ... [optional, default +infinity]\n"..
                "   actionname    = ... \n"..
                "   argument= ...[optional, '']\n"..
                "}\n note: lower and upper cannot be both unspecified\n";
    if arg=='help' then
        print(msg)
        return
    end
    if arg==nil or arg.context==nil or arg.expr==nil or arg.actionname==nil then
        error( "\nConstraint expects at least a table with a context (Context), \nexpr (expression to monitor) and an actionname\n\n"..msg)
    end
    namedcheck({"context","expr"},{arg.context,arg.expr}, {"Context","expression_double"},msg)
    namedcheck({"lower","upper","name","actionname","argument"}, {arg.lower,arg.upper,arg.name,arg.actionname,arg.argument},  {"?number","?number","?string","string","?string"},msg)
    if arg.lower==nil and arg.upper==nil then
        error("\nlower and upper are both unspecified\n\n"..msg);
    end
    if arg.lower==nil then
        arg.lower=-1E100
    end
    if arg.upper==nil then
        arg.upper=1E100
    end
    if arg.name==nil then
        if monitor_counter==nil then
            monitor_counter=0
        end
        arg.name = "monitor" .. tostring(monitor_counter)
        monitor_counter = monitor_counter + 1
    end
    if arg.argument==nil then
        arg.argument=""
    end
    chk_add( arg.context:addMonitor( arg.name, arg.expr, arg.lower, arg.upper, arg.actionname, arg.argument ) );
end

function Variable(arg)
    local msg = "This function is called as follows :\n" ..
                "Variable {\n" ..
                "   context  = ...\n" ..
                "   name     = ...\n"..
                "   vartype  = ... [optional, default 'feature']\n"..
                "   initial  = ... [optional, default 0.0]\n"..
                "   weight   = ... [optional, default 1.0]\n"..
                "}\n\nreturns an expression graph pointing to the variable\n\n";
    if arg=='help' then
        print(msg)
        return
    end
    if arg==nil or arg.context==nil or arg.name==nil  then
        error( "\nConstraint expects at least a table with a context (Context), and a name  \n\n"..msg)
    end
    namedcheck({"context","name","vartype","initial","weight"},
               {arg.context,arg.name,arg.vartype,arg.initial, arg.weight}, 
               {"Context","string","?string","?number","?expression_double|number"},msg)
    if arg.initial==nil then
        arg.initial = 0.0;
    end
    if arg.vartype==nil then
        arg.vartype="feature"
    end
    if arg.weight==nil then
        arg.weight=constant(1)
    end
    if extendedtype(arg.weight)=="number" then
        arg.weight = constant( arg.weight )
    end
    local res = arg.context:addScalarVariable(arg.name,arg.vartype, arg.initial, arg.weight)
    return res
end


function deriv( e, varname )
    local msg= [[
        deriv(e, varname)
        
        INPUT:
            e:  an expression
            varname: the name of a variable
        OUTPUT:
            the partial derivative of the expression towards the variable.
    ]]
    if e=='help' then
        print(msg)
        return
    end
    namedcheck({"e","varname"},
               {e,varname},
               {"expression_double|expression_vector|expression_rotation|expression_frame|expression_twist|expression_wrench","string"},msg)
    local nr = ctx:getScalarNdx(varname)
    if (nr==-1) then
        error("deriv(...) : variable name does not exists")
    end
    return e:derivativeExpression(nr)
end


function addJointsToOutput(u,ctx, typename)
    msg = "addJointsToOutput(u,ctx,typename) adds all joints of a robot as an output\n"..
          "  u is an UrdfExpr object, ctx is a Context object and typename is the handler name\n"..
          "  for the output\n\n";
    if u=='help' then
        print(msg)
        return
    end
    namedcheck({"u","ctx","typename"},{u,ctx,typename},{"UrdfExpr","Context","string"},msg);
    local names=u:getAllJointNames()
    for i,n in pairs(names) do
        e=ctx:getScalarExpr(n)
        if e~=nil then 
            ctx:addOutput(n,typename,ctx:getScalarExpr(n))
        end
    end
end 

function addJointsToOutputExpression(u,ctx)
    msg = "addJointsToOutput(u,ctx) adds all joints of a robot as an output expression\n"..
          "  u is an UrdfExpr object, ctx is a Context object \n"..
          "  for the output\n\n";
    if u=='help' then
        print(msg)
        return
    end
    namedcheck({"u","ctx"},{u,ctx},{"UrdfExpr","Context"},msg);
    local names=u:getAllJointNames()
    for i,n in pairs(names) do
        e=ctx:getScalarExpr(n)
        if e~=nil then 
            ctx:setOutputExpression(n,ctx:getScalarExpr(n))
        end 
    end 
end 

-- uses roslines to output a frame representation to ROS.
-- X: red, Y: blue, Z: cyan
--
function outputFrame(ctx,name,F)
   namedcheck({"ctx","name",F},
               {ctx,name,F},
               {"Context","string","expression_frame"})
  
    local scale = constant(0.1);
    local zero  = constant(0.0);
    local o1 = F*vector(zero,zero,zero)
    local ox = F*vector(scale,zero,zero)
    local oy = F*vector(zero,scale,zero)
    local oz = F*vector(zero,zero,scale)
    ctx:addOutput(name.."_X", "roslines_1",o1)
    ctx:addOutput(name.."_X", "roslines_1",ox)
    ctx:addOutput(name.."_Y", "roslines_2",o1)
    ctx:addOutput(name.."_Y", "roslines_2",oy)
    ctx:addOutput(name.."_Z", "roslines_3",o1)
    ctx:addOutput(name.."_Z", "roslines_3",oz)
end
    

ctx.defaultK = 4
ctx.defaultweight = 1
ctx.defaultname = "no_name"
ctx.defaultname_counter = 1
ctx.defaultpriority = 2


-----------------------------------------------------------------------------------------------------
-- additional functions
-----------------------------------------------------------------------------------------------------

local _rospack_find = rospack_find 
function rospack_find( arg )
    local msg=[[
        rospack_find(arg)

        find a ros package directory using the ros package name.

        INPUT
            arg:  ros package name

    ]]
    if arg=="help" then
        print(msg)
        return
    end
    namedcheck({"arg"},{arg},{"string"},msg)
    return _rospack_find(arg)
end


-----------------------------------------------------------------------------------------------------
-- Operators
-----------------------------------------------------------------------------------------------------

_vector = vector
function vector(a,b,c)
    local msg=[[
        vector(a,b,c)

        create an vector expression with the given elements.

        INPUT:
            a,b,c:    elements of the vector, can be expression or value

        (constant numbers are automatically cast to expressions)
    ]]
    if a=="help" or a==nil then
        print(msg)
        return
    end
    namedcheck({"a","b","c"},{a,b,c},{"number|expression_double", "number|expression_double","number|expression_double"},msg)
    a=convert_to_expression(a)
    b=convert_to_expression(b)
    c=convert_to_expression(c)
    return _vector(a,b,c)
end



function binary_op(op, symb)
    local encapsulated_op = function (a,b)
        a=convert_to_expression(a)
        b=convert_to_expression(b)
        local status,retval= pcall( function() return op(a,b) end )
        if not status then
            retval = "Operator "..symb.." : extended type of first argument is "..extendedtype(a).." and of second argument is "..extendedtype(b).."\n"..retval
            error(retval,2)
        else
            return retval
        end
    end
    return encapsulated_op
end

function negate(a)
    return unm(a)
end


pi = 3.14159265359

local c=constant(1.0);
local mt = getmetatable(c)
mt.__add = binary_op(add,"+")
mt.__sub = binary_op(sub,"-") 
mt.__div = binary_op(div,"/")
mt.__mul = binary_op(mul,"*")
mt.__unm = negate

local v = vector(c,c,c)
mt = getmetatable(v)
mt.__add = binary_op(add,"+") 
mt.__sub = binary_op(sub,"-") 
mt.__unm = negate

local r = rot_x(c);
mt = getmetatable(r)
mt.__mul = binary_op(mul,"*")

local f = frame(v)
mt = getmetatable(f)
mt.__mul = binary_op(mul,"*")

local t = twist(v,v)
mt = getmetatable(t)
mt.__mul = binary_op(mul,"*")
mt.__div = binary_op(div,"/")

local w = wrench(v,v)
mt = getmetatable(t)
mt.__mul = binary_op(mul,"*")
mt.__div = binary_op(div,"/")

local F = Frame(Vector(1,1,1))
mt = getmetatable(F)
mt.__mul = binary_op(mul,"*")

local R = Rotation.EulerZYZ(1,1,1)
mt = getmetatable(R)
mt.__mul = binary_op(mul,"*")

local t = Twist.Zero()
mt = getmetatable(t)
mt.__div = binary_op(div,"/")
mt.__mul = binary_op(mul,"*")

local w = Wrench.Zero()
mt = getmetatable(t)
mt.__mul = binary_op(mul,"*")
mt.__div = binary_op(div,"/")


c=nil
v=nil
r=nil
f=nil
t=nil
w=nil
F=nil
R=nil
mt=nil

-- for autocompletion of static methods on types to work:
-- these methods are always checked for existence
completion_extra= {
        "Zero","Identity","EulerZYZ", "RPY","EulerZYX","Quaternion","Identity","RotX","RotY","RotZ","Rot"
}

function getRPY( arg )
    local msg=[[
        r,p,y = getRPY( R )

        INPUT:
            R: expression of a Rotation 
        OUTPUT:
            expressions for roll(r), pitch(p) and yaw (y) angles
    ]]
    if arg=='help' then
        print(msg)
        return
    end
        
    if extendedtype(arg)~="expression_rotation" then
        error("R should be an expression_rotation.\n"..msg ,2)
    end
    local v = cached(getRPY_raw(arg))
    return coord_x(v), coord_y(v), coord_z(v)
end


_conditional = conditional

function conditional(a,b,c)
    local msg = "conditional(a,b,c)\n"..
          "  returns an expression that returns the following: \n"..
          "  if a >=0 returns b, otherwise returns c \n"..
          "  \n\n";
    if a=='help' then
        print(msg)
        return
    end
    a=convert_to_expression(a)
    b=convert_to_expression(b)
    c=convert_to_expression(c)
    namedcheck({"a","b","c"},{a,b,c},{"expression_double",
    "expression_double|expression_vector|expression_rotation|expression_frame|expression_twist|expression_wrench",
    "expression_double|expression_vector|expression_rotation|expression_frame|expression_twist|expression_wrench",
    },msg);
 
    return _conditional(a,b,c)
end



--
-- define some decorators on the methods of ctx for typechecking and documentation
--

mtable_context = {}

mtable_context.getScalarExpr = function (this, name)
    local msg=[[
        getScalarExpr(this, name)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
            OUTPUT:
                expression for the given variable
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name"},{name},{"string"},msg)
    return this:_getScalarExpr(name) 
end

mtable_context.getScalarNdx = function (this, name)
    local msg=[[
        getScalarNdx(this, name)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
            OUTPUT:
                internal index (an integer) for this variable 
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name"},{name},{"string"},msg)
    return this:_getScalarNdx(name) 
end


mtable_context.createInputChannelFrame = function (this, name, defaultvalue)
    local msg=[[
        createInputChannelFrame(this, name, defaultvalue)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
                defaultvalue[optional]:  a Frame indicating the default value
            OUTPUT:
                internal index (an integer) for this variable 
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name","defaultvalue"},{name,defaultvalue},{"string","?Frame"},msg)
    if defaultvalue==nil then
        return this:_createInputChannelFrame(name) 
    else
        return this:_createInputChannelFrame(name,defaultvalue) 
    end
end


mtable_context.createInputChannelRotation = function (this, name, defaultvalue)
    local msg=[[
        createInputChannelRotation(this, name, defaultvalue)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
                defaultvalue[optional]:  a Rotation indicating the default value
            OUTPUT:
                internal index (an integer) for this variable 
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name","defaultvalue"},{name,defaultvalue},{"string","?Rotation"},msg)
    if defaultvalue==nil then
        return this:_createInputChannelRotation(name) 
    else
        return this:_createInputChannelRotation(name,defaultvalue) 
    end
end


mtable_context.createInputChannelScalar = function (this, name, defaultvalue)
    local msg=[[
        createInputChannelScalar(this, name, defaultvalue)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
                defaultvalue[optional]:  a scalar number indicating the default value
            OUTPUT:
                internal index (an integer) for this variable 
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name","defaultvalue"},{name,defaultvalue},{"string","?number"},msg)
    if defaultvalue==nil then
        return this:_createInputChannelScalar(name) 
    else
        return this:_createInputChannelScalar(name,defaultvalue) 
    end
end


mtable_context.createInputChannelVector = function (this, name, defaultvalue)
    local msg=[[
        createInputChannelVector(this, name, defaultvalue)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
                defaultvalue[optional]:  a Vector indicating the default value
            OUTPUT:
                internal index (an integer) for this variable 
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name","defaultvalue"},{name,defaultvalue},{"string","?Vector"},msg)
    if defaultvalue==nil then 
        return this:_createInputChannelVector(name) 
    else
        return this:_createInputChannelVector(name,defaultvalue) 
    end
end


mtable_context.createInputChannelTwist = function (this, name, defaultvalue)
    local msg=[[
        createInputChannelTwist(this, name, defaultvalue)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
                defaultvalue[optional]:  a Twist indicating the default value
            OUTPUT:
                internal index (an integer) for this variable 
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name","defaultvalue"},{name,defaultvalue},{"string","?Twist"},msg)
    if defaultvalue==nil then
        return this:_createInputChannelTwist(name) 
    else
        return this:_createInputChannelTwist(name, defaultvalue) 
    end
end


mtable_context.createInputChannelWrench = function (this, name, defaultvalue)
    local msg=[[
        createInputChannelWrench(this, name, defaultvalue)
        
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of a variable
                defaultvalue[optional]:  a Wrench indicating the default value
            OUTPUT:
                internal index (an integer) for this variable 
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name","defaultvalue"},{name,defaultvalue},{"string","?Wrench"},msg)
    if defaultvalue==nil then
        return this:_createInputChannelWrench(name) 
    else
        return this:_createInputChannelWrench(name,defaultvalue) 
    end
end


mtable_context.setOutputExpression = function (this, name, expr)
    local msg=[[
        setOutputExpression(this, name, expr)
       
            declares an expression for output
 
            INPUT:
                this: this method is called upon this object (no need to fill this argument
                      when using ':')
                name: name of the output expression 
                expr: expression that will be used for output (expression_double, expression_scalar,
                      expression_vector, expression_rotation, expression_frame)
            OUTPUT:
                nothing
    ]]
    if this=='help' or name=='help' then
        print(msg)
        return
    end
    namedcheck({"name","expr"},{name,expr},
       {"string","expression_double|expression_scalar|expression_vector|expression_rotation|expression_frame"},
       msg);
    this:_setOutputExpression(name,expr) 
    return
end


ctx=decorate_obj(ctx, mtable_context, [[
Object of type Context
    The context object encapsulate all information for the eTaSL task specification.
    (Contraints, Monitors, output expressions, input channels, variables)
]]);

local ftable={
    Constraint              =   Constraint,
    BoxConstraint           =   BoxConstraint,
    VelocityConstraint      =   VelocityConstraint,
    Monitor                 =   Monitor,
    Variable                =   Variable,
    deriv                   =   deriv,
    addJointsToOutputExpression  =   addJointsToOutputExpression,
    getRPY                  =   getRPY,
    conditional             =   conditional,
    objects_of_type_context =   ctx.contents,
    rospack_find            =   rospack_find,
    vector                  =   vector
}



--
-- Document UrdfExpr object constructor and instances and
-- encapsulate methods to add argument checking
--

mtable_urdf = {}
mtable_urdf.readFromFile = function (this, filename)
    local msg=[[
        readFromFile(this, filename)

            reads an urdf file r into an UrdfExpr object          
 
            INPUT:
                this:     this method is called upon this object (no need to fill this argument
                          when using ':')
                filename: name of the file
            OUTPUT:
                nothing
            REMARK:
                - can throw a LUA error if file could not be read.
                - callbacks (if used) need to be set before reading
    ]]
    if this=='help' or filename=='help' then
        print(msg)
        return
    end
    namedcheck({"filename"},{filename},
       {"string"},
       msg);
    this:_readFromFile(filename) 
    return
end


mtable_urdf.readFromString = function (this, str)
    local msg=[[
        readFromString(this, str)

            reads an urdf string into an UrdfExpr object          
 
            INPUT:
                this:     this method is called upon this object (no need to fill this argument
                          when using ':')
                str: string containing the URDF definition 
            OUTPUT:
                nothing
            REMARK:
                - can throw a LUA error if string could not be read.
                - callbacks (if used) need to be set before reading
    ]]
    if this=='help' or str=='help' then
        print(msg)
        return
    end
    namedcheck({"str"},{str},
       {"string"},
       msg);
    this:_readFromString(str) 
    return
end

mtable_urdf.readFromParam = function (this, param)
    local msg=[[
        readFromParam(this, param)

            reads an urdf specification specified by a ros parameter into an UrdfExpr object          
 
            INPUT:
                this:     this method is called upon this object (no need to fill this argument
                          when using ':')
                param: ros parameter refering to string containing the URDF definition 
            OUTPUT:
                nothing
            REMARK:
                - can throw a LUA error if string could not be read.
                - callbacks (if used) need to be set before reading
    ]]
    if this=='help' or param=='help' then
        print(msg)
        return
    end
    namedcheck({"param"},{param},
       {"string"},
       msg);
    this:_readFromParam(param) 
    return
end


mtable_urdf.addTransform = function (this, key, target_link, reference_link)
    local msg=[[
        addTransform(this, key, target_link, reference_link)

            Declares that you are interested in an expression for the frame of target_link with respect
            to reference_link.  Using getExpressions() you can get all the declared expressions at once.
            ( in this way, common parts of the expressions you request are only evaluated once)
             
            INPUT:
                this:     this method is called upon this object (no need to fill this argument
                          when using ':')
                key:      string: key to store the expression under in the resulting table (getExpressions()) 
                target_link :  string indicating the target_link (corresponding to the names in the URDF file)
                reference_link: string indicating the reference_link (corresponding to the names in the URDF file)
            OUTPUT:
                nothing
            REMARK:
                - can throw a LUA error if string could not be read.
                - should be called after reading the URDF, before calling getExpressions()
    ]]
    if this=='help' or key=='help' then
        print(msg)
        return
    end
    namedcheck({"key","target_link","reference_link"},{key, target_link, reference_link},
       {"string","string","string"},
       msg);
    this:_addTransform(key, target_link, reference_link) 
    return
end

mtable_urdf.getExpressions = function (this, context)
    local msg=[[
        getExpressions(this, context)

            Using getExpressions() you can get all the declared expressions at once.
                        
 
            INPUT:
                this:     this method is called upon this object (no need to fill this argument
                          when using ':')
                context:   the context to declare the variables, and constraints in.
            OUTPUT:
                a table containing the requested expressions (using the key given by addTransform)
            REMARK:
                - should be called after reading the URDF, and after addTransform()
                - also constructs position limit constraints, velocity limit constraints and create the
                  joint variables (as "robot" variables)
                - behaviour can be modified by setJointsAndConstraintsCallback()
    ]]
    if this=='help' or context=='help' then
        print(msg)
        return
    end
    namedcheck({"context"},{context},
       {"Context"},
       msg);
    return this:_getExpressions(context)
end

mtable_urdf.setJointsAndConstraintsCallback = function (this, cb)
    local msg=[[

        setJointsAndConstraintsCallback(this, cb)

            sets a callback function to change the behaviour of UrdfExpr w.r.t. 
            creating variables and constraints.
 
            INPUT:
                this:     this method is called upon this object (no need to fill this argument
                          when using ':')
                cb:       callback function, if you need state or pass information
                          you can use LUA closures.
            OUTPUT:
                nothing
            REMARK:
                - should be called before reading the URDF
                - the callback function is of the form:
                    function(ctx,name,vL,vU,pL,pU)
                        with ctx the context,
                        with name a name associated with the joint under consideration
                        with vL,vU the lower and upper velocity limits
                        with pL,pU the lower and upper postition limits
                     it returns an expression for the joint variable and the name of the joint 
                   The callback function is supposed to :
                        - manage creation and use of variables
                        - create constraints according to the specifications vL,vU and pL,pU

                - see also the urdf_util library.
    ]]
    if this=='help' or context=='help' then
        print(msg)
        return
    end
    namedcheck({"cb"},{ cb},
       {"function"},
       msg);
    return this:_setJointsAndConstraintsCallback(cb)
end

UrdfExpr_obj_help= [[
    Object of type UrdfExpr
        Encapsulate the reading of an URDF file and using it to 
        define constraints and variables, and to generate expressions from it.
    ]]

local _UrdfExpr=UrdfExpr
function UrdfExpr(dt, K_limits, velocity_scale)
    local msg=[[
        UrdfExpr(dt, K_limits, velocity_scale) constructor
        or UrdfExpr(dt) constructor
 
            declares an expression for output
 
            INPUT 
                K_limits : the control gain to be used for the position limits (lower and upper). 
                velocity_scale :  a scale factor on the velocity limits (lower and upper).
                dt : update step time
            OUTPUT:
               an object of type UrdfExpr
            
            REMARK:
                either both K_limits and velocity_scale are specified, or they are none of
                the arguments 
    ]]
    if K_limits=='help' then
        print(msg)
        return
    end
    namedcheck({"K_limits","velocity_scale"},{name,expr},
       {"?number","?number"},
       msg);
    if (K_limits==nil and velocity_scale~=nil) or
       (K_limits~=nil and velocity_scale==nil) then
        error(" either both K_limits and velocity_scale are specified, or none of the arguments\n"..msg,2)
    end
    local u
    if K_limits==nil and velocity_scale==nil then 
        u = _UrdfExpr(dt)
    else
        u = _UrdfExpr(dt, K_limits,velocity_scale)
    end 
    u.__tostring = function () return "UrdfExpr object" end
    decorate_obj(u, mtable_urdf, UrdfExpr_obj_help);
    return u
end

ftable['objects_of_type_UrdfExpr'] = decorate_obj(nil, mtable_urdf, UrdfExpr_obj_help).contents;
ftable['UrdfExpr']     = UrdfExpr




--
-- Document and encapsulate the motionprofile_trapezoidal objects
-- and their constructor
--

mtable_mp = {}
mp_obj_help = [[
    motionprofile_trapezoidal object

        an object to manage trapezoidal motion profiles,
        i.e. a velocity profile with a maximal velocity and maxium acceleration (+/-).
]]

mtable_mp.setProgress = function (this, varexp)
    local msg=[[
        setProgress(this, varexp)
        
            sets the progress variable for this motion profile.
            The motion profile is a function of this variable,
            and the maximum acceleration and velocity is expressed
            for this variable.
            INPUT:
                varexp:  double expression indicating the progress variable
    ]]
    if this=='help' or varexp=='help' then
        print(msg)
        return
    end
    namedcheck({"varexp"},{ varexp},
       {"expression_double"},
       msg);
    return this:_setProgress(varexp)
end

mtable_mp.addOutput = function (this, startv,endv,maxv,maxa)
    local msg=[[
        addOutput(startv,endv,maxv,maxa)
           defines an additional output for this motion profile
           INPUT:
            startv : double expression indicating the start value 
            endv : double expression indicating the end value 
            maxv : double expression indicating the maximum velocity 
            maxa : double expression indicating the maximum acceleration 
    ]]
    if this=='help' or startv=='help' then
        print(msg)
        return
    end
    namedcheck({"startv","endv","maxv","maxa"},{ startv, endv, maxv, maxa},
       {"expression_double", "expression_double", "expression_double", "expression_double"},
       msg);
    return this:_addOutput(startv, endv, maxv, maxa)
end



local _get_duration = get_duration
function get_duration( motprof,arg )
    local msg=[[
        get_duration(motprof)
        or motprof:get_duraction if called as a method

            get the planned duration for a motionprofile
            (this requires addOutput to have been called at least once)
    ]]
    if motprof=='help' or arg=='help' then
        print(msg)
        return
    end
    namedcheck({"motprof"},{motprof},{"motionprofile_trapezoidal"}, msg)
    return _get_duration( motprof)
end
ftable['get_duration'] = get_duration
mtable_mp['get_duration'] = get_duration

local _get_output_profile = get_output_profile
function get_output_profile( motprof, idx)
    local msg=[[
        get_output_profile(motprof,idx) or 
        get_output_profile(idx) when called as a method.

            gets an expression for the idx-th  output of the 
            motion profile. 
            
            0<=idx<= motprof:getNrOfOutputs()
    ]]
    if motprof=='help' or idx=='help' then
        print(msg)
        return
    end
    namedcheck({"motprof","idx"},{motprof,idx},{"motionprofile_trapezoidal","number"}, msg)
    return _get_output_profile( motprof, idx)
end
ftable['get_output_profile'] = get_output_profile
mtable_mp['get_output_profile'] = get_output_profile


mtable_mp.getEndValue = function ( motprof, idx)
    local msg=[[
        getEndValue(motprof,idx) or 
        motprof:getEndValue(idx) when called as a method.

            gets an expression for the end value for the  idx-th  
            output of the motion profile. 
            
            0<=idx<= motprof:getNrOfOutputs()
    ]]
    if motprof=='help' or idx=='help' then
        print(msg)
        return
    end
    namedcheck({"motprof","idx"},{motprof,idx},{"motionprofile_trapezoidal","number"}, msg)
    return motprof:_getEndValue(idx)
end

mtable_mp.getMaxVelocity = function ( motprof, idx)
    local msg=[[
        getMaxVelocity(motprof,idx) or 
        motprof:getMaxVelocity(idx) when called as a method.

            gets an expression for the maximum velocity for the  idx-th  
            output of the motion profile. 
            
            0<=idx<= motprof:getNrOfOutputs()
    ]]
    if motprof=='help' or idx=='help' then
        print(msg)
        return
    end
    namedcheck({"motprof","idx"},{motprof,idx},{"motionprofile_trapezoidal","number"}, msg)
    return motprof:_getMaxVelocity(idx)
end

mtable_mp.getMaxAcceleration = function ( motprof, idx)
    local msg=[[
        getMaxAcceleration(motprof,idx) or 
        motprof:getMaxAcceleration(idx) when called as a method.

            gets an expression for the maximum acceleration for the  idx-th  
            output of the motion profile. 
            
            0<=idx<= motprof:getNrOfOutputs()
    ]]
    if motprof=='help' or idx=='help' then
        print(msg)
        return
    end
    namedcheck({"motprof","idx"},{motprof,idx},{"motionprofile_trapezoidal","number"}, msg)
    return motprof:_getMaxAcceleration(idx)
end

mtable_mp.getStartValue = function ( motprof, idx)
    local msg=[[
        getStartValue(motprof,idx) or 
        motprof:getStartValue(idx) when called as a method.

            gets an expression for the starting value for the  idx-th  
            output of the motion profile. 
            
            0<=idx<= motprof:getNrOfOutputs()
    ]]
    if motprof=='help' or idx=='help' then
        print(msg)
        return
    end
    namedcheck({"motprof","idx"},{motprof,idx},{"motionprofile_trapezoidal","number"}, msg)
    return motprof:_getStartValue(idx)
end



local _create_motionprofile_trapezoidal = create_motionprofile_trapezoidal
function create_motionprofile_trapezoidal(arg)
    local msg=[[
        create_motionprofile_trapezoidal()

        a constructor that creates an object to manage trapezoidal motion profiles,
        i.e. a velocity profile with a maximal velocity and maxium acceleration (+/-).


        EXAMPLE:
                require("context")

                mp = create_motionprofile_trapezoidal()
                mp:setProgress(time)

                maxvel = constant(0.5)
                maxacc = constant(0.5)

                mp:addOutput(constant(0), constant(1.0), maxvel, maxacc)
                mp:addOutput(constant(-1), constant(2.0), maxvel, maxacc)

                duration = get_duration(mp)

                x = get_output_profile(mp,0)
                y = get_output_profile(mp,1)

        REMARK:
                the outputs are numbered starting from 0
    ]]
    if arg=='help' then
        print(msg)
        return
    end
    local mp = _create_motionprofile_trapezoidal()

    mp.__tostring = function () 
        local str= "motionprofile {\n"  
        str = str .. "    number of outputs : " .. mp:nrOfOutputs() .. "\n"
        for i=0,mp:nrOfOutputs()-1 do
            
            str = str .. "        start: " .. mp:getStartValue(i):value() .. ",  end: " .. mp:getEndValue(i):value()
                  .. ", max. vel.: " .. mp:getMaxVelocity(i):value() .. ", max. acc. : " .. mp:getMaxAcceleration(i):value() .."\n"
        end
        str = str .."}\n"
        return str
    end
    decorate_obj(mp, mtable_mp, mp_obj_help);
    return mp
end

ftable['objects_of_type_motionprofile_trapezoidal'] = 
    decorate_obj(nil, mtable_mp, mp_obj_help).contents
ftable['create_motionprofile_trapezoidal']     = create_motionprofile_trapezoidal 









-- this should be last:
ftable['contents'] = _contents(ftable,"context")
ftable['help']     = _help(ftable,"context")


return ftable



