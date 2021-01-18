require("context")
require("libexpressiongraph_velocities")

-- to provide automatically counting names:
function counter(name)
    if counter_map[name]==nil then
        counter_map[name]=0
    end
    counter_map[name] = counter_map[name] + 1
end



-- routine needed:
--    project_derivative_on_direction( prev_dir, expr)
--        value : constant(0)
--        derivative and derivative expr: dot(prev_dir, expr)



function normalized_velocity( time, arg, tol)
    local msg = 
    [[ 
    normalized_velocity(time, arg, tol)

    returns a normal vector in the direction of the velocity, if the velocity is close to zero
    (or at the first time the expression is evaluated), this return will return a zero vector.
    tol is an optional parameter indicating how close to zero the velocity should be (default:1E-6).
    ]] 
    namedcheck({"time"},{time},{"expression_double"}, msg)
    namedcheck({"arg"},{arg},{"expression_vector|expression_rotation"}, msg)
    if tol==nil then
        tol = 1E-6
    end
    local vel    = previous_velocity( time, arg )
    local nvel   = cached(norm(vel))
    local retval = conditional( nvel-constant(tol),  vel/nvel,  constant(Vector(0,0,0)) )
    return retval
end

function VelocityLimits(arg)
    local msg = "\nLimits the MAGNITUDE of the velocity of a given expression. \nThis function is called as follows:\n" ..
                "VelocityLimits{\n"..
                "   context         = ... \n" ..
                "   name            = ...    [optional, default_name<nr>]\n" ..
                "   expr            = ... (! scalar position-level expression !)\n" ..
                "   limit           = ...       [optional, 0.0]\n" ..
                "   weight          = ...       [optional, defaultweight] (scalar)\n" ..
                "   priority        = ...     [optional, defaultpriority]\n"..
                "}\n ";
    if arg==nil then
        error( "VelocityLimits expects a table as argument:"..msg)
    end
    namedcheck({"context","name","expr","target_velocity","weight","priority"},
               {arg.context,arg.name,arg.expr, arg.target_velocity, arg.weight, arg.priority},
               {"Context","?string","expression_double", "?number","?number|?expression_double","?number"},msg)


    namedcheck({"context"},{arg.context},{"Context"}, msg)


    namedcheck({"name"},{arg.name},{"?string"}, msg)
    if arg.name==nil then
        arg.name = "velocitylimits_" .. tostring(counter("velocitylimits"))
    end 


    namedcheck({"weight"},{arg.weight},{"?number|?expression_double"}, msg)
    if arg.weight==nil then
        arg.weight = 1.0
    end
    if extendedtype(arg.weight) ~= "expression_double" then
       arg.weight = constant( arg.weight ) 
    end


    namedcheck({"priority"},{arg.priority},{"?number"}, msg)
    if arg.priority==nil then
        arg.priority = 2 
    end
    if extendedtype(arg.weight) ~= "expression_double" then
       arg.weight = constant( arg.weight ) 
    end

    namedcheck({"limit"},{arg.limit},{"number"}, msg)
      
    namedcheck({"expr"},{arg.expr},{"expression_double|expression_vector|expression_rotation"}, msg)
   
    local t = extendedtype(arg.expr)
    if t=="expression_double" then
        if arg.expr:isScalarVariable() >=0 then
            -- box constraints:
            svarname = arg.ctx:getScalarName( arg.expr:isScalarVariable() )
            BoxConstraint{
                context=arg.ctx,
                var_name = svarname,
                lower = -arg.limit,
                upper =  arg.limit
            }
        else
            -- two separate constraints.
            Constraint{
                context      = arg.context,
                name         = arg.name.."_upper",
                expr         = arg.expr - constant(arg.limit)*time,
                target_upper = 0,
                weight       = arg.weight,
                priority     = arg.priority,
                K            = 0.0
            }
            Constraint{
                context      = arg.context,
                name         = arg.name.."_lower",
                expr         = arg.expr + constant(arg.limit)*time,
                target_lower = 0,
                weight       = arg.weight,
                priority     = arg.priority,
                K            = 0.0
            }
        end
    elseif t=="expression_vector" then
        local prev_dir = normalized_velocity(time, arg.expr)
        Constraint{
                context      = arg.context,
                name         = arg.name,
                expr         = dot(prev_dir, arg.expr) - constant(arg.limit)*time,
                target_upper = 0,
                weight       = arg.weight,
                priority     = arg.priority,
                K            = 0.0
            }
    elseif t=="expression_rotation" then
    end

end


