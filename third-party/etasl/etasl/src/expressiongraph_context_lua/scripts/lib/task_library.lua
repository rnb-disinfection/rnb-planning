require("context")
require("geometric")

if ilua~=nil then
    print( [[ 
task_library called by ilua, printing most important functions :
    DesiredJointPositions(), flexible_progress_parameter() ,
    LookAtConstraint(), CollisionConstraint, 

]])
end

-- THIS VERSION IS OBSOLETE, USE DesiredJointPositions
-- defines constraints that apply a soft constraint to each of the joints
-- with a given control constant K and weight.
-- the joints are specified by a table namedvalues that contains the
-- joint names and thier value, e.g.:
--      nv.l_wrist_roll_joints = 0.2
--      nv.l_wrist_flex_joints = 0.3
function desired_joint_positions(ctx,name, namedvalues, K, weight)
    check( {ctx,name,namedvalues,K,weight}, {"Context", "string","namedvalues","number","number"})
    for k,v in pairs(namedvalues) do
        local e = ctx:getScalarExpr(k)
        if e==nil then
            error("unknown joint value in list",2)
        end
        Constraint{
            context=ctx,
            name   = name .. ":" .. k,
            expr   = e - constant(v),
            K      = K,
            weight = weight,
            priority = 2
        }
    end
end

function DesiredJointPositions(arg)
    local msg = "define a number of joint constraints with common properties\n"..
                "DesiredJointPositions{\n"..
                "   context =  [obligatory] the context to add the constraints to\n"..
                "   name    =  [obligatory] prefix for the name of the added constraints\n"..
                "   list    =  [obligatory] lua table containing the joints and their desired values\n"..
                "   K       =  [optional, default K=4] control constant for the added constraints\n"..
                "   weight  =  [optional, default weight=1] weight for the added constraints\n"..
                "   strict  =  [optional, default=true] whether or not to ignore non-existing joints\n"..
                "}\n\n"
   namedcheck({"context","name","list","K","weight","strict"},
              {arg.context, arg.name, arg.list, arg.K, arg.weight,arg.strict},
              {"Context","string","namedvalues","?number","?number","?boolean"},
              msg)
   if arg.K==nil then
        arg.K=4
   end
   if arg.weight==nil then
        arg.weight=1
   end
   if arg.strict==nil then
        arg.strict = true
   end
   for k,v in pairs(arg.list) do
        local e = ctx:getScalarExpr(k)
        if e==nil then
            if arg.strict then
                error("unknown joint value in list",2)
            end
        else 
            Constraint{
                context= arg.context,
                name   = arg.name .. ":" .. k,
                expr   = e - constant(v),
                K      = arg.K,
                weight = arg.weight,
                priority = 2
            }
        end
    end
end

function flexible_progress_parameter(ctx, name, nominal_velocity)
    if ctx==nil and name==nil and nominal_velocity==nil then
        error("should be called as flexible_progress_parameter(context, name, nominal_velocity) ");
    end
    check( {ctx,name,nominal_velocity},{"Context","string","expression_double|number"} )
    if extendedtype(nominal_velocity)=="number" then
        nominal_velocity = constant( nominal_velocity )
    end
    p = ctx:addScalarVariable(name..":progress","feature",0.0, 1)
    Constraint{
        context = ctx,
        name    = name,
        expr    = p - nominal_velocity*time,
        K       = 0.0,
        weight  = 0.1,
        priority= 2
    }
    return p
end



lookat_counter = 1

-- implements a look_at constraint.
-- also demonstrates more intelligent handling of its arguments.
function LookAtConstraint(arg)
    local msg = "This function is called as follows:\n"..
                "LookAtConstraint{\n" ..
                "   context = ...\n" ..
                "   name    = ... [optional]\n" ..
                "   camera  = ...\n" ..
                "   object  = ...\n" ..
                "}\n";
    if arg==nil or arg.context==nil or arg.camera==nil or arg.object==0 then
        error("LookAtConstraint is missing arguments\n"..msg)  
    end
    if arg.name==nil then
        arg.name = "lookat"+tostring(lookat_counter)
        lookat_counter = lookat_counter + 1
    end
    namedcheck({"context","name","camera","object"},
                {arg.context,arg.name,arg.camera,arg.object},
                {"Context","string", "expression_frame","expression_vector|expression_frame"},
               msg) 
    if extendedtype(arg.object) == "expression_frame" then
        arg.object = cached( origin( arg.object ) )
    end
    distance = ctx:addScalarVariable(arg.name..":distance","feature",0.1, 1)
    Constraint{
        context=ctx,
        name   = arg.name,
        expr   = arg.object - arg.camera*vector(constant(0),constant(0), distance),
        K      = 2,
        weight = 0.1,
        priority = 2
    }
end


-- A record that describes a segment :
-- {startid=1,startp= expression_frame, endid=2, endp = expressionframe, radius = ...}
function CollisionConstraint( arg )
    -- type checking :
    local msg = "CollisionConstraint{\n"..
                "     context=..., [context to be used]\n"..
                "     name   =..., [optional, default ''][prefix to the names of the collision constraints]\n"..
                "     list   =..., [list of link specifications]\n".. 
                "     K      =..., [optional, default 10][control constant for constraints]\n"..
                "     weight =..., [optional, default 1.0][weight for the constraints (only if priority=2)]\n"..
                "     priority=...,[optional, default 1][priority for the collision constraints]\n"..
                "     output  =...,[optional, default ''][if not empty, output for of the constraints to this type] \n"..
                "}\n"..
                "list is a list of the following type:\n"..
                "{ {startid=1,startp= expression_frame, endid=2, endp = expression_frame, radius = ...}, {startid=...,...}, ... }\n\n";
    if arg==nil then
        error("no arguments given\n\nis called as follows:\n"..msg)
    end
    namedcheck( {"context", "name", "list", "K", "weight","output","weight","priority"},
                {arg.context, arg.name, arg.list, arg.K,   arg.output, arg.weight, arg.priority},
                {"Context", "?string", "table", "?number", "?string",  "?number",  "?number"  },msg);
    if arg.priority==nil then
        arg.priority=2
    end
    if arg.priority~=2 and arg.weight~=nil then
        error("It does not make sense to specify weight for a priority not equal to 2\n\n"..msg)
    end
    if arg.K==nil then
        arg.K = 10;
    end
    if arg.name==nil then
        arg.name = "";
    end
    if #arg.name~= 0 then
        arg.name = arg.name .. ":"
    end
    if arg.output==nil then
        arg.output=""
    end
    if arg.weight==nil then
        arg.weight=1
    end
    local n = #arg.list
    for i=1,n do
        namedcheck( {"startid","startp","endid","endp","radius"},
                    {arg.list[i].startid, arg.list[i].startp, arg.list[i].endid, arg.list[i].endp, arg.list[i].radius},
                    {"number","expression_vector|expression_frame","number","expression_vector|expression_frame","?number"},
                    msg)
        if arg.list[i].radius==nil then
            arg.list[i].radius=0;
        end
        if extendedtype(arg.list[i].startp) == "expression_frame" then
            arg.list[i].startp = origin( arg.list[i].startp )
        end
        if extendedtype(arg.list[i].endp) == "expression_frame" then
            arg.list[i].endp = origin( arg.list[i].endp )
        end
    end
    -- generating constraints :
    for i=1,n do
        for j=1,n do
            if arg.list[i].startid ~= arg.list[j].startid and
               arg.list[i].startid ~= arg.list[j].endid   and
               arg.list[i].endid   ~= arg.list[j].startid and
               arg.list[i].endid   ~= arg.list[j].endid   then 
                    local d =  distance_segment_segment( arg.list[i].startp, arg.list[i].endp, arg.list[j].startp, arg.list[j].endp )
                               - constant( arg.list[i].radius + arg.list[j].radius)
                    local cname    = arg.name.."collision-"..tostring(i).."-"..tostring(j) 
                    Constraint{
                        context = arg.context,
                        name    = cname, 
                        expr    = d, 
                        target_lower = 0,
                        K       = arg.K,
                        priority = 1
                    }
                    if arg.output~="" then
                        arg.context:addOutput(cname, arg.output,d)
                    end
            end
        end
    end
end


