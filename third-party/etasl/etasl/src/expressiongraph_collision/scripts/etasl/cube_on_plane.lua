require("context")
require("geometric")
require("libexpressiongraph_collision")

if not robot_ee then
    error("a robot should be loaded before running this script",2)
end

obj_bottom = robot_ee*translate_z(0.16);



---------
ctx:pushGroup("moving_down")
p  = origin(obj_bottom*translate_z(0.1))
Constraint {
    context         = ctx,
    name            = "moving_down",
    expr            =  p  -( constant(Vector(0.00,0 ,-0.05))*time),
    weight          = 1,
    priority        = 2,
    K               = 2
}
Constraint {
    context         = ctx,
    name            = "moving_down_rot",
    expr            = rotation(robot_ee)*constant(Rotation.EulerZYX(0,math.pi,0)),
    weight          = 1,
    priority        = 2,
    K               = 4
}
Monitor{
        context=ctx,
        name='finished',
        upper=8,
        actionname='exit',
        expr=time
}
ctx:popGroup()

-------
ctx:pushGroup("moving")
p2  = origin(obj_bottom*translate_z(0.0))
Constraint {
    context         = ctx,
    name            = "moving",
    expr            = p2 -( constant(Vector(0.0,0.0 ,-0.05))*time),
    weight          = 1,
    priority        = 2,
    K               = 0
}
Constraint {
    context         = ctx,
    name            = "rotating",
    expr            = ctx:getScalarExpr("lbr_iiwa_joint_1") - constant(5/180*math.pi)*time;
    weight          = 1,
    priority        = 2,
    K               = 0
}

Monitor{
        context=ctx,
        name='finish_moving',
        upper=110,
        actionname='exit',
        expr=time
}

--[[
Constraint {
    context         = ctx,
    name            = "moving_down_rot",
    expr            = rotation(robot_ee)*constant(Rotation.EulerZYX(0,math.pi,0)),
    weight          = 1,
    priority        = 2,
    K               = 4
}

--]]
ctx:popGroup()

-------
ctx:pushGroup("stretching")
Constraint {
    context         = ctx,
    name            = "rotating",
    expr            = ctx:getScalarExpr("lbr_iiwa_joint_4") - constant(5/180*math.pi)*time;
    weight          = 1,
    priority        = 2,
    K               = 0
}

ctx:popGroup()


-------
ctx:setOutputExpression("z",coord_z(origin(obj_bottom)))
ctx:setOutputExpression("z_initial",initial_value(time,coord_z(origin(obj_bottom))))


