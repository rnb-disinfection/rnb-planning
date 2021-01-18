require("context")
require("geometric")
require("libexpressiongraph_collision")

if not robot_ee then
    error("a robot should be loaded before running this script",2)
end

obj_bottom = robot_ee*translate_z(0.16);



---------
ctx:pushGroup("moving_left")
p  = origin(obj_bottom*translate_z(0.1))
Constraint {
    context         = ctx,
    name            = "moving_left",
    expr            =  p - initial_value(time,p) -( constant(Vector(0.0,0.05 ,-0.04))*time),
    weight          = 1,
    priority        = 2,
    K               = 2
}

Constraint {
    context         = ctx,
    name            = "moving_left_rot",
    expr            = rotation(robot_ee)*constant(Rotation.EulerZYX(0,math.pi,0)),
    weight          = 1,
    priority        = 2,
    K               = 4
}
Monitor{
        context=ctx,
        name='finished',
        upper=7,
        actionname='exit',
        expr=time
}
ctx:popGroup()

-------
ctx:pushGroup("moving_back")
p2  = origin(obj_bottom*translate_z(0.1))
Constraint {
    context         = ctx,
    name            = "moving_down",
    expr            = p2 -initial_value(time,p2)-( constant(Vector(-0.05,0.0 ,0.0))*time),
    weight          = 1,
    priority        = 2,
    K               = 4
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
        name='finish_after_some_time',
        upper=10,
        actionname='exit',
        expr=time
}
ctx:popGroup()

-------
ctx:setOutputExpression("z",coord_z(origin(obj_bottom)))
ctx:setOutputExpression("z_initial",initial_value(time,coord_z(origin(obj_bottom))))


