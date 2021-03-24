-- A simple eTaSL task
-- requires a robot specification to be read before loading this script.

-- Input:
tgt_x = ctx:createInputChannelScalar("tgt_x",0.7)
tgt_y = ctx:createInputChannelScalar("tgt_y",0)
tgt_z = ctx:createInputChannelScalar("tgt_z",0.7)


robot_ee = robot_mp

-- Some additional constraints on the velocity of the robot joints:
maxvel=0.30
for i=1,#robot_jname do
    BoxConstraint{
        context = ctx,
        var_name = robot_jname[i],
        lower = -maxvel,
        upper = maxvel
    }
end

-- The end effector origin follows the input signals:
Constraint{
    context=ctx,
    name="x",
    expr = tgt_x - coord_x(origin(robot_mp)),
    priority = 2,
    K        = 4
}

Constraint{
    context=ctx,
    name="y",
    expr = tgt_y - coord_y(origin(robot_mp)),
    priority = 2,
    K        = 4
}

Constraint{
    context=ctx,
    name="z",
    expr = tgt_z - coord_z(origin(robot_mp)),
    priority = 2,
    K        = 4
}

-- Somewhat artificial introduction of a feature variable
-- (to verify that feature variables are correctly handled)
f1=Variable{context=ctx, name="f1", vartype="feature"}

Constraint{
    context=ctx,
    name="f",
    expr = f1 - coord_y(origin(robot_ee)),
    priority = 2,
    K        = 1,
    weight   = 0.01
}


-- Output
ctx:setOutputExpression("ee_x",coord_x(origin(robot_ee)))
ctx:setOutputExpression("ee_y",coord_y(origin(robot_ee)))
ctx:setOutputExpression("ee_z",coord_z(origin(robot_ee)))
ctx:setOutputExpression("f1",f1)