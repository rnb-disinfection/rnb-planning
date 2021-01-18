require("context")
require("geometric")
require("libexpressiongraph_spline")

-- loading a model for the unversal robot UR10:
u=UrdfExpr();
res=u:readFromFile(rospack_find("ur_description").."/urdf/ur10_robot.urdf")
if not res then
   print("could not read robot file")
end
u:addTransform("ee","ee_link","base_link")


r = u:getExpressions(ctx)
tool = r.ee * rotate_y(math.pi/2)*translate_z(0.1);

-- only for the simulatio:
--ctx:setInitialScalarValues({ shoulder_lift_joint=math.pi/2});

print("frames in the URDF file:")
print(u)


-- trajectory to follow:
maxvel     = constant(0.15)
maxacc     = constant(0.5)

spl        = CubicSpline(1)
spl:readPoints(rospack_find("expressiongraph_spline").."/scripts/circle.csv"," \t,",1)
spln       = spl:getNormalizer(spl:getNrOfRows()*30)
spln:setInput(time)
spl:setInput(getSplineOutput(spln,0))
pathlength = spln:getMaxArgument()

mp         = create_motionprofile_trapezoidal()
mp:addOutput(constant(0),constant(pathlength*4), maxvel,maxacc)
duration   = get_duration(mp)
spln:setInput(get_output_profile(mp,0))
y1=getSplineOutput(spl,0)
y2=getSplineOutput(spl,1)

ox=constant(0.5)
oy=constant(0.5)
trajectory = vector(y1+ox,y2+oy,constant(0.4))
vdiff = trajectory - origin(tool)
write_expressions_to_dot('lua_scripts2.dot',{vdiff})

target_orientation = constant(Rotation.EulerZYX(0.0,math.pi,0.0));

Constraint{
           context=ctx,
           name="spline_trajectory",
           expr=vdiff
};
Constraint{
        context=ctx,
        name="perpendicular_to_plane",
        expr = inv(target_orientation)*rotation(tool)
};
Constraint{
        context=ctx,
        name = "elbow up config",
        target_lower = 0.0,
        expr = ctx:getScalarExpr("elbow_joint")
};
Monitor{
        context=ctx,
        name='finish_after_some_time',
        upper=10,
        actionname='exit',
        expr=time-duration
}


function outputFrame(ctx,name,F)
    local msg="outputFrame(context, name, expression_frame)\n"..
              "   outputs a frame to Ros for visualization\n\n"
    namedcheck({"ctx", "name","F" },
               {ctx, name, F},
               {"Context","string","expression_frame"},
               msg)
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


ctx:addOutput("time", "csv",time)
ctx:addOutput("x", "csv", y1+ox)
ctx:addOutput("y", "csv", y2+ox)
ctx:addOutput("tool","csv",origin(tool))
outputFrame(ctx,"tool",tool)
ctx:addOutput("ee_marker","ros_1",origin(tool));
addJointsToOutput(u,ctx,"csv")

print(ctx)
