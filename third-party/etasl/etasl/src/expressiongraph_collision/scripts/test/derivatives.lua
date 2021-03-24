-- run this script using "lua derivatives.lua" in order
-- to test out the automatic differentiation in combination
-- with the distance between convex objects
-- 

require("context")
require("geometric")
require("libexpressiongraph_collision")

--plane = Halfspace(0,0,1,0)
plane = Box(10,10,1)
plane_pose = translate_x(0)

name = rospack_find("expressiongraph_collision").."/meshes/superquadric.obj"
--obj = ConvexObject(name)
obj = Box(1,1,1)

obj_pose = translate_x(input(1))*translate_z(input(2))*rotate_y(input(3))
d=distance_between(plane_pose,plane,0.00, 0.0,obj_pose, obj, 0.0, 0.0)
--d=distance_between(obj_pose,obj,0.00, 0.0,plane_pose, plane, 0.0, 0.0)

print("----------------------------------------------------------")
x=0
z=5
r=0
print("x " .. x)
print("z " .. z)
print("r " .. r)

d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv=d:value()
print("distance : "..dv)

dt = 1E-2
d:setInputValue(1,x+dt)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv1=d:value()
der1 = (dv1-dv)/dt
print("num. der towards 1 "..der1)
print("autodiff towards 1 "..d:derivative(1)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z+dt)
d:setInputValue(3,r)
dv2=d:value()
der2 = (dv2-dv)/dt
print("num. der towards 2 "..der2)
print("autodiff towards 2 "..d:derivative(2)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r+dt)
dv3=d:value()
der3 = (dv3-dv)/dt
print("num. der towards 3 "..der3)
print("autodiff towards 3 "..d:derivative(3)..'\n')

print("----------------------------------------------------------")

x=0
z=5
r=30/180*math.pi
print("x " .. x)
print("z " .. z)
print("r " .. r)



d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv=d:value()
print("distance : "..dv)

d:setInputValue(1,x+dt)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv1=d:value()
der1 = (dv1-dv)/dt
print("num. der towards 1 "..der1)
print("autodiff towards 1 "..d:derivative(1)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z+dt)
d:setInputValue(3,r)
dv2=d:value()
der2 = (dv2-dv)/dt
print("num. der towards 2 "..der2)
print("autodiff towards 2 "..d:derivative(2)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r+dt)
dv3=d:value()
der3 = (dv3-dv)/dt
print("num. der towards 3 "..der3)
print("autodiff towards 3 "..d:derivative(3)..'\n')

print("----------------------------------------------------------")

x=0
z=5 - 3.75
r=30/180*math.pi
print("x " .. x)
print("z " .. z)
print("r " .. r)



d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv=d:value()
print("distance : "..dv)

d:setInputValue(1,x+dt)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv1=d:value()
der1 = (dv1-dv)/dt
print("num. der towards 1 "..der1)
print("autodiff towards 1 "..d:derivative(1)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z+dt)
d:setInputValue(3,r)
dv2=d:value()
der2 = (dv2-dv)/dt
print("num. der towards 2 "..der2)
print("autodiff towards 2 "..d:derivative(2)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r+dt)
dv3=d:value()
der3 = (dv3-dv)/dt
print("num. der towards 3 "..der3)
print("autodiff towards 3 "..d:derivative(3)..'\n')

print("----------------------------------------------------------")

x=0
z=5-3.78
r=30/180*math.pi
print("x " .. x)
print("z " .. z)
print("r " .. r)



d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv=d:value()
print("distance : "..dv)

d:setInputValue(1,x+dt)
d:setInputValue(2,z)
d:setInputValue(3,r)
dv1=d:value()
der1 = (dv1-dv)/dt
print("num. der towards 1 "..der1)
print("autodiff towards 1 "..d:derivative(1)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z+dt)
d:setInputValue(3,r)
dv2=d:value()
der2 = (dv2-dv)/dt
print("num. der towards 2 "..der2)
print("autodiff towards 2 "..d:derivative(2)..'\n')


d:setInputValue(1,x)
d:setInputValue(2,z)
d:setInputValue(3,r+dt)
dv3=d:value()
der3 = (dv3-dv)/dt
print("num. der towards 3 "..der3)
print("autodiff towards 3 "..d:derivative(3)..'\n')

