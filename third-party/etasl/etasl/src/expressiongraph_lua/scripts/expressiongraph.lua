-- This file is execute before every configuration
require("libexpressiongraph_lua")
--
-- define some of the operations that could not be defined using luabind:
--
pi = 3.14159265359
c=constant(1.0);
mt = getmetatable(c)
mt.__add = add
mt.__sub = sub
mt.__div = div
mt.__mul = mul
mt.__unm = unm
v = vector(c,c,c)

mt = getmetatable(v)
mt.__add = add
mt.__sub = sub
mt.__unm = unm
r = rot_x(c);

mt = getmetatable(r)
mt.__mul = mul 
f = frame(v)

mt = getmetatable(f)
mt.__mul = mul

t = twist(v,v)
mt = getmetatable(t)
mt.__mul = mul 
mt.__div = div 

w = wrench(v,v)
mt = getmetatable(t)
mt.__mul = mul 
mt.__div = div 

F = Frame(Vector(1,1,1))
mt = getmetatable(F)
mt.__mul = mul

R = Rotation.EulerZYZ(1,1,1)
mt = getmetatable(R)
mt.__mul = mul

t = Twist.Zero()
mt = getmetatable(t)
mt.__div = div
mt.__mul = mul 

w = Wrench.Zero()
mt = getmetatable(t)
mt.__mul = mul 
mt.__div = div 



