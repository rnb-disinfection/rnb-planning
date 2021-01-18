-- standalone example to demonstrate expressiongraph_collision
-- This script computes the distance between a series of convex
-- objects.

require("context")
require("geometric")
--require("libexpressiongraph_collision")
require("collision")


function display_distance(f1,s1,radius1, margin1, f2, s2, radius2,margin2)
    local d=distance_between(f1,s1,radius1,margin1,f2,s2,radius2,margin2)
    print ("--------------------------------------------------")
    print ("location 1 : ")
    print(f1)
    print ("shape    1 : ")
    print(s1)
    print ("radius   1 : "..radius1)
    print ("margin   1 : "..margin1)
    print ("location 2 : ")
    print(f2)
    print ("shape    2 : ")
    print(s2)
    print ("radius   2 : "..radius2)
    print ("radius   2 : "..margin2)
    print ("distance " .. d:value())
    return d
end




f1=translate_x(0.0)
f2=translate_x(4.0)

-- create a box with length in x==0.16, length in y==0.16, length in z == 0.16
s1=Box(0.16,0.16,0.16) 

s2=MultiSphere({Vector(0,0,0)},{0.08})

-- create a capsule with axis along x, radius=1, length =1
s3=CapsuleX(1,2)
-- create a capsule with axis along y, radius=1, length =1
s4=CapsuleY(1,2)
-- create a capsule with axis along z, radius=1, length =2
s5=CapsuleZ(1,2)

-- creates a cylinder with axis along x, sizex=1, sizey=1.1, sizez=1.2 
s6=CylinderX(1,1,1)
-- creates a cylinder with axis along y, sizex=1, sizey=1.1, sizez=1.2 
s7=CylinderY(1,1,1)
-- creates a cylinder with axis along z, sizex=1, sizey=1.1, sizez=1.2 
s8=CylinderZ(1,1,1)

-- creates a cone with axis along x, radius =1, length = 2
s9=ConeX(1,1)
-- creates a cone with axis along y, radius =1, length = 2
s10=ConeY(1,1)
-- creates a cone with axis along z, radius =1, length = 2
s11=ConeZ(1,1)
-- creates a convex shape as defined by a Wavefront .obj file:
name=rospack_find("expressiongraph_collision").."/data/cube.obj"
s12=ConvexObject(name)

s13=MultiSphere({Vector(1,1,0),Vector(-1,1,0),Vector(1,-1,0),Vector(-1,-1,0)},{1,1,1,1})


-- distance between two shapes s1 and s2 at respectively location f1 and location f2
-- for each shape, and additional radius is given an a margin for the computations
print("distance 4");
margin=0.0001
--margin=0.0
print("margin "..margin)
radius=0.0
print("radius "..radius)

display_distance(f1,s1,radius,margin,f2,s2,radius,margin)
display_distance(f1,s1,radius,margin,f2,s3,radius,margin)
display_distance(f1,s1,radius,margin,f2,s4,radius,margin)
display_distance(f1,s1,radius,margin,f2,s5,radius,margin)
display_distance(f1,s1,radius,margin,f2,s6,radius,margin)
display_distance(f1,s1,radius,margin,f2,s7,radius,margin)
display_distance(f1,s1,radius,margin,f2,s8,radius,margin)
display_distance(f1,s1,radius,margin,f2,s9,radius,margin)
display_distance(f1,s1,radius,margin,f2,s10,radius,margin)
display_distance(f1,s1,radius,margin,f2,s11,radius,margin)
display_distance(f1,s1,radius,margin,f2,s12,radius,margin)
display_distance(f1,s1,radius,margin,f2,s13,radius,margin)
display_distance(f1,s12,radius,margin,f2,s13,radius,margin)




-- does it work in degenerate cases ?
--

margin=0.0001
radius=0.0
square = MultiSphere({Vector(1,1,0),Vector(-1,1,0), Vector(-1,-1,0), Vector(1,-1,0)},{0,0,0,0})

point  = MultiSphere({Vector(0,0,0)},{0})
f_square = translate_x(0.0)
f_point  = translate_x(0.2)*translate_y(0.2)
print(f_point:value())
display_distance(f_square,square,radius,margin,f_point,point,radius,margin)
d= distance_between(f_square,square,radius,margin,f_point,point,radius,margin)
