
require("expressiongraph")

-- static methods are accessed using "."
-- normal methods are accessed using ":"

-- kdl frame and its operations :

F = Frame(Rotation.RotX(30.0/180.0*pi),Vector(1,2,3))
print("construction of a frame : ")
print(F)
print("construction of a identity frame : ")
print(Frame.Identity())

print("inverse of a frame transformation ")
print( F:Inverse() )

print(" composition of frame transformation ")
F2 = Frame(Rotation.RotX(30.0/180.0*pi),Vector(0,0,1))
F3 = F * F2
print( F3)

v = Vector(3,4,5);
print( " transformation of a vector (rotational+translational) ")
print(F*v)


