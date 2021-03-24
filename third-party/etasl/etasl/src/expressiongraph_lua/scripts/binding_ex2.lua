 
    require("expressiongraph")


    -- kdl rotation and its operations :

    --   ways of creating a Rotation:

    R=Rotation(Vector(1,0,0),Vector(0,0,-1),Vector(0,1,0))
    print("Rotation(...) : ")
    print(R)
    -- first around Z, then around the new Y, then around the new X :
    R=Rotation.EulerZYX(0,20.0/180.0*pi,0)
    print("Rotation.EulerZYX(...) : ")
    print(R)
    
    -- roll pitch yaw angles:
    R=Rotation.RPY(0,20.0/180.0*pi,0)
    print("Rotation.RPY(...) : ")
    print(R)
    
    -- first around Z, then around the new Y, then around the new Z :
    R=Rotation.EulerZYZ(0,20.0/180.0*pi,0)
    print("Rotation.EulerZYZ(...) : ")
    print(R)
    
    -- Quaternions:
    R=Rotation.Quaternion(0.5,0.5,0.5,0.5)
    print("Rotation.Quaternion : ")
    print(R)

    -- Identity :
    R=Rotation.Identity()
    print("Rotation.Identity : ")
    print(R)

    -- Rotation around X
    R=Rotation.RotX(30/180.0*pi)
    print("Rotation.RotX : ")
    print(R)
    
    -- Rotation around Y
    R=Rotation.RotY(30/180.0*pi)
    print("Rotation.RotY : ")
    print(R)
    
    -- Rotation around Z
    R=Rotation.RotZ(30/180.0*pi)
    print("Rotation.RotZ : ")
    print(R)

    -- Axis/angle rotation:
    -- It is not necessary to normalize the rotation axis :
    print("Rotation.Rot : ")
    R=Rotation.Rot(Vector(1,1,1),20.0/180.0*pi)
    print(R)
    


    R=Rotation.EulerZYX(10.0/180.0*pi, 20.0/180.0*pi, 40.0/180.0*pi) 
    print("Rotation matrix for EulerZYX(10,20,40): ")
    print(R)
    
    -- getting EulerZYX representation
    alpha,beta,gamma = R:getEulerZYX()
    print("EulerZYX : alpha ",alpha/pi*180.0,"     beta ",beta/pi*180.0,"     gamma ",gamma/pi*180.0)
    
    -- getting EulerZYX representation
    alpha,beta,gamma = R:getEulerZYZ()
    print("EulerZYZ : alpha ",alpha/pi*180.0,"     beta ",beta/pi*180.0,"     gamma ",gamma/pi*180.0)
    
    -- getting RPY representation
    r,p,y = R:getRPY() 
    print("RPY : r ",r/pi*180.0, "   p ",p/pi*180.0,"   y ",y/pi*180.0)
    
    -- getting Quaternion representation
    q1,q2,q3,q4 = R:getQuaternion()
    print("Quaternion ",q1, q2, q3, q4)

    print("X-axis, Y-axis and Z-axis of a rotation matrix ")
    print(R:UnitX())
    print(R:UnitY())
    print(R:UnitZ())


    R1=Rotation.RotZ( 45.0/180.0*pi)
    R2=Rotation.RotZ(-15.0/180.0*pi)
    print("R1")
    print(R1)
    print("R2")
    print(R2)
    print("inverse of a rotation matrix ")
    print(R1:Inverse())

    v=Vector(0,0.4,1)
    print("(pure rotational) transformation of a vector using a rotation matrix : ")
    v_transformed = R1*v
    print(v_transformed) 

    print("composition of two rotation matrices : ")
    R = R1*R2

    print(R)
