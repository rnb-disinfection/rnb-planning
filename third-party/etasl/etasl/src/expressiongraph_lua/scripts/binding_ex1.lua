    
    require("expressiongraph")

    -- static methods are accessed using "."
    -- normal methods are accessed using ":"
    
    -- kdl vector an its operations :
    v=Vector(0.1, 0.5, 0.5)
    print(v)       -- print a vector
    print( v:x() )   -- print out the x-component of a vector
    print( v:y() )
    print( v:z() )
    v2 = Vector.Zero()  -- returns a zero vector
    print( v2 )

