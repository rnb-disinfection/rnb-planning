-- definition of additional type checking for the collision avoidance routines.
-- also provides some automatic type conversion where appropriate.
--
require("context")
require("libexpressiongraph_collision")

function help_collision()
    print( [[ 
    printing the content of the collision.lua library. Further help() can be obtained by
    calling the function with no arguments, e.g. Box()

    This package provides shapes and computes the distances between those shapes.

    The following shapes are defined:
        - Box, 
        - MultiSphere
        - CapsuleX, CapsuleY, CapsuleZ
        - ConeX,ConeY,ConeZ
        - CylinderX, CylinderY, CylinderZ
        - ConvexObject, ConvexObjectScale

    The following function computes the distance:
        - distance_between
]])

end

--if ilua~=nil then
--     print( [[ collision.lua called from ilua, type help_collision() for more help on collision.lua ]]); 
--end

local collision_lib={} 

collision_lib.Box = Box
function Box(B,L,H)
    local msg= [[
    function Box(B,L,H)
        create a box shape
            with B the length along the x axis
            with L the length along the y axis
            with H the length along the z axis
        ]]
    if B=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"B","L","H"},
        {B,L,H},
        {"number","number","number"},
        msg);
    return collision_lib.Box(B,L,H)
end

collision_lib.MultiSphere = MultiSphere 
function MultiSphere(centers,radii)
    local msg = [[
    function MultiSphere(centers,radii)
        create a shape consisting of the convex hull of a series of spheres 
            with centers a table of vectors describing the origin of the spheres 
            with radii   a table of numbers describing the radii of the spheres 
    ]]
    if (centers=='help') then
        print(msg)
        return
    end
    namedcheck(
        {"centers","radii"},
        {centers,radii},
        {"table_Vector","table_number"},
        msg
    );
    return collision_lib.MultiSphere(centers,radii)
end

collision_lib.CapsuleX = CapsuleX
function CapsuleX(radius,length)
    local msg =
        [[
    function CapsuleX(radius,length)
        create a capsule shape along the X-axis
            with radius corresponding to the radius of the spherical endcaps of the capsule
            with length corresponding to the length between the centers of the endcaps.
        ]]
    if radius=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"radius","length"},
        {radius,length},
        {"number","number"},
        msg
    )
    return collision_lib.CapsuleX(radius,length)
end

collision_lib.CapsuleY = CapsuleY
function CapsuleY(radius,length)
    local msg =  [[
    function CapsuleY(radius,length)
        create a capsule shape along the Y-axis
            with radius corresponding to the radius of the spherical endcaps of the capsule
            with length corresponding to the length between the centers of the endcaps.
    ]]
    if radius=='help' then
       print(msg)
        return
    end
    namedcheck(
        {"radius","length"},
        {radius,length},
        {"number","number"},
        msg
    )
    return collision_lib.CapsuleY(radius,length)
end

collision_lib.CapsuleZ = CapsuleZ
function CapsuleZ(radius,length)
    local msg =  [[
    function CapsuleZ(radius,length)
        create a capsule shape along the X-axis
            with radius corresponding to the radius of the spherical endcaps of the capsule
            with length corresponding to the length between the centers of the endcaps.
        ]]
    if radius=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"radius","length"},
        {radius,length},
        {"number","number"},
        msg
    )
    return collision_lib.CapsuleZ(radius,length)
end

collision_lib.ConeX = ConeX
function ConeX(radius,height)
    local msg = [[
    function ConeX(radius,height)
        create a cone shape along the X-axis
            with radius corresponding to the radius of the cylindrical base of the capsule
            with height corresponding to the heigth of the cone.
        ]]
    if radius=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"radius","height"},
        {radius,height},
        {"number","number"},
        msg
    )
    return collision_lib.ConeX(radius,height)
end


collision_lib.ConeY = ConeY
function ConeY(radius,height)
    local msg =  [[
    function ConeY(radius,height)
        create a cone shape along the Y-axis
            with radius corresponding to the radius of the cylindrical base of the capsule
            with height corresponding to the heigth of the cone.
        ]]
    if radius=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"radius","height"},
        {radius,height},
        {"number","number"},
        msg
    )
    return collision_lib.ConeY(radius,height)
end

collision_lib.ConeZ = ConeZ
function ConeZ(radius,height)
    local msg=
        [[
    function ConeZ(radius,height)
        create a cone shape along the Z-axis
            with radius corresponding to the radius of the cylindrical base of the capsule
            with height corresponding to the heigth of the cone.
        ]]
    if radius=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"radius","height"},
        {radius,height},
        {"number","number"},
        msg
    )
    return collision_lib.ConeZ(radius,height)
end


collision_lib.CylinderX = CylinderX
function CylinderX(widthx,widthy,widthz)
    local msg=
    [[ 
    function CylinderX(widthx, widthy, widthz)
        create a cylindrical shape with the axis along the X-axis
        where:
            - widthx : corresponds to the width along the x-axis
            - widthy : corresponds to the width along the y-axis
            - widthz : corresponds to the width along the z-axis
    ]] 
    if widthx=='help' then
        print(msg)
        return msg
    end
    namedcheck(
        {"widthx","widthy","widthz"},
        {widthx, widthy, widthz},
        {"number","number","number"},
        msg
    )
    return collision_lib.CylinderX(widthx,widthy,widthz)
end


collision_lib.CylinderY = CylinderY
function CylinderY(widthx,widthy,widthz)
    local msg=
    [[ 
    function CylinderY(widthx, widthy, widthz)
        create a cylindrical shape with the axis along the Y-axis
        where:
            - widthx : corresponds to the width along the x-axis
            - widthy : corresponds to the width along the y-axis
            - widthz : corresponds to the width along the z-axis
    ]] 
    if widthx=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"widthx","widthy","widthz"},
        {widthx, widthy, widthz},
        {"number","number","number"},
        msg
    )
    return collision_lib.CylinderY(widthx,widthy,widthz)
end


collision_lib.CylinderZ = CylinderZ
function CylinderZ(widthx,widthy,widthz)
    local msg =
    [[ 
    function CylinderZ(widthx, widthy, widthz)
        create a cylindrical shape with the axis along the Z-axis
        where:
            - widthx : corresponds to the width along the x-axis
            - widthy : corresponds to the width along the y-axis
            - widthz : corresponds to the width along the z-axis
    ]] 
    if widthx=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"widthx","widthy","widthz"},
        {widthx, widthy, widthz},
        {"number","number","number"},
        msg
    )
    return collision_lib.CylinderZ(widthx,widthy,widthz)
end






collision_lib.ConvexObject = ConvexObject
function ConvexObject( filename )
    local msg =
        [[
    function ConvexObject( filename )
        create a convex shape from a .obj file
            filename : file name to load
                (filename cannot be 'help')

        The .obj file (wafefront .obj) contains tekst with lines
        only lines starting with 'v' and 3 numbers are taken into
        account (vertices).  'f' and 'o' are ignored, '#' is used for comments. 
        ]]
    if filename=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"filename"},
        {filename},
        {"string"},
        msg
    )
    return collision_lib.ConvexObject( filename )
end

collision_lib.ConvexObjectScale = ConvexObjectScale
function ConvexObjectScale( filename, scalex, scaley, scalez )
    local msg =     [[
    function ConvexObjectScale( filename, scalex, scaley, scalez )

        create a convex shape from a stl file
        scale:    if < 0, then abs(scale) the object will be scaled such that its largest extend along one of the axes
                  will correspond to abs(scale)
                  if > 0  then the object is scaled using abs(scale) scale factor

        if scaley and scalez are equal to zero, scalex is a universal scale ( can still be < 0 to specify the size of the largest extend)
        
        INPUT: 
            filename : file name to load (filename cannot be 'help')
            scalex   : scale in x direction
            scaley   : scale in y direction
            scalez   : scale in z direction
        ]]
    if filename=='help' then
        print(msg)
        return
    end
    namedcheck(
        {"filename","scalex","scaley","scalez"},
        {filename, scalex,scaley,scalez},
        {"string","number","?number","?number"},
        msg
    )
    if scaley==nil then
        scaley=0
    end
    if scalez==nil then
        scalez=0
    end
    return collision_lib.ConvexObjectScale( filename )
end

function is_shape( s )
    local n = extendedtype(s)
    return n=="BoxShape" or
           n=="CapsuleShapeX" or
           n=="CapsuleShapeY" or
           n=="CapsuleShapeZ" or
           n=="ConeShapeX" or 
           n=="ConeShapeY" or 
           n=="ConeShapeZ" or 
           n=="CylinderShapeX" or 
           n=="CylinderShapeY" or 
           n=="CylinderShapeZ" or 
           n=="MultiSphereShape" or
           n=="ConvexObjectShape"
end

collision_lib.distance_between = distance_between
function distance_between( frame1, shape1, radius1, margin1, frame2, shape2, radius2, margin2)
    local msg =
    [[
        function distance_between( frame1, shape1, radius1, margin1, frame2, shape2, radius2, margin2)
            create an expression for the distance between two shapes
                - frame1   : expression for the pose of shape1
                - shape1   : object representing a shape1
                - radius1  : the shape can be spherically expanded by a given radius1
                - margin1  : margin used to make the GJK algorithm more robust
                - frame2   : expression for the pose of shape2
                - shape2   : object representing a shape2
                - radius2  : the shape can be spherically expanded by a given radius2
                - margin2  : margin used to make the GJK algorithm more robust
    ]]
    if frame1=='help' then
        print(msg)
        return
    end
    namedcheck(
        { "frame1" ,"radius1", "margin1", "frame2","radius2","margin2"},
        { frame1  ,   radius1,   margin1,   frame2,   radius2,  margin2 },
        { "expression_frame|Frame", "number","number","expression_frame|Frame","number","number"},
        msg
    )
    frame1 = convert_to_expression(frame1)
    frame2 = convert_to_expression(frame2)
    return collision_lib.distance_between(frame1,shape1, radius1, margin1, frame2, shape2, radius2, margin2 )
end

local ftable={
    Box                         =   Box,
    MultiSphere                 =   MultiSphere,
    CapsuleX                    =   CapsuleX,
    CapsuleY                    =   CapsuleY,
    CapsuleZ                    =   CapsuleZ,
    ConeX                       =   ConeX,
    ConeY                       =   ConeY,
    ConeZ                       =   ConeZ,
    CylinderX                   =   CylinderX,
    CylinderY                   =   CylinderY,
    CylinderZ                   =   CylinderZ,
    ConvexObject                =   ConvexObject,
    ConvexObjectScale           =   ConvexObjectScale,
    distance_between            =   distance_between
}

ftable['contents'] = _contents(ftable,"collision")
ftable['help']     = _help(ftable,"collision")

return ftable
