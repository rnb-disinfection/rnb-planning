-- library to define a virtual funnel
--
--
require("context")
require("geometric")
require("collision")
require("io")

function help_funnel()
    print( [[ 
        Defines a funnel object to facilitate  guiding a robot to a pick-up point.
        The following functions are defined (calling with no parameters displays additional
        help):
        - create_funnel
        - distance_to_funnel 
        - write_funnel 

]])

end

if ilua~=nil then
     print( [[ funnel.lua called from ilua, type help_funnel() for more help on funnel.lua ]]);
end
local function convert_to_expression(a)
    if extendedtype(a)=="number" or 
       extendedtype(a)=="Vector" or
       extendedtype(a)=="Rotation" or
       extendedtype(a)=="Frame" or 
       extendedtype(a)=="Twist" then
        a=constant(a)
    end
    return a
end

local function create_block_points(b,l,h,B,L,H)
    namedcheck({"b","l","h","B","L","H"},
               {b,l,h,B,L,H},
               {"number","number","number","number","number","number"},
                [[
    function create_block_points(b,l,h,B,L,H) 
        b,l,h stand for the width, the length and the height of the inner hole.
        B,L,H stand for the width, the length and the height of the outer funnel box.
                    ]]);
    local v     = {
        Vector( b/2, -l/2,    0),
        Vector( B/2, -L/2,    0),
        Vector( B/2,  L/2,    0),
        Vector( b/2,  l/2,    0),
        Vector( b/2, -l/2,    h),
        Vector( B/2, -L/2,    H),
        Vector( B/2,  L/2,    H),
        Vector( b/2,  l/2,    h)
    }
    local radii = {0,0,0,0,0,0,0,0};
    return v, radii 
end



local function create_block(b,l,h,B,L,H)
    local v,radii
    v,radii = create_block_points(b,l,h,B,L,H)
    local shape = MultiSphere(v,radii)
    return shape
end


function create_funnel(T_ref_funnel, b,l,h,B,L,H)
    namedcheck({"T_ref_funnel","b","l","h","B","L","H"},
               {T_ref_funnel,b,l,h,B,L,H},
               {"expression_frame|Frame","number","number","number","number","number","number"},
                [[
    function create_funnel(T_ref_funnel, b,l,h,B,L,H) 
        returns shapes,locations.

        with T_ref_funnel a frame and the other arguments are numbers
        b,l,h stand for the width, the length and the height of the inner hole.
        B,L,H stand for the width, the length and the height of the outer funnel box.

    the function returns a list of convex shapes and a list of their locations
                    ]]);
    T_ref_funnel = convert_to_expression(T_ref_funnel)

    local shapes={}
    local locations={}
    local deg2rad = math.pi / 180.0
    shapes[1]     = create_block(b,l,h,B,L,H)
    shapes[2]     = create_block(l,b,h,L,B,H)
    shapes[3]     = create_block(b,l,h,B,L,H)
    shapes[4]     = create_block(l,B,h,L,B,H)
    locations[1]  = cached(T_ref_funnel * rotate_z(0))
    locations[2]  = cached(T_ref_funnel * rotate_z(90*deg2rad))
    locations[3]  = cached(T_ref_funnel * rotate_z(180*deg2rad))
    locations[4]  = cached(T_ref_funnel * rotate_z(270*deg2rad))
    return shapes,locations
end

function distance_to_funnel( p, shapes, locations, radius, margin)
    namedcheck( {"p","locations","radius","margin"},
                {p,locations,radius,margin},
                {"expression_vector|Vector","table_expression_frame","number","number"},
                [[
    function distance_to_funnel( p, shapes, locations,radius,margin)
        computes the distance of a reference point to a funnel
            - p : reference point
            - shapes : a table of shapes that together describe the funnel
            - locations: a table of frames describing the locations of the shapes
            - radius: additional radius added to both funnel and point.
            - margin : margin for distance computations.
                ]]
     )
    p = convert_to_expression(p)
    local d = constant(1E10)
    local shape_p = MultiSphere({Vector(0,0,0)},{0})
    for i,pv in ipairs(shapes) do
        local di = distance_between( frame(p), shape_p, radius, margin, locations[i], shapes[i], radius, margin )
        d  = minimum( d, di )
    end
    return cached(d)
end


local function write_block_obj(f, T, b,l,h, B,L,H, vertexcount)
    local v,radii = create_block_points(b,l,h,B,L,H)
    local vt = {}
    for i=1,#v,1 do
        vt[i] = T*v[i]
    end
    for i=1,#v,1 do
        f:write("v "..vt[i]:x().." "..vt[i]:y().." "..vt[i]:z().."\n")
    end
    faces = { {1,5,8, 4}, 
              {1,2,6, 5},
              {2,3,7, 6},
              {4,3,7, 8},
              {1,4,3, 2},
              {5,6,7, 8} };
    for i=1,#faces,1 do
        f:write("f "..  (faces[i][1]+vertexcount-1).." "
                    .. (faces[i][2]+vertexcount-1) .." "
                    .. (faces[i][3]+vertexcount-1).." "
                    .. (faces[i][4]+vertexcount-1).."\n")
    end
    vertexcount=vertexcount+8
    return vertexcount
end



function write_funnel(f,T, b,l,h,B,L,H,vertexcount)
    namedcheck({"f","T","b","l","h","B","L","H","vertexcount"},
               {f,T,b,l,h,B,L,H,vertexcount},
               {"userdata","Frame","number","number","number","number","number","number","number"},
                [[
    function write_funnel(f,T, b,l,h,B,L,H,vertexcount)
        with f a file to write to
        with T a frame that indicates the origin of the funnel. 
        b,l,h stand for the width, the length and the height of the inner hole.
        B,L,H stand for the width, the length and the height of the outer funnel box.
                    ]]
    )
 
    local deg2rad = math.pi / 180.0
    vertexcount     = write_block_obj(f,T,b,l,h,B,L,H,vertexcount)
    vertexcount     = write_block_obj(f,T*Frame(Rotation.EulerZYX(90*deg2rad,0,0)),l,b,h,L,B,H,vertexcount)
    vertexcount     = write_block_obj(f,T*Frame(Rotation.EulerZYX(180*deg2rad,0,0)),b,l,h,B,L,H,vertexcount)
    vertexcount     = write_block_obj(f,T*Frame(Rotation.EulerZYX(270*deg2rad,0,0)),l,b,h,L,B,H,vertexcount)
    return vertexcount
end



function tstfunnel()
    f=io.open("tst.obj","w")
    vertexcount=1
    vertexcount = write_funnel(f,Frame.Identity(), 1,0.5, 1.5, 6, 6, 2,vertexcount)
    f:close()
end


