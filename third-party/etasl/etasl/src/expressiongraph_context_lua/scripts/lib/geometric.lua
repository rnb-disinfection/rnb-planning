require("context")
-- if ilua~=nil then
--     print( [[ 
-- geometric.lua called from ilua, most important functions:
--    translate_x(), translate_y(), translate_z(), rotate_x(), rotate_y(),
--    rotate_z(), angle_between_vectors(a,b), angle_between_vectors2(a,b),
--    angle_plane_vector(P,a), angle_plane_plane(P1,P2), distance_point_point(p1,p2),
--    distance_line_point(L,a), distance_line_line(L1,L2), distance_segment_segment(a1,a2,b1,b2),
--    distance_segment_point(a1,a2,b), 
--    camera_model(cu,cv,alpha,f,Cf,p)
--]])
--end

-- standard joints :

function translate_x(a)
    local msg="translate_x( a ) with a scalar variable or expression\n   translate along the X-axis with the given value\n\n"
    namedcheck({"first argument"},{a},{"expression_double|number"},msg)
    if extendedtype(a)=="number" then
        a = constant(a)
    end
    return frame(vector(a,constant(0),constant(0)))
end

function translate_y(a)
    local msg="translate_y( a ) with a scalar variable or expression\n   translate along the Y-axis with the given value\n\n"
    namedcheck({"first argument"},{a},{"expression_double|number"},msg)
    if extendedtype(a)=="number" then
        a = constant(a)
    end
    return frame(vector(constant(0),a,constant(0)))
end

function translate_z(a)
    local msg="translate_z( a ) with a scalar variable or expression\n   translate along the Z-axis with the given value\n\n"
    namedcheck({"first argument"},{a},{"expression_double|number"},msg)
    if extendedtype(a)=="number" then
        a = constant(a)
    end
    return frame(vector(constant(0),constant(0),a))
end

function rotate_x(a)
    local msg="rotate_x( a ) with a scalar variable or expression\n   rotates along the X-axis with the given value\n\n"
    namedcheck({"first argument"},{a},{"expression_double|number"},msg)
    if extendedtype(a)=="number" then
        a = constant(a)
    end
    return frame(rot_x(a))
end

function rotate_y(a)
    local msg="rotate_y( a ) with a scalar variable or expression\n   rotates along the Y-axis with the given value\n\n"
    namedcheck({"first argument"},{a},{"expression_double|number"},msg)
    if extendedtype(a)=="number" then
        a = constant(a)
    end
    return frame(rot_y(a))
end

function rotate_z(a)
    local msg="rotate_z( a ) with a scalar variable or expression\n   rotates along the Z-axis with the given value\n\n"
    namedcheck({"first argument"},{a},{"expression_double|number"},msg)
    if extendedtype(a)=="number" then
        a = constant(a)
    end
    return frame(rot_z(a))
end



-- angle between directed vectors : has a range of [0..pi[
function angle_between_vectors(a,b)
    local msg=[[ angle_between_vectors(a, b) 
                 INPUT: 
                    a : expression for the first vector
                    b : expression for the second vector
                 OUTPUT:
                    returns an expression for the angle between two vectors
                    the returned value is in the half-open interval [0..pi[ 

                 REMARK:
                    computed via dot-product.
    ]]
    if a==nil and b==nil then
        print(msg)
        return
    end
    namedcheck({"a","b"},{a,b},{"expression_vector","expression_vector"}, msg)
    local ca = cached(a)
    local cb = cached(b)
    return cached(acos(dot(ca/norm(ca),cb/norm(cb))))
end

-- smallest angle between lines : has a range of [0..pi/2[
function angle_between_vectors2(a,b)
    local msg=[[ angle_between_vectors2(a, b) 
                 INPUT: 
                    a : expression for the first vector
                    b : expression for the second vector
                 OUTPUT:
                    returns an expression for the angle between two vectors
                    the returned value is in the half-open interval [0..pi/2[ 

                 REMARK:
                    computed via cross-product.
    ]]
    if a==nil and b==nil then
        print(msg)
        return
    end
    namedcheck({"a","b"},{a,b},{"expression_vector","expression_vector"}, msg)
    local ca = cached(a)
    local cb = cached(b)
    return cached(asin( norm(cross(ca,cb))/norm(ca)/norm(cb) ))
end

-- angle between 
function angle_plane_vector(P,a)
    local msg=[[ angle_plane_vectors(P, a) 
                 INPUT: 
                    P : expression for a plane ( a frame with the plane corresponding to the
                                                 X-Y plane of the frame) 
                    a : expression for a directional vector
                 OUTPUT:
                    returns an expression for the angle between the vector and the plane
                    (i.e. between the normal of the plane and the vector)
                    the returned value is in the half-open interval [0..pi[ 

                 REMARK:
                    computed via dot-product and acos.
    ]]
    if P==nil and a==nil then
        print(msg)
        return
    end
    namedcheck({"P","a"},{P,a},{"expression_frame","expression_vector"}, msg)
    local ca = cached(a)
    return cached(acos( dot( unit_z(rotation(P)), ca) / norm(ca) ))
end


function angle_plane_plane(P1,P2)
    local msg=[[ angle_plane_plane(P1, P2) 
                 INPUT: 
                    P1 : expression for a plane ( a frame with the plane corresponding to the
                                                 X-Y plane of the frame) 
                    P2 : expression for a plane ( a frame with the plane corresponding to the
                                                 X-Y plane of the frame) 
                 OUTPUT:
                    returns an expression for the angle between the two planes
                    (i.e. between the normals of the planes )
                    the returned value is in the half-open interval [0..pi[ 

                 REMARK:
                    computed via dot-product and acos.
    ]]
    if P1==nil and P2==nil then
        print(msg)
        return
    end
    namedcheck({"P1","P2"},{P1,P2},{"expression_frame","expression_frame"}, msg)
    return acos( dot( unit_z(rotation(P1)), unit_z(rotation(P2)) ) )
end

function distance_point_point(p1,p2)
    local msg=[[ distance_point_point(p1, p2) 
                 INPUT: 
                    p1 : expression for a vector representing a point
                    p2 : expression for a vector representing a point
                 OUTPUT:
                    returns the distance between the two points.

    ]]
    if p1==nil and p2==nil then
        print(msg)
        return
    end
    namedcheck({"p1","p2"},{p1,p2},{"expression_vector","expression_vector"}, msg)
 
    check({p1,p2},{"expression_vector","expression_vector"})
    return norm(p1-p2)
end



function distance_line_point(Line,point)
    local msg=[[ distance_line_point(Line, point) 
                 INPUT: 
                    Line : expression for a frame representing a line 
                            (the z-axis of the frame represents the line)
                    point : expression for a vector representing a point
                 OUTPUT:
                    returns the distance between the line and the point

    ]]
    if Line==nil and point==nil then
        print(msg)
        return
    end
    namedcheck({"Line","point"},{Line,point},{"expression_frame","expression_vector"}, msg)
 
    local point2= cached(point)
    local Line2 = cached(Line)
    local d = cached( point2 - origin(Line2)  )
    local n = cached( unit_z(rotation(Line2)) )
    return cached(norm( d - n*dot(n,d) ))
end

function distance_line_line(L1,L2)
    local msg=[[ distance_line_line(L1,L2) 
                 INPUT: 
                   L1 : expression for a frame representing a line 
                            (the z-axis of the frame represents the line)
                   L2 : expression for a frame representing a line 
                            (the z-axis of the frame represents the line)
                    point : expression for a vector representing a point
                 OUTPUT:
                    returns the distance between the two lines

    ]]
    if L1==nil and L2==nil then
        print(msg)
        return
    end
    namedcheck({"L1","L2"},{L1,L2},{"expression_frame","expression_frame"}, msg)
 
    local n1 = cached( unit_z(rotation(L1)) )
    local n2 = cached( unit_z(rotation(L2)) )
    local n  = cached(cross(n1,n2))
    local nn = cached(norm(n))
    return cached(near_zero(nn,1E-8,
                abs(dot(origin(L1)-origin(L2),n/nn)),
                distance_line_point(L2,origin(L1))   -- for // lines, 
           ))
end


function camera_model(cu,cv,alpha,f, CF, p)
    local msg = [[
    camera model from Chaumette2006-RAM paper

    function camera_model(cu,cv,alpha,f, CF, p)
    INPUT:
        cu,cv coordinates of principle point
        alpha ratio of pixel dimensions.  
        f     focal length in pixels
        The above parameters are no expression graphs, but just numerical constants.
        
        CF    camera frame Z-axis along camera, X-axis horizontal, Y-axis pointing to top side of image.
        p     point (CF and p in the same coord. frame expressed)
    OUTPUT:
        u,v coordinates in the image.    

    ex. 36 x 24 mm full frame camera with 50mm lens at 1920x1280 pixels, with p in [m]
        alpha = 1
        f=1920/0.036*0.050 = 2666.666
        cu = 960
        cv = 640

    REFERENCES:
        [1] Chaumette, F., & Hutchinson, S. (2006). Visual servo control. I. Basic approaches. 
            IEEE Robotics & Automation Magazine, 13(4), 82-90. 
        [2] Chaumette, F., & Hutchinson, S. (2007). Visual servo control, Part II: Advanced 
            approaches. IEEE Robotics and Automation Magazine, 14(1), 109-118.
    ]]
    namedcheck({"cu","cv","alpha","f","CF","p"},
             {cu,cv,alpha,f,CF,p},
             {"number","number","number","number","expression_frame","expression_vector"},
             msg)
    p_rel = cached(  inv(CF)*p )
    u = constant(cu) + constant(f*alpha)*coord_x(p_rel)/coord_z(p_rel)
    v = constant(cv) + constant(f)*coord_y(p_rel)/coord_z(p_rel)
    return u,v
end



-- you can specify segments by begin and end point table.
function distance_segment_segment(a1,a2,b1,b2)
    local msg=[[ distance_segment_segment(a1,a2,b1,b2) 

                 computes the distance between the finite line segment between
                 a1 and a2, and the finite line segment between b1 and b2.

                 INPUT: 
                   a1 : expression for a vector representing the point a1
                   a2 : expression for a vector representing the point a2
                   b1 : expression for a vector representing the point b1
                   b2 : expression for a vector representing the point b2
                 OUTPUT:
                    returns the distance between the two segments

    ]]
    if a1==nil and a2==nil and b1==nil and b2==nil then
        print(msg)
        return
    end
    namedcheck({"a1","a2","b1","b2"},{a1,a2,b1,b2},
               {"expression_vector","expression_vector","expression_vector","expression_vector"}, msg)
 
    local A11         = cached( dot( a2-a1, a2-a1) );
    local A12         = cached(-dot( a2-a1, b2-b1) );  
    local B1          = cached(-dot( a2-a1, b2-a2) );
    local A22         = cached( dot( b2-b1, b2-b1) );
    local B2          = cached( dot( b2-b1, b2-a2) );
    local denominator = cached( A11*A22 - A12*A12 + constant(1E-8) );
    local lambda1     = cached( (B1*A22-A12*B2) / denominator );
    local lambda2     = cached( (A11*B2-A12*B1) / denominator );
    local lambda1_lim = cached( minimum( maximum(constant(0),lambda1), constant(1) ) )
    local lambda2_lim = cached( minimum( maximum(constant(0),lambda2), constant(1) ) )

    local p1 = cached( lambda1_lim*a1 + (constant(1)-lambda1_lim)*a2 );
    local p2 = cached( lambda2_lim*b1 + (constant(1)-lambda2_lim)*b2 );
    return cached(norm(p2-p1)),p1,p2
end

-- you can specify segments by begin and end point or by a {begin_point, end_point}  table.
function distance_segment_point(a1,a2,b )
    local msg=[[ distance_segment_point(a1,a2,p) 

                 computes the distance between the finite line segment between
                 a1 and a2, and the point p 

                 INPUT: 
                   a1 : expression for a vector representing the point a1
                   a2 : expression for a vector representing the point a2
                   p : expression for a vector representing the point p 
                 OUTPUT:
                    returns the distance between the point and the line segment a1-a2.

    ]]
    if a1==nil and a2==nil and b==nil  then
        print(msg)
        return
    end
    namedcheck({"a1","a2","p"},{a1,a2,b},
               {"expression_vector","expression_vector","expression_vector"}, msg)
 
    local a12        = a2-a1;
    local lambda     = cached( dot(a12,b-a1) / ( dot(a12,a12) + Constant(1E-8)));
    local lambda_lim = cached( minimum( maximum(Constant(0),lambda), constant(1) ) )
    local p          = (constant(1.0)-lambda_lim)*a1  + lambda_lim*a2;
    return cached(norm(p-b)), p
end


function trajectory_circular( pathv,center, radius )
    local msg=[[ trajectory_circular(pathv, center, radius) 

                 expression that represents a circular trajectory with
                 a center point in the frame 'center', lying in the plane X-Y of
                 the frame 'center', with the given 'radius' and starting
                 at the x-axis in counter-clockwise direction ( according to the
                 orientation of the z-axis)

                 INPUT:
                   pathv : an expression for a number representing the path variable.  
                           Expressed in distance units.
                   center : a frame, the origin gives the center of
                           the circular path, the X-Y plane the plane of the trajectory. 
                   radius : a number representing the radius of the trajectory.
                 OUTPUT:
                    an expression for the circular trajectory.
    ]]
    if pathv==nil and center==nil and radius==nil  then
        print(msg)
        return
    end
    namedcheck({"pathv","center","radius"},{pathv, center,radius},
               {"expression_double","Frame","number"}, msg)
 
    check({pathv,center,radius},{"expression_double","Frame","number"})
    local R=constant(radius)
    local w=constant(1.0/radius)
    return cached( constant(center) * vector( R*sin(w*pathv), R*cos(w*pathv), constant(0)))
end

local ftable={
    translate_x             =   translate_x,
    translate_y             =   translate_y,
    translate_z             =   translate_z,
    rotate_x                =   rotate_x,
    rotate_y                =   rotate_y,
    rotate_z                =   rotate_z,
    angle_between_vectors   =   angle_between_vectors,
    angle_between_vectors2  =   angle_between_vectors2,
    angle_plane_vector      =   angle_plane_vector,
    angle_plane_plane       =   angle_plane_plane,
    distance_point_point    =   distance_point_point,
    distance_line_point     =   distance_line_point,
    distance_line_line      =   distance_line_line,
    camera_model            =   camera_model,
    distance_segment_segment=   distance_segment_segment,
    distance_segment_point  =   distance_segment_point,
    trajectory_circular     =   trajectory_circular
}

ftable['contents'] = _contents(ftable,'geometric')
ftable['help']     = _help(ftable,'geometric')

return ftable
