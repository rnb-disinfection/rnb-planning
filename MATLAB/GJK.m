function [dist,flag] = GJK(shape1,shape2,iterations)
% GJK Gilbert-Johnson-Keerthi Collision detection implementation.
% Returns whether two convex shapes are are penetrating or not
% (true/false). Only works for CONVEX shapes.
%
% Inputs:
%   shape1: 
%   must have fields for XData,YData,ZData, which are the x,y,z 
%   coordinates of the vertices. Can be the same as what comes out of a 
%   PATCH object. It isn't required that the points form faces like patch
%   data. This algorithm will assume the convex hull of the x,y,z points
%   given.
%
%   shape2: 
%   Other shape to test collision against. Same info as shape1.
%   
%   iterations: 
%   The algorithm tries to construct a tetrahedron encompassing
%   the origin. This proves the objects have collided. If we fail within a
%   certain number of iterations, we give up and say the objects are not
%   penetrating. Low iterations means a higher chance of false-NEGATIVES
%   but faster computation. As the objects penetrate more, it takes fewer
%   iterations anyway, so low iterations is not a huge disadvantage.
%   
% Outputs:
%   flag:
%   true - objects collided
%   false - objects not collided
%
%  
%   This video helped me a lot when making this: https://mollyrocket.com/849
%   Not my video, but very useful.
%   
%   Matthew Sheen, 2016
%

%Point 1 and 2 selection (line segment)
v = [0.8 0.5 1];
dist = 0;
[a,b] = pickLine(v,shape2,shape1);

%Point 3 selection (triangle)
[a,b,c,flag] = pickTriangle(a,b,shape2,shape1,20);


%Point 4 selection (tetrahedron)
if flag == 1 %Only bother if we could find a viable triangle.
    [a,b,c,d,dist,flag] = pickTetrahedron(a,b,c,shape2,shape1,iterations);
end

end

function [a,b] = pickLine(v,shape1,shape2)
%Construct the first line of the simplex
b = support(shape2,shape1,v);
a = support(shape2,shape1,-v);
end

function [a,b,c,flag] = pickTriangle(a,b,shape1,shape2,IterationAllowed)
flag = 0; %So far, we don't have a successful triangle.

%First try:
ab = b-a;
ao = -a;
v = cross(cross(ab,ao),ab); % v is perpendicular to ab pointing in the general direction of the origin.

c = b;
b = a;
a = support(shape2,shape1,v);

for i = 1:IterationAllowed %iterations to see if we can draw a good triangle.
    %Time to check if we got it:
    [b_,c_,v,dist] = pickClosestLine(a,b,c);
    a_ = support(shape2,shape1,v);
    if dist<=0
        flag = 1; 
        break; %We got a good one.
    else
        a = a_;
        b = b_;
        c = c_;
    end
    
end
end

function [b,c,v,dist] = pickClosestLine(a,b,c)
    Imat = [1, 2; 1, 3];
    ab = b-a;
    ao = -a;
    ac = c-a;

    %Normal to face of triangle
    abc = cross(ab,ac);

    %Perpendicular to AB going away from triangle
    abp = cross(ab,abc);
    %Perpendicular to AC going away from triangle
    acp = cross(abc,ac);

    abpo = dot(abp,ao);
    acpo = dot(acp, ao);

    [dist, I] = max([abpo ,acpo]);
    abc = [a;b;c];
    v_candi = [abp;acp];
    Ivec = Imat(I,:);
    b  = abc(Ivec(1),:);
    c  = abc(Ivec(2),:);
    v = v_candi(I,:);
end


function [a,b,c,abc,dist] = pickClosestFace(a,b,c,d)
    Imat = [1, 2, 3; 1, 3, 4; 1, 4, 2];
    
    %Check the tetrahedron:
    ab = b-a;
    ao = -a;
    ac = c-a;
    ad = d-a;
    
    %We KNOW that the origin is not under the base of the tetrahedron based on
    %the way we picked a. So we need to check faces ABC, ABD, and ACD.
    
    %Normal to face of triangle
    abc = cross(ab,ac);
    acd = cross(ac,ad);%Normal to face of triangle
    adb = cross(ad,ab);%Normal to face of triangle
    abco = dot(abc, ao);
    acdo = dot(acd, ao);
    adbo = dot(adb, ao);
    [dist, I] = max([abco,acdo,adbo]);
% %     [M, dist] = max([abco,acdo,adbo]);
% %     dist = dist-1;
    abcd = [a;b;c;d];
    abc_candi = [abc;acd; adb];
    Ivec = Imat(I,:);
    a  = abcd(Ivec(1),:);
    b  = abcd(Ivec(2),:);
    c  = abcd(Ivec(3),:);   
    abc = abc_candi(I,:);
end

function [a,b,c,d,dist,flag] = pickTetrahedron(a,b,c,shape1,shape2,IterationAllowed)
%Now, if we're here, we have a successful 2D simplex, and we need to check
%if the origin is inside a successful 3D simplex.
%So, is the origin above or below the triangle?
flag = 0;

ab = b-a;
ac = c-a;

%Normal to face of triangle
abc = cross(ab,ac);
ao = -a;

if dot(abc, ao) > 0 %Above
    d = c;
    c = b;
    b = a;
    
    v = abc;
    a = support(shape2,shape1,v); %Tetrahedron new point;
    
else %below
    d = b;
    b = a;
    v = -abc;
    a = support(shape2,shape1,v); %Tetrahedron new point
end

for i = 1:IterationAllowed %Allowing 10 tries to make a good tetrahedron.
    [a,b,c,abc,dist] = pickClosestFace(a,b,c,d);
    if dist<=0
        flag = 1; 
    end
    if dot(abc, ao) > 0 %Above
        d = c;
        c = b;
        b = a;    
        v = abc;
        a = support(shape2,shape1,v); %Tetrahedron new point
    else %below
        d = b;
        b = a;
        v = -abc;
        a = support(shape2,shape1,v); %Tetrahedron new point
    end
%     dist = min(sqrt(a*a'), abs(a*v'));
    if flag>0
        break; %It's inside the tetrahedron.
    end
    flag = -i;
end
end

function point = getFarthestInDir(shape, v)
%Find the furthest point in a given direction for a shape
XData = shape.Vertices(:,1); % shape.XData; % Making it more compatible with previous MATLAB releases.
YData = shape.Vertices(:,2); % shape.YData;
ZData = shape.Vertices(:,3); % shape.ZData;
dotted = XData*v(1) + YData*v(2) + ZData*v(3);
[maxInCol,rowIdxSet] = max(dotted);
point = [XData(rowIdxSet), YData(rowIdxSet), ZData(rowIdxSet)];
end

function point = support(shape1,shape2,v)
%Support function to get the Minkowski difference.
point1 = getFarthestInDir(shape1, v);
point2 = getFarthestInDir(shape2, -v);
point = point1 - point2;
end
