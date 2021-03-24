--
-- Experiment to solve an optimal control problem related to a motion profile.
--
-- NOT FINISHED!1
--

require("context")
require("bspline")
pprint = require("pprint")


-- generates a spline between in function of u with start point expression and end point expression
-- and corresponding feature variables as control points.
-- base_name is a prefix for the feature variable name.
function generate_cp( context, base_name, knots, degree )
    local N = #knots - degree  + 1
    local cp = {}
    for i=1,N do
        local vn = base_name..i
        cp[i] = Variable{ context=context,name=vn,vartype="feature" };
    end
    return cp
end

function generate_cp_zerovel( context, base_name, startp, endp, knots, degree )
    local N = #knots - degree  + 1
    local cp = {}
    cp[1] = startp
    cp[2] = startp
    for i=3,N-2 do
        local vn = base_name..i
        cp[i] = Variable{ context=context,name=vn,vartype="feature" };
    end
    cp[N-1] = endp
    cp[N] = endp
    return cp
end



function generate_spline( u, knots, cp, degree)
    local expr = constant(0)
    for i = 1, #cp do
        expr = expr + cp[i]*bspline_basis(u, knots, i, degree)
    end
    return cached(expr)
end

function generate_spline_d( u, knots, cp, degree)
    local expr = constant(0)
    local d_knots
    local d_basis_cp
    for i = 1, #cp do
        local basis_cp = control_points( knots, i, degree)
        d_knots, d_basis_cp, d_degree = derivknots( knots, basis_cp, degree)
        expr = expr + cp[i]*bspline_expr(u, d_knots, d_basis_cp, d_degree)
    end
    return cached(expr)
end

function generate_spline_dd( u, knots, cp, degree)
    local expr = constant(0)
    local d_knots
    local dd_knots
    local d_basis_cp
    local dd_basis_cp
    for i = 1, #cp do
        local basis_cp = control_points( knots, i, degree)
        d_knots, d_basis_cp, d_degree = derivknots( knots, basis_cp, degree)
        dd_knots, dd_basis_cp, dd_degree = derivknots( d_knots, d_basis_cp, d_degree)
        expr = expr + cp[i]*bspline_expr(u, dd_knots, dd_basis_cp, dd_degree)
    end
    return cached(expr)
end


function linspace(start,stop,N)
    local r={}
    for i=0,N-1 do
        table.insert(r, start + i * (stop-start)/(N-1) )
    end
    return r
end

--
degree = 3
knots  = {0,0,0,0.5,1,1.5, 2,2.5, 3,3.5,4,4.5,5,5,5}
tau    = linspace(0,5,10)

maxvel = 0.3
maxacc = 0.5

cp = generate_cp(ctx, "fv", knots, degree) 
Constraint{
    context=ctx,
    name="startpos",
    expr= generate_spline( constant(tau[1]), knots, cp, degree),
    target = 0,
    K=10
}
Constraint{
    context=ctx,
    name="startvel",
    expr= generate_spline_d( constant(tau[1]), knots, cp, degree),
    target = 0,
    K=10
}
Constraint{
    context=ctx,
    name="endpos",
    expr= generate_spline( constant(tau[#tau]), knots, cp, degree),
    target = 1,
    K=10
}
Constraint{
    context=ctx,
    name="endvel",
    expr= generate_spline_d( constant(tau[#tau]), knots, cp, degree),
    target = 0,
    K=10
}
for i,v in ipairs(tau) do
    if v > 0.1 then
        Constraint{
            context=ctx,
            name="position_"..i,
            expr= generate_spline( constant(v), knots, cp, degree),
            target = 1,
            K=10,
            weight=0.0001
        }
    end
    Constraint{
        context=ctx,
        name="velocity_"..i,
        expr= generate_spline_d( constant(v), knots, cp, degree),
        target_lower = -maxvel,
        target_upper = maxvel,
        K=10
    }
    Constraint{
        context=ctx,
        name="acceleration_"..i,
        expr= generate_spline_dd( constant(v), knots, cp, degree),
        target_lower = -maxacc,
        target_upper = maxacc,
        K=10
    }
end

e     = generate_spline( time, knots, cp, degree);





--[[
cp = generate_cp_zerovel(ctx, "fv", constant(0), constant(1), knots, degree) 

e1    = generate_spline( constant(0.1), knots, cp, degree);
e1_d  = generate_spline_d( constant(0.1), knots, cp, degree);
e1_dd  = generate_spline_dd( constant(0.1), knots, cp, degree);
e2    = generate_spline( constant(1.2), knots, cp, degree);
e2_d  = generate_spline_d( constant(1.2), knots, cp, degree);
e2_dd  = generate_spline_dd( constant(1.2), knots, cp, degree);
e3    = generate_spline( constant(3.3), knots, cp, degree);
e3_d  = generate_spline_d( constant(3.3), knots, cp, degree);
e3_dd  = generate_spline_dd( constant(3.3), knots, cp, degree);
--]]
--
--e = generate_spline(ctx, time, "fv",constant(0), constant(1), knots, degree)
--e1 = generate_spline(ctx, constant(0.1), "fv",constant(0), constant(1), knots, degree)
--e2 = generate_spline(ctx, constant(0.2), "fv",constant(0), constant(1), knots, degree)
--e3 = generate_spline(ctx, constant(0.3), "fv",constant(0), constant(1), knots, degree)

