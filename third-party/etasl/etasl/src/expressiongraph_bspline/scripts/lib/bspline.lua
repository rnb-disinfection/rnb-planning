require("libexpressiongraph_bspline")
require("context")

local _bspline_basis = bspline_basis
local _bspline_expr = bspline_expr

local function control_points( knots, j , degree )
    local msg=[[
    function control_points( knots, j, d)


    Inputs 
         knots: a table with numbers containing the knot vector. 

         j:     compute the j-th basis function.(1...N)

         d:     the degree of the B-spline (should be >=0)

    Returns:
        
        a control points vector representing base function j
        (all 0.0 except for element j, that is equal to 1.0)
    Note:
       The tables with numbers should start at index 1.

    ]]
    namedcheck({"knots","j","degree"},
               {knots,j,degree},
               {"table","number","number"},
               msg);
    local cp = {}
    for i=1, #knots - degree + 1 do
        cp[i] = 0.0
    end
    cp[j] = 1.0
    return cp
end



local function number_of_basisfunctions(knots,degree)
    local msg=[[
        function number_of_basisfunctions(knots,degree)
        
        INPUTS:
            knots: table representing the knot vector
            degree: degree of the spline
        OUTPUTS:
            the number of basisfunctions that are needed 
    ]]
    namedcheck({"knots","degree"}, {knots, degree}, {"table","number"}, msg)
    return #knots-degree+1
end

local function augknots( knots, degree)
    local msg=[[

    function augknots( knots, degree)

    INPUTS:
        knots : a table representing the knot vector
        degree: the degree (and multiplicity of the the first and last element
                of the result).
    OUTPUTS:
        returns an augmented knot vector, i.e. a knot vector with the _exact_
        multiplicity of the first and last element equal to degree.
    ]]
    namedcheck({"knots","degree"}, {knots, degree}, {"table","number"}, msg)

    if #knots <= 1 then
        error("knots vector to small to compute augmented knots vector",2)
    end
    local startp=1
    local el = knots[startp]
    for i=startp+1,#knots do
        if knots[i]~=el then
            startp = i-1
            break
        end
        el = knots[i]
    end
    el = knots[#knots]
    local endp = #knots
    for i=#knots-1,1,-1 do
        if knots[i]~=el then
            endp = i+1
            break
        end
        el = knots[i]
    end
    if endp-startp < 1 then
        error("knots vector is invalid",2)
    end
    if degree==0 then
        local result = {}
        for i=startp,endp,1 do
            table.insert(result, knots[i]);
        end
        return result
    end
    local result = {}
    for i=1,degree-1,1 do
        table.insert(result, knots[startp])
    end
    for i=startp,endp,1 do
        table.insert(result, knots[i])
    end
    for i=1,degree-1 , 1 do
        table.insert(result, knots[endp])
    end
    return result
end




local function bspline_basis( x, knots, j, d)
    local msg=[[
    function bspline_basis( x, knots, j, d)

        Returns an expression that evaluates a B-Spline

    Inputs 
         x:     a double expression, the B-spline will be evaluated at this point.

         knots: a table with numbers containing the knot vector.  Should be increasing.  
                Knots can have a multiplicity up to "degree".  The spline will be continuous 
                up to derivative (degree-multiplicity).
                It should start and end with knots of multiplicity "degree" (called an augmented 
                knot vector in B-spline literature).

         j:     compute the j-th basis function.

         d:     the degree of the B-spline (should be >=0)

    Returns:
       expression that evaluates to the spline.

    Note:
       The tables with numbers should start at index 1.

    ]]
    namedcheck({"x","knots","j","d"},
               {x,knots,j,d},
               {"expression_double","table","number","number"},
               msg);

    return _bspline_basis(x,knots,j,d)
end

function bspline_expr( x, knots, cp, d)
    local msg=[[
    function bspline_expr( x, knots, cp, d)

        Returns an expression that evaluates a B-Spline

    Inputs 
         x:     a double expression, the B-spline will be evaluated at this point.

         knots: a table with numbers containing the knot vector.  Should be increasing.  
                Knots can have a multiplicity up to "degree".  The spline will be continuous 
                up to derivative (degree-multiplicity).
                It should start and end with knots of multiplicity "degree" (called an augmented 
                knot vector in B-spline literature).

         cp:    a table with numbers containing the control points.  
                #cp == #knots - degree +1 

         d:     the degree of the B-spline (should be >=0)

    Returns:
       expression that evaluates to the spline.

    Note:
       The tables with numbers should start at index 1.

    ]]
    namedcheck({"x","knots","cp","d"},
               {x,knots,cp,d},
               {"expression_double","table","table","number"},
               msg);

    return _bspline_expr(x,knots,cp,d)
end


local function derivknots( knots, cp, degree)
    local msg=[[
    function derivknots( knots, cp, degree)

    Inputs 

         knots:   a table with numbers containing the knot vector. 
         cp:      a table with numbers containing the control points.  
                  #cp == #knots + degree -1 
         degree:  the degree of the B-spline (should be >=0)

    Returns:
        d_knots : the knot vector for the derivative of the spline in the arguments.
        d_cp    : the control points for the derivatie of the spline in the arguments
        d_degree: 
    Note:
       The tables with numbers start at index 1.
    ]]
    namedcheck({"knots","cp","degree"},
               {knots,cp,degree},
               {"table","table","number"},
               msg);

    local d_knots
    local d_cp
    if degree>1 then
        d_knots={}
        d_cp={}
        for i=1, #cp-1 do
            d_cp[i] = degree*( cp[i+1]-cp[i] ) / (knots[i+degree]-knots[i])
        end
        for i=1,#knots-2 do
            d_knots[i] = knots[i+1]
        end
    else
        d_knots={}
        d_cp={}
        for i=1,#cp-1 do
            d_cp[i] = degree*( cp[i+1]-cp[i] ) / (knots[i+degree]-knots[i])
        end
        for i=1,#knots-1 do
            d_knots[i] = knots[i+1]
        end
    end
    return d_knots, d_cp, degree-1
end


local function generate_cp( context, base_name, knots, degree )
    local msg = [[
    function generate_cp( context, base_name, knots, degree )

    Creates feature variables for the control points for a spline

    INPUT:
        context:   context in which the variables will be defined.
        base_name: the variables will be the base_name appended with a sequence number
        knots:     knot vector of the spline
        degree:    degree of the spline
    OUTPUT:
        a table containing the feature variable expressions
    ]]
    namedcheck({"context","base_name","knots","degree"},
               {context, base_name, knots,degree},
               {"Context","string","table","number"},
               msg);


    local N = #knots - degree  + 1
    local cp = {}
    for i=1,N do
        local vn = base_name..i
        cp[i] = Variable{ context=context,name=vn,vartype="feature" };
    end
    return cp
end


local function generate_spline( u, knots, cp, degree)
    local msg=[[
    function generate_spline( u, knots, cp, degree)

    uses the knot vector and the control point expressions in cp
    to create a spline with the given degree.

    INPUTS
        u:            return a spline in function of the expression u
        knots:        knot vector of the spline
        cp:           a table with the control points (double expression)
        degree:       degree of the spline
    OUTPUTS
        an expression for the given spline
    ]]
    namedcheck({"u","knots","cp","degree"},
               {u, knots, cp,degree},
               {"expression_double","table","table","number"},
               msg);
    local expr = conditional(u-constant(knots[#knots]), cp[#cp], constant(0.0))
                 + conditional(u-constant(knots[1]), constant(0.0), cp[1] )
    for i = 1, #cp do
        expr = expr + cp[i]*bspline_basis(u, knots, i-1, degree)
    end
    return cached(expr)
end

local function generate_spline_d( u, knots, cp, degree)
    local msg=[[
    function generate_spline_d( u, knots, cp, degree)

    uses the knot vector and the control point expressions in cp
    to create an expression for the derivative of a spline with the 
    given degree.

    INPUTS
        u:            return a spline in function of the expression u
        knots:        knot vector of the spline
        cp:           a table with the control points (double expression)
        degree:       degree of the spline
    OUTPUTS
        an expression for the derivative of the given spline specified by
        knots, cp and degree.
    ]]
    namedcheck({"u","knots","cp","degree"},
               {u, knots, cp,degree},
               {"expression_double","table","table","number"},
               msg);

    local expr = constant(0)
    local d_knots
    local d_basis_cp
    local d_degree
    for i = 1, #cp do
        local basis_cp = control_points( knots, i, degree)
        d_knots, d_basis_cp, d_degree = derivknots( knots, basis_cp, degree)
        expr = expr + cp[i]*bspline_expr(u, d_knots, d_basis_cp, d_degree)
    end
    return cached(expr)
end

local function generate_spline_dd( u, knots, cp, degree)
    local msg=[[
    function generate_spline_dd( u, knots, cp, degree)

    uses the knot vector and the control point expressions in cp
    to create an expression for the 2nd derivative of a spline with the 
    given degree.

    INPUTS
        u:            return a spline in function of the expression u
        knots:        knot vector of the spline
        cp:           a table with the control points (double expression)
        degree:       degree of the spline
    OUTPUTS
        an expression for the 2nd derivative of the given spline specified by
        knots, cp and degree.
    ]]
    namedcheck({"u","knots","cp","degree"},
               {u, knots, cp,degree},
               {"expression_double","table","table","number"},
               msg);
    local expr = constant(0)
    local d_knots
    local dd_knots
    local d_basis_cp
    local dd_basis_cp
    local d_degree
    local dd_degree
    for i = 1, #cp do
        local basis_cp = control_points( knots, i, degree)
        d_knots, d_basis_cp, d_degree = derivknots( knots, basis_cp, degree)
        dd_knots, dd_basis_cp, dd_degree = derivknots( d_knots, d_basis_cp, d_degree)
        expr = expr + cp[i]*bspline_expr(u, dd_knots, dd_basis_cp, dd_degree)
    end
    return cached(expr)
end


local function linspace(start,stop,N)
    local msg=[[
    function linspace(start,stop,N)

    INPUTS:
        start :  start value
        stop  :  stop value 
        N     :  number of values
    OUTPUTS:
        return a table representing an array of N numbers, equally spaced
        from start to stop, including the start and stop values.
    ]]
    namedcheck({"start","stop","N"},
               {start, stop, N},
               {"number","number","number"},
               msg);
    local r={}
    for i=0,N-1 do
        table.insert(r, start + i * (stop-start)/(N-1) )
    end
    return r
end


local ftable={
    control_points              = control_points,
    number_of_basisfunctions    = number_of_basisfunctions,
    augknots                    = augknots,
    bspline_basis               = bspline_basis,
    bspline_expr                = bspline_expr,
    derivknots                  = derivknots,
    generate_cp                 = generate_cp,
    generate_spline             = generate_spline,
    generate_spline_d           = generate_spline_d,
    generate_spline_dd          = generate_spline_dd,
    linspace                    = linspace
}

ftable['contents'] = _contents(ftable,'bspline')
ftable['help'] = _help(ftable,'bspline')

return ftable
