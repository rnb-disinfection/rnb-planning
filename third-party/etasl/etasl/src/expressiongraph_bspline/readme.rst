expressiongraph_bspline:  BSpline functionality in eTaSL
========================================================

License
--------

GNU LGPL v3, see LICENSE file.

Introduction
-------------

This package offers B-spline functionality in eTaSL.  B-splines
are spline basis-functions with compact support and can be used
to:

    - specify trajectories using Bezier curves or bsplines;
    - describe the variations on top of a given trajectory;
    - formulate optimization problems over a time horizon.
   

How to use
-----------

::

    bs=require("bspline")
     
    >> bs
    {
        augknots = function, bspline_basis = function, bspline_expr = function, 
        contents = function, control_points = function, derivknots = function, 
        generate_cp = function, generate_spline = function, 
        generate_spline_d = function, generate_spline_dd = function, 
        help = function, linspace = function, number_of_basisfunctions = function
    }

You can call help() to obtain more information:
::

    bs.help()


Here you find the different functions defined in the B-spline library:

::

    bspline library
    ----------------


    augknots:



        function augknots( knots, degree)

        INPUTS:
            knots : a table representing the knot vector
            degree: the degree (and multiplicity of the the first and last element
                    of the result).
        OUTPUTS:
            returns an augmented knot vector, i.e. a knot vector with the _exact_
            multiplicity of the first and last element equal to degree.
        




    bspline_expr:


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
                    #cp == #knots + degree -1 

             d:     the degree of the B-spline (should be >=0)

        Returns:
           expression that evaluates to the spline.

        Note:
           The tables with numbers should start at index 1.

        




    linspace:


        function linspace(start,stop,N)

        INPUTS:
            start :  start value
            stop  :  stop value 
            N     :  number of values
        OUTPUTS:
            return a table representing an array of N numbers, equally spaced
            from start to stop, including the start and stop values.
        




    control_points:


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

        




    generate_cp:


        function generate_cp( context, base_name, knots, degree )

        Creates feature variables for the control points for a spline

        INPUT:
            context:   context in which the variables will be defined.
            base_name: the variables will be the base_name appended with a sequence number
            knots:     knot vector of the spline
            degree:    degree of the spline
        OUTPUT:
            a table containing the feature variable expressions
        




    number_of_basisfunctions:


            function number_of_basisfunctions(knots,degree)
            
            INPUTS:
                knots: table representing the knot vector
                degree: degree of the spline
            OUTPUTS:
                the number of basisfunctions that are needed 
        




    generate_spline_dd:


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
        




    generate_spline_d:


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
        




    generate_spline:


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
        




    bspline_basis:


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

        




    derivknots:


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
        



Examples
---------



Author
------

Erwin Aertbeliën

