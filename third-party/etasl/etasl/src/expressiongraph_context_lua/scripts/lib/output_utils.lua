-- some utilities to handle output of etasl variables.
--
--
if ilua~=nil then
    print( [[ 
output_utils.lua called from ilua, printing most important functions :
    output_all_scalar_expressions, output_all_vector_expressions, output_all_rotation_expressions,
    output_all_frame_expressions,  output_all_twist_expressions,  output_all_expressions
    
    output_table_expressions, output_table_vector_expressions, output_table_rotation_expressions,
    output_table_frame_expressions,  output_table_twist_expressions,  output_table_expressions

    output_recursive_expressions

    the function without arguments gives a help message
]])
end




function output_all_scalar_expressions(ctx)
    if extendedtype(ctx)~="Context" then
        hlp=[[
            function output_all_scalar_expressions(ctx)

            declares all scalar expressions in the global lua context as an output expressions

            INPUT:
                ctx:   context
        ]]
        error("requires 1 argument of type 'Context'\n\n"..hlp, 2)
    end
    for k,v in pairs(_G) do
        if extendedtype(v)=="expression_double" then
            ctx:setOutputExpression("_"..k, v)
        end
    end
end

function output_all_vector_expressions(ctx)
    if extendedtype(ctx)~="Context" then
        hlp=[[
            function output_all_vector_expressions(ctx)

            declares all vector expressions in the global lua context as an output expressions

            INPUT:
                ctx:   context
        ]]
        error("requires 1 argument of type 'Context'\n\n"..hlp, 2)
    end
    for k,v in pairs(_G) do
        if extendedtype(v)=="expression_vector" then
            ctx:setOutputExpression("_"..k, v)
        end
    end
end


function output_all_rotation_expressions(ctx)
    if extendedtype(ctx)~="Context" then
        hlp=[[
            function output_all_rotation_expressions(ctx)

            declares all rotation expressions in the global lua context as an output expressions

            INPUT:
                ctx:   context
        ]]
        error("requires 1 argument of type 'Context'\n\n"..hlp, 2)
    end
    for k,v in pairs(_G) do
        if extendedtype(v)=="expression_rotation" then
            ctx:setOutputExpression("_"..k, v)
        end
    end
end

function output_all_frame_expressions(ctx)
    if extendedtype(ctx)~="Context" then
        hlp=[[
            function output_all_frame_expressions(ctx)

            declares all frame expressions in the global lua context as an output expressions

            INPUT:
                ctx:   context
        ]]
        error("requires 1 argument of type 'Context'\n\n"..hlp, 2)
    end
    for k,v in pairs(_G) do
        if extendedtype(v)=="expression_frame" then
            ctx:setOutputExpression("_"..k, v)
        end
    end
end


function output_all_twist_expressions(ctx)
    if extendedtype(ctx)~="Context" then
        hlp=[[
            function output_all_twist_expressions(ctx)

            declares all twist expressions in the global lua context as an output expressions

            INPUT:
                ctx:   context
        ]]
        error("requires 1 argument of type 'Context'\n\n"..hlp, 2)
    end
    for k,v in pairs(_G) do
        if extendedtype(v)=="expression_twist" then
            ctx:setOutputExpression("_"..k, v)
        end
    end
end


function output_all_wrench_expressions(ctx)
    if extendedtype(ctx)~="Context" then
        hlp=[[
            function output_all_wrench_expressions(ctx)

            declares all wrench expressions in the global lua context as an output expressions

            INPUT:
                ctx:   context
        ]]
        error("requires 1 argument of type 'Context'\n\n"..hlp, 2)
    end
    for k,v in pairs(_G) do
        if extendedtype(v)=="expression_wrench" then
            ctx:setOutputExpression("_"..k, v)
        end
    end
end


function output_all_expressions(ctx)
    output_all_scalar_expressions(ctx)
    output_all_vector_expressions(ctx)
    output_all_rotation_expressions(ctx)
    output_all_frame_expressions(ctx)
    output_all_twist_expressions(ctx)
    output_all_wrench_expressions(ctx)
end

function output_table_scalar_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" or extendedtype(t)~="table" then
        hlp=[[
            function output_table_scalar_expressions(ctx,t,prefix)

            declares all scalar expressions in the table as an output expressions

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    for k,v in pairs(t) do
        if extendedtype(v)=="expression_double" then
            ctx:setOutputExpression(n..k, v)
        end
    end
end

function output_table_vector_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" or extendedtype(t)~="table" then
        hlp=[[
            function output_table_vector_expressions(ctx,t,prefix)

            declares all vector expressions in the table as an output expressions

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    for k,v in pairs(t) do
        if extendedtype(v)=="expression_vector" then
            ctx:setOutputExpression(n..k, v)
        end
    end
end


function output_table_rotation_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" or extendedtype(t)~="table" then
        hlp=[[
            function output_table_rotation_expressions(ctx,t,prefix)

            declares all rotation expressions in the table context as an output expressions

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    for k,v in pairs(t) do
        if extendedtype(v)=="expression_rotation" then
            ctx:setOutputExpression(n..k, v)
        end
    end
end

function output_table_frame_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" or extendedtype(t)~="table" then
        hlp=[[
            function output_table_frame_expressions(ctx,t,prefix)

            declares all frame expressions in the table as an output expressions

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    for k,v in pairs(t) do
        if extendedtype(v)=="expression_frame" then
            ctx:setOutputExpression(n..k, v)
        end
    end
end


function output_table_twist_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" or extendedtype(t)~="table" then
        hlp=[[
            function output_table_twist_expressions(ctx,t,prefix)

            declares all twist expressions in the table as an output expressions

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    for k,v in pairs(t) do
        if extendedtype(v)=="expression_twist" then
            ctx:setOutputExpression(n..k, v)
        end
    end
end


function output_table_wrench_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" or extendedtype(t)~="table" then
        hlp=[[
            function output_table_wrench_expressions(ctx,t,prefix)

            declares all wrench expressions in the table as an output expressions

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    for k,v in pairs(t) do
        if extendedtype(v)=="expression_wrench" then
            ctx:setOutputExpression(n..k, v)
        end
    end
end


function output_table_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" or extendedtype(t)~="table" then
        hlp=[[
            function output_table_expressions(ctx,t,prefix)

            declares all expressions in the table as an output expressions
                (shallow, does not go recursively deep into the datastructure)

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    output_table_scalar_expressions(ctx,t,n)
    output_table_vector_expressions(ctx,t,n)
    output_table_rotation_expressions(ctx,t,n)
    output_table_frame_expressions(ctx,t,n)
    output_table_twist_expressions(ctx,t,n)
    output_table_wrench_expressions(ctx,t,n)
 
end

function output_recursive_expressions(ctx,t,n)
    if extendedtype(ctx)~="Context" then
        hlp=[[
            function output_recursive_expressions(ctx,t,prefix)

            recursively goes through the data structure and declare outputs for all expressions

            INPUT:
                ctx:   context
                t  :   table
                prefix: prefixstring [optional]
                separator: string [optional, default "_"]
        ]]
        error("requires 2 or 3 arguments of type 'Context'\n\n"..hlp, 2)
    end
    if (n==nil) or extendedtype(n)~="string" then
        n=""
    end
    if (separator==nil) or extendedtype(separator)~="string" then
        separator = "_"
    end
    output_table_scalar_expressions(ctx,t,n)
    output_table_vector_expressions(ctx,t,n)
    output_table_rotation_expressions(ctx,t,n)
    output_table_frame_expressions(ctx,t,n)
    output_table_twist_expressions(ctx,t,n)
    output_table_wrench_expressions(ctx,t,n)
    for k,v in pairs(t) do
        if extendedtype(v)=="table" then
            output_recursive_expressions(ctx,v,n..tostring(k)..separator)
       end
    end
end


