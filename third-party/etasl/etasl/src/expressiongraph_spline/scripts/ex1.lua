require("context")
require("libexpressiongraph_spline")
if ilua~=nil then
    print( [[
    ilua is called....
]])
end



a=CubicSpline()
a:readPoints("../notebooks/circle.csv",0)
--a:prepare()
--print( "status " .. a:getPreparedStatus() )
s = input(0)
a:setInput(s)

y1=getSplineOutput(a,0)
y2=getSplineOutput(a,1)

a:normalize(600)
write_expressions_to_dot('ex1.dot',{y1,y2})

