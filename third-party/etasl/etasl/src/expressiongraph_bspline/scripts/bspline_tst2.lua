require("context")
require("bspline")

time   = input(1)
knots  = {0,0,0,0.5,1,1,1}
degree = 3
N      = #knots - degree +1

-- demonstrates the uses of constant folding optimization of expression graphs
-- if an expression is used in another expression, it is checked whether the 
-- expression can be replaced with a constant or not.
f = bspline_basis(constant(0.2), knots,2,degree);

print(f)
f2=cached(f)
print("and now constant folding was applied :")
print(f2)


-- or directly specifying all the control points:
-- somewhat confusing because lua has 1-based arrays, such that the numbering
-- of the basis functions start at 1 instead of 0.
-- So basis function 2 becomes index 3 here.
cp={}
for i=1,#knots-degree+1 do
    cp[i] = 0.0 
end
cp[3]=1


f3 = bspline_expr(constant(0.2), knots,cp,degree);
print(f3)
f4 = cached(f3);
print(f4)

d_knots, d_cp = derivknots(knots,cp,degree)


