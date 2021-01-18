require("context")
require("bspline")

time   = input(1)
knots  = {0,0,0,0.5,1,1,1}
degree = 3
N      = #knots - degree +1

print("import matplotlib.pyplot as plt")
print("import numpy as np")

fn = {}
for i=0,N-1 do
    fn[i] = bspline_basis(time, knots, i, degree)
end

print("result=np.array([")
first = true
-- the spline is defined in the half-open interval [0,1(, at point 1 all basis functions are zero.
for tau=0,1-1E-7,0.01 do
    if first then
        print("[" .. tau)
        first=false
    else
        print(",[" .. tau)
    end
    for i=0,N-1 do
        fn[i]:setInputValue(1,tau)
        print(", "..fn[i]:value())
        --print(", "..fn[i]:derivative(1))
    end
    print("]")
end
print("])");
print("dresult=np.array([")
first = true
for tau=0,1-1E-7,0.01 do
    if first then
        print("[" .. tau)
        first=false
    else
        print(",[" .. tau)
    end
    for i=0,N-1 do
        fn[i]:setInputValue(1,tau)
        fn[i]:value() -- always evaluate value() before derivative()...
        print(", "..fn[i]:derivative(1))
    end
    print("]")
end
print("])");




print("plt.subplot(2,1,1)")
print("plt.plot(result[:,0], result[:,1:])")
print("plt.legend( (")
print("'basis_fn 0'")
for i=1,N-1 do
    print(", 'basis_fn " .. i  .. "'")
end
print("), shadow=True, fancybox=True);");
print("plt.ylabel('basis fn');");
print("plt.grid(True)");

print("plt.subplot(2,1,2)")
print("plt.plot(dresult[:,0], dresult[:,1:])")
print("plt.ylabel('deriv.');");
print("plt.xlabel('tau');");

print("plt.grid(True)");
print("plt.show()")

