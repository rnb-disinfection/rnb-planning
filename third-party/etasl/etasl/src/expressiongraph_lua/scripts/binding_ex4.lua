require("expressiongraph")

-- demonstration of expression graphs - basic functionality:


-- you use constant if you want to translate a constant value to a expression graph
-- This can be doubles but also other KDL types such as Vector, Frame, Rotation.
print("an expression graph for a constant double value : ")
a = constant(1.0)
print(a)

-- This is _not_ useful for configuration of VKC's: 
print("its value : ")
print(a:value())
print("its derivative towards variable 1 : ")
print(a:derivative(1))

-- you can write it to a .dot file:
a:write_dotfile("t.dot")

-- input(i) defines variable i, the derivative of input(i) towards i is 1.0.
-- you typically do NOT use input(i) in the configuration of VKC's.

-- define b in function of variable 1, using the input(...) function
b = constant(2.0) * input(1) + constant(3.0)
-- you should always call setInputValue, value and derivative in this fixed order !
-- setInputValue sets the value for variable 1 to the value 3.0
b:setInputValue(1,3)
print("value : ")
print(b:value())
print("derivative towards 1 : ")
print(b:derivative(1))


-- efficiency can be improved by using cached for common subexpressions
-- you typically assign the result of cached to a variable that you reuse
-- multiple times later on.
--
-- note also that all common operations and functions on doubles are supported.
-- e.g.: +, -, *, /, sin, cos, tan, asin, acos, atan2, log, exp, sqrt, abs
--
b = cached( constant(3.0)*input(1) + constant(1.0) )
a1 = sin(b)
print(a1)
a2 = cos(b)
print(a2)


-- a few special purpose functions:

-- if the first argument >= 0, then it returns the 2nd argument, otherwise it returns the 3th argument :
c = conditional( input(1)-constant(1.0), constant(2)*input(1), constant(3)*input(1)-constant(1.0) )
print("conditional : ")
print(c)

-- near_zero:
-- if abs(first argument) < 2nd value, return 3th argument otherwise return 4th argument
-- 1st,3th and 4th argument are expression graphs
-- 2nd argument is a value
a = input(1)*input(1)
e = near_zero( a, 1E-10, constant(0.0), a/abs(a) )
print(e)
