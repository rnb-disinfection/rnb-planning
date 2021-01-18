require("expressiongraph")
function getRPY( arg )
    local v = cached(getRPY_raw(arg))
    return coord_x(v), coord_y(v), coord_z(v)
end

R=rot_z(input(3))*rot_y(input(2))*rot_x(input(1))

R:setInputValue(1,0)
R:setInputValue(2,0)
R:setInputValue(3,0)
r,p,y = getRPY(R)
print("r "..r:value().." derivatives: "..r:derivative(1).." "..r:derivative(2).." "..r:derivative(3))
print("p "..p:value().." derivatives: "..p:derivative(1).." "..p:derivative(2).." "..p:derivative(3))
print("y "..y:value().." derivatives: "..y:derivative(1).." "..y:derivative(2).." "..y:derivative(3))


R:setInputValue(1,0.1)
R:setInputValue(2,0.3)
R:setInputValue(3,0.1)
r,p,y = getRPY(R)
print("r "..r:value())
print(" derivatives: "..r:derivative(1).." "..r:derivative(2).." "..r:derivative(3))
print("p "..p:value())
print(" derivatives: "..p:derivative(1).." "..p:derivative(2).." "..p:derivative(3))
print("y "..y:value())
print(" derivatives: "..y:derivative(1).." "..y:derivative(2).." "..y:derivative(3))


