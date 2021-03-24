require("context")
require("geometric")
require("libexpressiongraph_collision")

u=UrdfExpr(5.0, 1.0)
u:readFromFile(rospack_find("expressiongraph_collision").."/robots/flexfellow_with_polyhedrontool.urdf")
-- robot_base_link to have it wrt robot
u:addTransform("ee","tool","robot_base_link")
u:addTransform("L7","lbr_iiwa_link_7","robot_base_link")
u:addTransform("L6","lbr_iiwa_link_6","robot_base_link")
u:addTransform("L5","lbr_iiwa_link_5","robot_base_link")
u:addTransform("L4","lbr_iiwa_link_4","robot_base_link")
u:addTransform("L3","lbr_iiwa_link_3","robot_base_link")
u:addTransform("L2","lbr_iiwa_link_2","robot_base_link")
u:addTransform("L1","lbr_iiwa_link_1","robot_base_link")
u:addTransform("L0","lbr_iiwa_link_0","robot_base_link")
u:addTransform("elbow","lbr_iiwa_link_3","robot_base_link")
robot=u:getExpressions(ctx)

robot_ee = robot.ee
robot_joints=u:getAllJointNames()

-- collision avoidance and environment model:

obj_pose = {}
obj      = {}

obj_pose[0] = cached(robot.L0*translate_z(0.0513881))
obj[0]      = CapsuleZ(0.150131,0.0923023)


obj_pose[1] = cached(robot.L1*translate_z(0.129502))
obj[1]      = CapsuleZ(0.115686,0.104536)

obj_pose[2] = cached(robot.L2*translate_y(0.0433212))
obj[2]      = CapsuleY(0.132107,0.0968504)

obj_pose[3] = cached(robot.L3*translate_z(0.148325))
obj[3]      = CapsuleZ(0.114593,0.112177)

obj_pose[4] = cached(robot.L4*translate_y(0.0526238))
obj[4]      = CapsuleY(0.114579,0.0801615)

obj_pose[5] = cached(robot.L5*translate_z(0.137758))
obj[5]      = CapsuleZ(0.099092,0.131361)

obj_pose[6] = cached(robot.L6*translate_y(0.00279075))
obj[6]      = CapsuleY(0.0795325,0.0358266)

obj_pose[7] = cached(robot.L7*translate_z(0.00385919))
obj[7]      = CapsuleZ(0.0519443,0.00678039)

obj_pose[8] =  cached(translate_x(-0.26)*translate_z(-0.45))   -- table at the base
obj[8]      =  Box(0.9,0.64,0.9)

obj_pose[9] = cached(translate_x(-0.61)*translate_y(0.24)*translate_z(0.96/2)) -- pole
obj[9]      = CapsuleZ(0.03/2, 0.96); 

obj_pose[10] = cached(robot_ee*translate_z(0.08) )             -- tool
toolname=rospack_find("expressiongraph_collision").."/meshes/polyhedron.obj"
obj[10]      = ConvexObject(toolname)

check = { {0,4},{0,5},{0,6},{0,7},
          {8,3},{8,4},{8,5},{8,7},
          {9,4},{9,5},{9,6},{9,7},
          {10,9},{10,8}, {10,0},{10,1}
        }

--check = { {10,8} }
--check = {} 
print("collision checked between ".. #check .. " convex objects");
for i,v in pairs(check) do
    local d = distance_between( obj_pose[v[1] ], obj[v[1] ] , 0.0025, 0.00,  obj_pose[v[2] ], obj[v[2] ], 0.0025,0.00)
    ctx:setOutputExpression("d"..i,d);    
    Constraint{
        context         = ctx,
        name            = "collision_"..i,
        expr            = d,
        target_lower    = 0,
        weight          = 1,
        priority        = 1,
        K               = 5
    }

end

