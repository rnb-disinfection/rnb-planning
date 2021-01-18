require("context")
require("geometric")

-- loading the UR robot with a standard script:
--
--  module that always defines the "robot_mp" 
--  and "robot_jnames" and "robot_jval" variables.
--  
--  "robot_mp" returns the robot end plate mounting frame ( the place on the
--  robot where you can mount tools and sensors on, z-axis is pointing
--  outwards)) with respect to the robot base.
--
-- "robot_jnames" returns the names of the robot joints in the order required 
--                by the driver for that robot.
-- "robot_jval"   returns expressions for these robot joints
--
require("context")
require("geometric")

-- loading a model for the unversal robot UR10:
local u=UrdfExpr();
local fn = rospack_find("etasl_py_examples").."/robots/ur10_robot.urdf"
u:readFromFile(fn)
u:addTransform("ee","ee_link","base_link")

local r = u:getExpressions(ctx)

robot_mp = r.ee * rotate_y(math.pi/2)    
robot_jname={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}

robot_jval = {}
for i=1,#robot_jname do
   robot_jval[i]   = ctx:getScalarExpr(robot_jname[i])
end

