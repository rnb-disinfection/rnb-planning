import tesseract
import os
import re
import numpy as np
import time
from pkg.utils.utils import *
from pkg.global_config import *

import rospkg
rospack = rospkg.RosPack()
class RosResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(RosResourceLocator,self).__init__()

    def locateResource(self, url):
        fname = None
        if "package://" in url:
            url_split = url[10:].split("/")
            pkg_name = url_split[0]
            pkg_path = rospack.get_path(pkg_name)
            fname = os.path.join(pkg_path, *url_split[1:])
            with open(fname,'rb') as f:
                resource_bytes = f.read()
        else:
            return None

        resource = tesseract.BytesResource(url, resource_bytes)

        return resource

with open(os.path.join(PROJ_DIR, "robots", "custom_robots.urdf"),'r') as f:
    robot_urdf = f.read()

with open(os.path.join(PROJ_DIR, "robots", "custom_robots.srdf"),'r') as f:
    robot_srdf = f.read()

t = tesseract.Tesseract()

MANIPULATOR = "indy0"
BASELINK = "indy0_link0"
ENDLINK = "indy0_tcp"

t.init(robot_urdf, robot_srdf, RosResourceLocator())

pci = tesseract.ProblemConstructionInfo(t)

pci.init_info.type = tesseract.InitInfo.STATIONARY
# pci.init_info.data = np.array([0.0,0,0,0,0,0])

pci.basic_info.n_steps = 10
pci.basic_info.manip = MANIPULATOR
pci.basic_info.start_fixed = False
pci.basic_info.use_time = False
pci.basic_info.dt_upper_lim = 1
pci.basic_info.dt_lower_lim = 0.9999

pci.opt_info.max_iter = 200
pci.opt_info.min_approx_improve = 1e-3
pci.opt_info.min_trust_box_size = 1e-3

pci.kin = pci.getManipulator(pci.basic_info.manip)

start_pos_terminfo = tesseract.JointPosTermInfo()
start_pos_terminfo.name = "start"
start_pos_terminfo.term_type = tesseract.TT_COST
# start_pos_terminfo.coeffs=np.ones(6)
start_pos_terminfo.first_step = 0
start_pos_terminfo.last_step = 0
# start_pos_terminfo.lower_tols=np.ones(6)*-0.01
# start_pos_terminfo.upper_tols=np.ones(6)*0.01

start_pos_terminfo.targets = np.array([0.0]*6, dtype=np.float64)

end_pos_terminfo = tesseract.CartPoseTermInfo()
jsonval = \
"""
{{
    "type" : "cart_pose",
    "params": {{
      "xyz" : [0.4,0.0,0.4],
      "wxyz" : [0.0,0.0,1.0,0.0],
      "target" : "{target}",
      "link" : "{link}"
    }}
}}

""".format(target=BASELINK, link=ENDLINK)
print(jsonval)
end_pos_terminfo.fromJson(pci, jsonval)
print("succ json")
# end_pos_terminfo = tesseract.JointPosTermInfo()
# # end_pos_terminfo = tesseract.CartPoseTermInfo()
end_pos_terminfo.name = "end"
end_pos_terminfo.term_type = tesseract.TT_COST
# end_pos_terminfo.coeffs=np.ones(6)
# end_pos_terminfo.targets=np.array([0.5]*6, dtype=np.float64)
end_pos_terminfo.first_step = pci.basic_info.n_steps-1
end_pos_terminfo.last_step = pci.basic_info.n_steps-1
# #end_pos_terminfo.lower_tols=np.ones(6)*-0.01
# #end_pos_terminfo.upper_tols=np.ones(6)*0.01

joint_vel = tesseract.JointVelTermInfo()
joint_vel.coeffs = np.ones(6)
joint_vel.targets = np.zeros(6)
joint_vel.first_step = 0
joint_vel.last_step = pci.basic_info.n_steps - 1
joint_vel.name = "Joint_vel"
joint_vel.term_type = tesseract.TT_COST

time_terminfo = tesseract.TotalTimeTermInfo()
time_terminfo.coeff = 1
time_terminfo.term_type = tesseract.TT_COST | tesseract.TT_USE_TIME
time_terminfo.name = "time"

collision = tesseract.CollisionTermInfo()
collision.name = "collision"
collision.term_type = tesseract.TT_CNT
collision.continuous = True
collision.first_step = 0
collision.last_step = pci.basic_info.n_steps - 2
collision.gap = 1
collision_info = tesseract.createSafetyMarginDataVector(pci.basic_info.n_steps, .0001, 40)
for i in collision_info:
    collision.info.append(i)

pci.cost_infos.append(start_pos_terminfo)
pci.cost_infos.append(end_pos_terminfo)
# pci.cost_infos.append(time_terminfo)
pci.cnt_infos.push_back(collision)
pci.cost_infos.push_back(joint_vel)

print("construct problem")
prob = tesseract.ConstructProblem(pci)
print("construct config")
config = tesseract.TrajOptPlannerConfig(prob)

print("construct planner")
planner = tesseract.TrajOptMotionPlanner()

print("set config")
planner.setConfiguration(config)
from pkg.utils.utils import *
gtimer = GlobalTimer.instance()
gtimer.tic("solve")
planner_status_code, planner_response = planner.solve(True)
print("solve: {}".format(gtimer.toc("solve")))

assert planner_status_code.value() == 0, "Planning failed"

print(planner_response)

print(planner_response.joint_trajectory.trajectory)
