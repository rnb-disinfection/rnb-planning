#!/usr/bin/env python
# coding: utf-8

# In[1]:


import os
import sys
sys.path.append(os.path.join(os.environ["TAMP_ETASL_DIR"], "moveit_plan_compact"))
import moveit_plan_compact as mpc
import numpy as np
from enum import Enum

class ObjectType(Enum):
    BOX = 1
    SPHERE = 2
    CYLINDER = 3
    
def make_assign_arr(type, vals):
    arr = type()
    for v in vals:
        arr.append(v)
    return arr

def make_assign_vec(type, vals, dim):
    arr = type()
    for i, v in zip(range(dim), vals):
        arr[i]=v
    return arr
    
def CartPose(*vals):
    return make_assign_vec(mpc.CartPose, vals, 7)
        
def Vec3(*vals):
    return make_assign_vec(mpc.Vec3, vals, 3)
        
def JointState(dim, *vals):
    return make_assign_vec(lambda: mpc.JointState(dim), vals, dim)
    
def NameList(*vals):
    return make_assign_arr(mpc.NameList, vals)

def spread(bp_arr, size):
    return [bp_arr[i] for i in range(size)]

class ObjectMPC:
    def __init__(self, name, type, pose, dims, link_name):
        self.name, self.type, self.pose, self.dims, self.link_name = name, type, pose, dims, link_name
        
class MoveitCompactPlanner(mpc.Planner):
    def __init__(self, urdf_path, srdf_path, group_names):
        mpc.Planner.__init__(self)
        self.urdf_path, self.srdf_path = urdf_path, srdf_path
        self.group_names_py = group_names
        self.group_names = NameList(*group_names)
        if not self.init_planner_from_file(urdf_path, srdf_path, self.group_names):
            raise(RuntimeError("Failed to initialize planner"))
        self.joint_names_py = spread(self.joint_names, self.joint_num)
        
    def set_scene(self, obj_list):
        self.clear_all_objects()
        for obj in obj_list:
            self.add_object(obj.name, obj.type.value, CartPose(*obj.pose), Vec3(*obj.dims), obj.link_name)
            
    def plan_py(self, robot_name, tool_link, goal_pose, goal_link, Q_init, timeout=0.1):
        gtimer.tic("plan")
        plan = self.plan(robot_name, tool_link, CartPose(*goal_pose), goal_link,
                         JointState(self.joint_num, *Q_init), timeout)
        gtimer.toc("plan")
        return np.array([spread(Q, self.joint_num) for Q in spread(plan.trajectory, len(plan.trajectory))]), plan.success


# In[2]:


from pkg.utils.utils import *


# In[3]:


urdf_path, srdf_path = './moveit_plan_compact/test_assets/custom_robots.urdf', './moveit_plan_compact/test_assets/custom_robots.srdf'


# In[4]:


planner = MoveitCompactPlanner(urdf_path, srdf_path, ["indy0", "indy0_tcp"])


# In[5]:


planner.set_scene([ObjectMPC("box", ObjectType.BOX, (-0.3,-0.2,0.0,0,0,0,1), (0.1,0.1,0.), "base_link")])


# In[7]:


gtimer = GlobalTimer.instance()
gtimer.tic("plan_py")
trajectory, success = planner.plan_py("indy0", "indy0_tcp", (-0.3,-0.2,0.4,0,0,0,1), "base_link",
                                      (0, 0, -1.57, 0, -1.57, 0, 0, -0.4, 0, -1.57, 0, 1.57, 1.57), 0.1)
gtimer.toc("plan_py")
print(gtimer)
print(success)


# In[ ]:




