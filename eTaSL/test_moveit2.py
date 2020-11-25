import os
import sys
sys.path.append(os.path.join(os.environ["TAMP_ETASL_DIR"], "moveit_plan_compact"))


import moveit_plan_compact as mpc


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
        arr[dim]=v
    return arr
    
def CartPose(*vals):
    return make_assign_vec(mpc.CartPose, vals, 7)
        
def Vec3(*vals):
    return make_assign_vec(mpc.Vec3, vals, 3)
    
def NameList(*vals):
    return make_assign_arr(mpc.NameList, vals)

planner = mpc.Planner()



group_names = NameList('indy0', 'panda1')
planner.init_planner_from_file(
    './moveit_plan_compact/test_assets/custom_robots.urdf', 
    './moveit_plan_compact/test_assets/custom_robots.srdf',
    group_names
)

planner.add_object(
    "box", ObjectType.BOX.value, CartPose(-0.3,-0.2,0.0,0,0,0,1), Vec3(0.1,0.1,0.), "base_link"
)

