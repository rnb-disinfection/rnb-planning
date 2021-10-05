#!/usr/bin/env python
# coding: utf-8

# In[1]:


from __future__ import print_function
import os
import sys
from test_arg_utils import *

sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "src"))

from pkg.utils.test_scripts import *
from pkg.planning.pddlstream.plan_rnb import *

rtype = "panda"
dat_root = "stowing-deep"
res_root = "stowing-deep-result"
dat_dir = "20210917-113211"
file_option = "obj_3c_obs7"
data_idx = 1
cname = "ToolReach"

GRASP_SAMPLE = 30
STABLE_SAMPLE = 50

IK_TIMEOUT_SINGLE = 0.01
IK_TRY_NUM = 10
TIMEOUT_MOTION = 3
MAX_TIME = 100

MAX_ITER = 100
MAX_SKELETONS = 30
SEARCH_SAMPLE_RATIO = 100

USE_MOVEIT_IK = True
TIMED_COMPLETE = False

VERBOSE = True
VISUALIZE = True
PLAY_RESULT = True
SHOW_STATE = False
USE_PYBULLET_GUI = False
SAVE_RESULTS = False
STACK_TIMELOG= True


CLEARANCE = 1e-3
TOOL_NAME="grip0"
ROBOT_TYPE = {e.name: e for e in RobotType}[rtype]


# In[2]:



########################################################
################### Create data folders ################
DATASET_PATH = create_data_dirs(dat_root, rtype, dat_dir)
RESULTSET_PATH = create_data_dirs(res_root, rtype, dat_dir)
print("-"*50)
print("DATASET_PATH: {}".format(DATASET_PATH))
print("RESULTSET_PATH: {}".format(RESULTSET_PATH))
print("-"*50)


########################################################
########## Load scene and prepare planner  #############
ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY, HOME_POSE, GRIP_DEPTH = get_single_robot_params(ROBOT_TYPE)
s_builder, pscene = prepare_single_robot_scene(ROBOT_TYPE, ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY, VISUALIZE=VISUALIZE)
crob, gscene = pscene.combined_robot, pscene.gscene
crob.home_pose = HOME_POSE
crob.home_dict = list2dict(HOME_POSE, gscene.joint_names)

fname = "data_%s_%02d.pkl" % (file_option, data_idx)
print(fname)
set_meta_data("dat_root", dat_root)
set_meta_data("rtype", rtype)
set_meta_data("dat_dir", dat_dir)
set_meta_data("fname", fname)


file_gtems = os.path.join(DATASET_PATH, fname)
initial_state = load_saved_scene(pscene, file_gtems, VISUALIZE=VISUALIZE)

gscene.NAME_DICT['obj_0'].color = (1,0,0,1)
gscene.update_markers_all()

mplan = MoveitPlanner(pscene)
checkers = get_checkers_by_case_name(cname, pscene)

mplan.motion_filters = checkers


# In[ ]:





# ## Edit scene

# In[3]:


for i in range(7):
    gscene.remove(gscene.NAME_DICT["obs{}".format(i)])


# In[4]:


obj0 = pscene.subject_dict['obj_0']
obj1 = pscene.subject_dict['obj_1']
obj2 = pscene.subject_dict['obj_2']

del obj0.action_points_dict["obj_0_side_g"].redundancy['x']
obj0.action_points_dict["obj_0_side_g"].redundancy['u'] = (np.pi*3/4, np.pi*5/4)
del obj1.action_points_dict["obj_1_side_g"].redundancy['x']
obj1.action_points_dict["obj_1_side_g"].redundancy['u'] = (np.pi*3/4, np.pi*5/4)
del obj2.action_points_dict["obj_2_side_g"].redundancy['x']
obj2.action_points_dict["obj_2_side_g"].redundancy['u'] = (np.pi*3/4, np.pi*5/4)

obj0.action_points_dict['bottom_p'].redundancy = {}
obj1.action_points_dict['bottom_p'].redundancy = {}
obj2.action_points_dict['bottom_p'].redundancy = {}

wpa=pscene.actor_dict['wp']
del wpa.redundancy["w"]
gpa=pscene.actor_dict['gp']
del gpa.redundancy["w"]

grip0 = pscene.actor_dict['grip0']
grip0.redundancy = {}
# grip0.redundancy = ['w'] = (-0.01,0.01)


# In[5]:


pscene.set_object_state(initial_state)
gscene.update_markers_all()

wp = gscene.NAME_DICT["wp"]
wp.set_offset_tf(center=np.add(wp.center, (0,0,0.4)))
wpa.update_handle()
wpb = gscene.NAME_DICT["wp_bot"]
wpb.set_dims(np.subtract(wpb.dims, (0.1, 0.1, 0.0)))
wpb.set_offset_tf(center=np.add(wpb.center, (0,0,0.4)))
wp_ceil = gscene.copy_from(wp, new_name="wp_ceil")
wp_ceil.set_offset_tf(center = np.add(wp.center, (0,0,0.3)))

gp = gscene.NAME_DICT["gp"]
gp.set_offset_tf(center=np.add(gp.center, (0,0,0.2)))
gpa.update_handle()
gpb = gscene.NAME_DICT["gp_bot"]
gpb.set_offset_tf(center=np.add(gpb.center, (0,0,0.2)))

gscene.update_markers_all()


# In[6]:


# pscene.remove_subject("obj_1")
# gscene.remove(gscene.NAME_DICT["obj_1"])

pscene.remove_subject("obj_2")
gscene.remove(gscene.NAME_DICT["obj_2"])


# In[7]:


# obj0.set_offset_tf(center=[0.29480234, 0.59536386, 0.1798489])
sprm = obj0.get_state_param()
T = sprm[1]
T[:3,:3] = np.identity(3)
T[:3,3] = [0.20, 0.50, 0.1798489+0.4]
obj0.set_state(obj0.binding, sprm)

sprm = obj1.get_state_param()
T = sprm[1]
T[:3,:3] = np.identity(3)
T[:3,3] = (0.15, 0.45, 0.1798489+0.4)
obj1.set_state(obj1.binding, sprm)

# sprm = obj2.get_state_param()
# T = sprm[1]
# T[:3,:3] = np.identity(3)
# T[:3,3] = (0.40, 0.25, 0.1798489+0.4)
# obj2.set_state(obj2.binding, sprm)

initial_state = pscene.initialize_state(initial_state.Q, force_fit_binding=True)
gscene.update_markers_all()


# In[8]:


GRASP_SAMPLE = 10
STABLE_SAMPLE = 20
SEARCH_SAMPLE_RATIO = 100
MAX_TIME = 300


# In[9]:


VERBOSE = True
VISUALIZE = True
PLAY_RESULT = True
SHOW_STATE = False


# # Solve in PDDLStream

# In[10]:


########################################################
#################### Solve problem  ####################
obj_pscene = pscene.subject_dict[pscene.subject_name_list[0]]
obj_pscene.geometry.color = (0.8, 0.2, 0.2, 1)
gscene.update_markers_all()
goal_pairs=[("obj_0", 'gp')]
# goal_pairs=[("obj_1", 'gp'), ("obj_0", 'gp')]

gtimer = GlobalTimer.instance()
gtimer.reset(stack=STACK_TIMELOG)


res, plan, log_dict = solve_in_pddlstream(pscene, mplan, ROBOT_NAME, TOOL_NAME, HOME_POSE, goal_pairs,
                                          TIMEOUT_MOTION, MAX_TIME, MAX_ITER, MAX_SKELETONS,
                                          GRASP_SAMPLE, STABLE_SAMPLE, SHOW_STATE, SEARCH_SAMPLE_RATIO,
                                          use_pybullet_gui=USE_PYBULLET_GUI, USE_MOVEIT_IK=USE_MOVEIT_IK, 
                                          TIMED_COMPLETE=TIMED_COMPLETE,
                                          IK_TRY_NUM=IK_TRY_NUM, IK_TIMEOUT_SINGLE=IK_TIMEOUT_SINGLE, VERBOSE=VERBOSE)

log_dict.update(mplan.result_log)
log_dict.update(GlobalLogger.instance())


# In[11]:


if VISUALIZE and PLAY_RESULT and res:
    play_pddl_plan(pscene, pscene.actor_dict["grip0"], initial_state=initial_state,
                   body_names=log_dict['body_names'], plan=plan, SHOW_PERIOD=0.01)


# In[ ]:


print({k:v for k,v in log_dict['body_names'].items() if "obj" in v})
print({k:v for k,v in log_dict['body_names'].items() if v.endswith('p')})


# In[ ]:


for k, v in pscene.actor_dict.items():
    print("{}:{}-{}".format(k, v.name, v.geometry.name))


# In[ ]:





# In[ ]:





# ## Old version

# In[8]:


from pkg.planning.pipeline import PlanningPipeline
ppline = PlanningPipeline(pscene)

from pkg.ui.ui_broker import *

# start UI
ui_broker = UIBroker.instance()
ui_broker.initialize(ppline, s_builder)
ui_broker.start_server()

ui_broker.set_tables()

from pkg.planning.motion.moveit.moveit_planner import MoveitPlanner
from pkg.planning.task.rrt import TaskRRT
tplan = TaskRRT(pscene)


# In[9]:


ppline.set_motion_planner(mplan)
ppline.set_task_planner(tplan)


# In[10]:


from_state = initial_state.copy(pscene)
from_state.Q = HOME_POSE
goal_nodes = [('gp','wp','wp')]
ppline.search(from_state, goal_nodes, verbose=False, display=False, 
              timeout_loop=100, multiprocess=True, timeout=1, add_homing=True)


# In[11]:


snode_schedule = ppline.tplan.get_best_schedule()


# In[12]:


ppline.play_schedule(snode_schedule)


# In[ ]:





# In[ ]:




