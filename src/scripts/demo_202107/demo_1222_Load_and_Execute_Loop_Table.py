#!/usr/bin/env python
# coding: utf-8

# # Demo Script for Milestone 10.15

# ## 0 Prepare task

# ### 0.1 prepare planning scene

# #### Run shared detector on bash
# ```bash
# python3 /home/kiro-ros/Projects/rnb-planning/src/scripts/demo_202107/demo_utils/shared_detector.py
# ```
# 
# #### Check and request ip setting from mobile udp client (robot-side)

# #### 0.1.1 Set parameters and create planning scene

# In[ ]:


raw_input("==================================================\n=========== Start IndySDK and press Enter ========\n")


# In[ ]:


raw_input("==================================================\n========= Connect to Conty and reset robot =======\n")
raw_input("==================================================\n========= Move robot to home pose =======\n")


# In[1]:


import numpy as np

CONNECT_INDY = True
CONNECT_MOBILE = False

CONNECT_TASK_PLANNER = False
VISUALIZE = False
VERBOSE = False
PLANNING_MULTIPROC = True
N_AGENTS = 10
NODE_TRIAL_MAX = N_AGENTS * 2
MAX_SOL_NUM = 5
BASE_COST_CUT = 110

TIMEOUT_MOTION = 0.5
TIMEOUT_FULL = 5

ROS_MASTER_ON_MOBILE = False
# Tool dimensions
TOOL_DIM = [0.24, 0.35]
TOOL_CENTER_OFF = [0, -0.035, 0]
TOOL_THICKNESS = 0.11 + 0.05

INDY_BASE_OFFSET = (0.172,0,0.439)
INDY_BASE_RPY = (0,0,0)
TOOL_NAME = "brush_face"
WALL_THICKNESS = 0.01
CLEARANCE = 0.001
WS_HEIGHT = 1.6

COL_COLOR = (1,1,1,0.2)

USE_SDK = True

IP_CUR = "192.168.0.10"# get_ip_address()
MOBILE_IP = "192.168.0.102"
INDY_IP = "192.168.0.3"

print("Current PC IP: {}".format(IP_CUR))
print("Mobile ROB IP: {}".format(MOBILE_IP))


# In[2]:


import os
import sys
sys.path.append(os.path.join(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src')))
sys.path.append(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src/scripts/demo_202107'))

from pkg.global_config import RNB_PLANNING_DIR
from pkg.utils.utils import *    
from pkg.utils.rotation_utils import *
from pkg.controller.combined_robot import *
from demo_utils.area_select import *
from pkg.detector.aruco.marker_config import get_aruco_map
aruco_map = get_aruco_map()

from pkg.detector.multiICP.multiICP import *
from pkg.detector.detector_interface import DetectionLevel
from pkg.detector.multiICP.config import *

from pkg.geometry.builder.scene_builder import SceneBuilder
from demo_utils.environment import *
from demo_utils.area_select import DATASET_DIR, SweepDirections
from demo_utils.demo_config import *
from demo_utils.detection_util import *

from pkg.utils.shared_function import *
clear_channels_on("kiromobilemap")

if not CONNECT_INDY:
    indy_7dof_client.kiro_tool.OFFLINE_MODE = True
kiro_udp_client.KIRO_UDP_OFFLINE_DEBUG = not CONNECT_MOBILE

mobile_config = RobotConfig(0, RobotType.kmb, ((0,0,0), (0,0,0)),
                "{}/{}".format(MOBILE_IP, IP_CUR))
robot_config = RobotConfig(1, RobotType.indy7kiro, 
                           (INDY_BASE_OFFSET, INDY_BASE_RPY),
                INDY_IP, root_on="kmb0_platform", 
                           specs={"no_sdk":True} if not USE_SDK else {})
ROBOT_TYPE = robot_config.type
MOBILE_NAME = mobile_config.get_indexed_name()
ROBOT_NAME = robot_config.get_indexed_name()
crob = CombinedRobot(robots_on_scene=[mobile_config, robot_config]
              , connection_list=[True, CONNECT_INDY])

s_builder = SceneBuilder(None)
SceneBuilder.autostart_roscore = not ROS_MASTER_ON_MOBILE
gscene = s_builder.create_gscene(crob)

gtems = s_builder.add_robot_geometries(
    color=COL_COLOR, display=True, collision=True)

gscene.set_workspace_boundary(
    -0.7, 1.55, -0.7, 2.5, -CLEARANCE, WS_HEIGHT, thickness=1.0)


from pkg.planning.scene import PlanningScene
pscene = PlanningScene(gscene, combined_robot=crob)

ROBOT_BASE = pscene.robot_chain_dict[ROBOT_NAME]['link_names'][0]
TIP_LINK = pscene.robot_chain_dict[ROBOT_NAME]["tip_link"]
CAM_LINK = TIP_LINK.replace("tcp", "link6")
MOBILE_BASE = pscene.robot_chain_dict[MOBILE_NAME]["tip_link"]
HOLD_LINK = MOBILE_BASE

viewpoint = add_cam(gscene, tool_link=CAM_LINK, center=(-0.0785, 0, 0.073))

add_brush(gscene, face_name=TOOL_NAME, tool_link=TIP_LINK,
          thickness=TOOL_THICKNESS, tool_dim=TOOL_DIM,
          col_color=COL_COLOR, center_off=TOOL_CENTER_OFF)

HOME_POSE = crob.home_pose
HOME_DICT = list2dict(HOME_POSE, gscene.joint_names)

from pkg.planning.pipeline import PlanningPipeline
ppline = PlanningPipeline(pscene)

# Set planner
from pkg.planning.motion.moveit.moveit_planner import MoveitPlanner
from pkg.planning.filtering.grasp_filter import GraspChecker
mplan = MoveitPlanner(pscene, enable_dual=False, 
                      incremental_constraint_motion=True)
mplan.motion_filters = [GraspChecker(pscene)]
mplan.update_gscene()
gcheck = GraspChecker(pscene)
mplan.motion_filters = [gcheck]

mplan.reset_PRQdict(enable_PRQ=0.5, radii=5e-2)
for tip_dir, SWEEP_AXIS in [
    (SweepDirections.front, "X"), (SweepDirections.up, "Z"), (SweepDirections.down, "Z")]:
    filename = SweepDirections.get_file_name(ROBOT_TYPE, tip_dir.name+SWEEP_AXIS)+"-PRQ.pkl"
    PRQ_PATH = os.path.join(SWEEP_DATA_DIR, filename)
    try:
        Pos_Rotvec_Qlist_dict = load_pickle(PRQ_PATH)
        mplan.register_PRQ(ROBOT_NAME, Pos_Rotvec_Qlist_dict, decimal=2)
        print("Loaded: {}".format(filename))
    except:
        print("File not exist: {}".format(filename))
        continue

from pkg.planning.task.rrt import TaskRRT
tplan = TaskRRT(pscene, node_trial_max=NODE_TRIAL_MAX)
tplan.prepare()
ppline.set_motion_planner(mplan)
ppline.set_task_planner(tplan)

# Register binders
from pkg.planning.constraint.constraint_actor import VacuumTool,     Gripper2Tool, PlacePlane, SweepFramer

brush_face = pscene.create_binder(
    bname=TOOL_NAME, gname=TOOL_NAME, _type=SweepFramer, 
    point=(0,0, -gscene.NAME_DICT['brush_face'].dims[2]/2-CLEARANCE), 
    rpy=(0,0,0))

gscene.create_safe(
    gtype=GEOTYPE.BOX, name="floor_box", link_name="base_link",
    dims=(15,15,0.4), center=(0,0,0), rpy=(0,0,0), 
    color=(1, 1, 1, 0.1), display=True, collision=False, fixed=True)

gscene.add_highlight_axis("hl", "base_coord", T=np.identity(4), dims=(0.5,0.1,0.1))

kmb = crob.robot_dict["kmb0"]
indy = crob.robot_dict["indy1"]
mobile_box = gscene.NAME_DICT['kmb0_platform_Box_2']
crob.simulator.set_gscene(gscene)
# indy.ktool.led_on()
# indy.ktool.led_off()
# time.sleep(2)
# indy.ktool.send_degree(-45)

if CONNECT_MOBILE:
    assert np.sum(np.abs(get_xyzw_cur()))>1e-4, "UDP Server not connected"
    
if CONNECT_TASK_PLANNER:
    from demo_proto.DisinfectionOperationServicer import serve_on_thread
    servicer = serve_on_thread()


# In[3]:


if CONNECT_INDY:
    with indy:
        stat = indy.get_robot_status()
        if stat['error'] or not stat['ready']:
            raise(RuntimeError("Indy not ready"))


# In[ ]:





# #### 0.1.2 Load environment map

# In[4]:


from demo_utils.ros_map_utils import KiroMobileMap
kmm = KiroMobileMap(MOBILE_IP, IP_CUR, CONNECT_MOBILE)
            
VALID_BOX_SCALE = 0.7
VALID_SCORE_CUT = 50
kmb.coster = (lambda Q: 
                  np.max(
                      kmm.get_box_costs(mobile_box, Q, kmm.T_bi, kmm.cost_im, kmm.resolution, 
                                        scale=VALID_BOX_SCALE)))
kmb.cost_cut = VALID_SCORE_CUT
kmb.gscene = gscene

kmm.init_node(timeout=10)


# In[5]:


pole_pt_list, pole_res = kmm.update_map(gscene, crob, MOBILE_BASE, timeout=100)
if not CONNECT_MOBILE:
    crob.joint_move_make_sure(kmm.Q_map)
    
if CONNECT_INDY:
    indy.move_joint_s_curve(crob.home_pose[6:], N_div=500)
else:
    indy.joint_move_make_sure(crob.home_pose[6:])

Qcur = crob.get_real_robot_pose()
crob.joint_move_make_sure(Qcur)


# In[6]:


# pole_list = kmm.add_pixel_poles("obs_pt", gscene, pole_pt_list, pole_res, height=WS_HEIGHT)
# gcheck.ignore_always = pole_list

# gscene.update_markers_all() 


# In[ ]:





# ## 1. Prepare scene

# In[7]:


HEIGHT_OFFSET = 0.02
TABLE_DIMS = (0.60,1.2, 0.71)
TABLE_LOC = (1.17,0.05,TABLE_DIMS[2]/2+HEIGHT_OFFSET)
TABLE_ANG = np.pi - np.deg2rad(0)
TABLE_MARGIN_I = 0.03
TABLE_MARGIN_O = 0.1
CLEARANCE = 0.002
TABLE_COLOR = (0.8,0.8,0.8,1)
SURF_COLOR = (0,1,0,0.4)
COL_COLOR = (1,1,1,0.2)

gscene.create_safe(GEOTYPE.BOX, "table", link_name="base_link",
                   dims=TABLE_DIMS, center=TABLE_LOC, rpy=Rot2rpy(Rot_axis(3, TABLE_ANG)),
                   color=TABLE_COLOR, collision=False
                  )
gscene.create_safe(GEOTYPE.BOX, "table_col", link_name="base_link",
                   dims=np.add(TABLE_DIMS, 
                               (TABLE_MARGIN_O*2, TABLE_MARGIN_O*2, 0)), 
                   center=(0,0,0), rpy=(0,0,0), color=COL_COLOR, 
                   collision=True, parent="table"
                  )
table_surf = gscene.create_safe(GEOTYPE.BOX, "table_surf", link_name="base_link",
                   dims=np.subtract(TABLE_DIMS[:2]+(CLEARANCE,), 
                                    (TABLE_MARGIN_I*2, TABLE_MARGIN_I*2, 0)), 
                   center=(0,0,(TABLE_DIMS[2]+CLEARANCE)/2), 
                   rpy=(0,0,0), color=SURF_COLOR, parent="table",
                   collision=False
                  )


# In[8]:


SOFA_DIMS = (0.50,0.98, 0.45)
SOFA_LOC = (1.15,1.4,SOFA_DIMS[2]/2+HEIGHT_OFFSET)
SOFA_SIDE_THIC = 0.2
SOFA_SIDE_HEIT = 0.27
SOFA_ANG = np.pi + np.deg2rad(0)

SOFA_MARGIN_O = 0.05
SOFA_MARGIN_Ix = 0.2 # 앞뒤
SOFA_MARGIN_Iy = 0.15  # 좌우
SOFA_MARGIN_Iz = 0.05 # 아래
SOFA_UV_DIST = 0.15

SOFA_COLOR = (0.4,0.4,0.4,1)

sofa = gscene.create_safe(GEOTYPE.BOX, "sofa", link_name="base_link",
                   dims=SOFA_DIMS, center=SOFA_LOC, 
                   rpy=Rot2rpy(Rot_axis(3, SOFA_ANG)),
                   color=SOFA_COLOR, collision=False
                  )
gscene.create_safe(GEOTYPE.BOX, "sofa_col", link_name="base_link",
                   dims=np.add(SOFA_DIMS, SOFA_MARGIN_O*2), 
                   center=(0,0,0), rpy=(0,0,0),
                   color=COL_COLOR, collision=True, parent="sofa"
                  )
sofa_left = gscene.create_safe(GEOTYPE.BOX, "sofa_left", link_name="base_link",
                   dims=(SOFA_DIMS[0], SOFA_SIDE_THIC, SOFA_DIMS[2]+SOFA_SIDE_HEIT), 
                   center=(0,(SOFA_DIMS[1]+SOFA_SIDE_THIC)/2, SOFA_SIDE_HEIT/2), 
                   rpy=(0,0,0), color=SOFA_COLOR, collision=False, parent="sofa"
                  )
gscene.create_safe(GEOTYPE.BOX, "sofa_left_col", link_name="base_link",
                   dims=np.add(sofa_left.dims, SOFA_MARGIN_O*2), 
                   center=(0,0,0), rpy=(0,0,0),
                   color=COL_COLOR, collision=True, parent="sofa_left"
                  )
sofa_right = gscene.create_safe(GEOTYPE.BOX, "sofa_right", link_name="base_link",
                   dims=(SOFA_DIMS[0], SOFA_SIDE_THIC, SOFA_DIMS[2]+SOFA_SIDE_HEIT), 
                   center=(0,-(SOFA_DIMS[1]+SOFA_SIDE_THIC)/2, SOFA_SIDE_HEIT/2), 
                   rpy=(0,0,0), color=SOFA_COLOR, collision=False, parent="sofa"
                  )
gscene.create_safe(GEOTYPE.BOX, "sofa_right_col", link_name="base_link",
                   dims=np.add(sofa_right.dims, SOFA_MARGIN_O*2), 
                   center=(0,0,0), rpy=(0,0,0),
                   color=COL_COLOR, collision=True, parent="sofa_right"
                  )
sofa_back = gscene.create_safe(GEOTYPE.BOX, "sofa_back", link_name="base_link",
                   dims=(SOFA_SIDE_THIC, SOFA_DIMS[1]+SOFA_SIDE_THIC*2, SOFA_DIMS[2]+SOFA_SIDE_HEIT), 
#                    dims=(SOFA_SIDE_THIC, SOFA_DIMS[1], SOFA_DIMS[2]+SOFA_SIDE_HEIT), 
                   center=(-(SOFA_DIMS[0]+SOFA_SIDE_THIC)/2, 0, SOFA_SIDE_HEIT/2), 
                   rpy=(0,0,0), color=SOFA_COLOR, collision=False, parent="sofa"
                  )
gscene.create_safe(GEOTYPE.BOX, "sofa_back_col", link_name="base_link",
                   dims=np.add(sofa_back.dims, SOFA_MARGIN_O*2), 
                   center=(0,0,0), rpy=(0,0,0),
                   color=COL_COLOR, collision=True, parent="sofa_back"
                  )

sofa_surf = gscene.create_safe(GEOTYPE.BOX, "sofa_surf", link_name="base_link",
                               dims=tuple(np.subtract(SOFA_DIMS[:2], (SOFA_MARGIN_Ix, SOFA_MARGIN_Iy*2)))+(CLEARANCE,), # (방석 앞뒤/좌우 마진)
                               center=(SOFA_MARGIN_Ix/2,0,SOFA_DIMS[2]/2+SOFA_UV_DIST), collision=False, 
                               rpy=(0,0,0), color=SURF_COLOR, parent="sofa"
                              )

sofa_back_surf = gscene.create_safe(GEOTYPE.BOX, "sofa_back_surf", link_name="base_link",
                               dims=sofa_back.dims[:2]+(CLEARANCE,), collision=False, 
                               center=(SOFA_MARGIN_Ix,0,sofa_back.dims[2]/2+SOFA_MARGIN_O+CLEARANCE), #  (등위 앞쪽 오프셋)
                               rpy=(0,0,0), color=SURF_COLOR, parent="sofa_back"
                              )

sofa_left_surf = gscene.create_safe(GEOTYPE.BOX, "sofa_left_surf", link_name="base_link",
                               dims=sofa_left.dims[:2]+(CLEARANCE,), collision=False, 
                               center=(0,0,sofa_left.dims[2]/2+SOFA_UV_DIST), 
                               rpy=(0,0,0), color=SURF_COLOR, parent="sofa_left"
                              )

sofa_right_surf = gscene.create_safe(GEOTYPE.BOX, "sofa_right_surf", link_name="base_link",
                               dims=sofa_right.dims[:2]+(CLEARANCE,), collision=False, 
                               center=(0,0,sofa_right.dims[2]/2+SOFA_UV_DIST), 
                               rpy=(0,0,0), color=SURF_COLOR, parent="sofa_right"
                              )

sofa_front = gscene.create_safe(GEOTYPE.BOX, "sofa_front", link_name="base_link",
                               dims=(sofa.dims[2], sofa.dims[1])+(CLEARANCE,), 
                               center=(sofa.dims[0]/2+SOFA_UV_DIST,0,SOFA_MARGIN_Iz), collision=False,  #  (다리앞 바닥 오프셋)
                               rpy=(0,np.pi/2,0), color=SURF_COLOR, parent="sofa"
                              )

sofa_left_front = gscene.create_safe(GEOTYPE.BOX, "sofa_left_front", link_name="base_link",
                               dims=(sofa_left.dims[2]-SOFA_MARGIN_Iz, sofa_left.dims[1])+(CLEARANCE,), 
                               center=(sofa_left.dims[0]/2+SOFA_UV_DIST,0,+SOFA_MARGIN_Iz/2), collision=False, # (왼쪽앞 바닥 오프셋)
                               rpy=(0,np.pi/2,0), color=SURF_COLOR, parent="sofa_left"
                              )

sofa_right_front = gscene.create_safe(GEOTYPE.BOX, "sofa_right_front", link_name="base_link",
                               dims=(sofa_right.dims[2]-SOFA_MARGIN_Iz, sofa_right.dims[1])+(CLEARANCE,), 
                               center=(sofa_right.dims[0]/2+SOFA_UV_DIST,0,+SOFA_MARGIN_Iz/2), collision=False,# (오른쪽앞 바닥 오프셋)
                               rpy=(0,np.pi/2,0), color=SURF_COLOR, parent="sofa_right"
                              )

sofa_back_front = gscene.create_safe(GEOTYPE.BOX, "sofa_back_front", link_name="base_link",# (등앞 바닥 오프셋, 등앞 좌우 오프셋)
                               dims=(max(TOOL_DIM), sofa_back.dims[1]-SOFA_SIDE_THIC*2-SOFA_MARGIN_Iy*2)+(CLEARANCE,),
#                                dims=(sofa_back.dims[2]-SOFA_DIMS[2]-SOFA_MARGIN_I, sofa_back.dims[1]-SOFA_MARGIN_I*3)+(CLEARANCE,), 
                               center=(sofa_back.dims[0]/2+SOFA_UV_DIST,0,sofa_back.dims[2]/2-sofa.dims[2]/2+max(TOOL_DIM)/2+SOFA_MARGIN_Iz),#  (등앞 아래 오프셋)
                               rpy=(0,np.pi/2,0), color=SURF_COLOR, collision=False, parent="sofa_back"
                              )


# In[9]:


ADD_POLES = False
if ADD_POLES:
    pole_pt_list = kmm.remove_poles_by_box(gscene, gscene.NAME_DICT["room_ws"], 
                        pole_pt_list, Qcur, inside=False, margin=0.1)
    pole_pt_list = kmm.remove_poles_by_box(gscene, gscene.NAME_DICT["table_col"], 
                        pole_pt_list, Qcur, margin=0.2)
    pole_pt_list = kmm.remove_poles_by_box(gscene, gscene.NAME_DICT["sofa_col"], 
                        pole_pt_list, Qcur, margin=0.2)
    # pole_pt_list = kmm.remove_poles_by_box(gscene, gscene.NAME_DICT["sofa_back_col"], 
    #                     pole_pt_list, Qcur, margin=0.1)
    # pole_pt_list = kmm.remove_poles_by_box(gscene, gscene.NAME_DICT["sofa_right_col"], 
    #                     pole_pt_list, Qcur, margin=0.1)

    pole_list = kmm.add_pixel_poles("obs_pt", gscene, pole_pt_list, pole_res, height=WS_HEIGHT)
    gcheck.ignore_always = pole_list

    gscene.update_markers_all() 


# In[ ]:





# ### Set UI

# In[10]:


from pkg.ui.ui_broker import *

# start UI
ui_broker = UIBroker.instance()
ui_broker.initialize(ppline, s_builder)
ui_broker.start_server()

ui_broker.set_tables()


# In[ ]:





# 
# ## 2. Prepare cleaning

# In[11]:


from pkg.planning.constraint.constraint_common import *
from pkg.planning.constraint.constraint_actor import *
from pkg.planning.constraint.constraint_subject import *
from pkg.utils.code_scraps import get_look_motion


# In[12]:


# mplan.reset_log(flag_log=True)
Qcur = crob.get_real_robot_pose()
Qcur[-1] = np.deg2rad(-45)
Qcur[6:] = crob.home_pose[6:]
crob.joint_move_make_sure(Qcur)
Qcur = crob.get_real_robot_pose()
if CONNECT_INDY:
    indy.ktool.led_off()


# In[13]:


# mplan.reset_log(flag_log=True)
Qcur = crob.get_real_robot_pose()
Qcur[-1] = np.deg2rad(-45)
if not CONNECT_INDY:
    Qcur[6:] = crob.home_pose[6:]
crob.joint_move_make_sure(Qcur)
HOME_POSE_SWEEP = np.copy(Qcur)
# HOME_POSE_SWEEP[6:] = 0
crob.home_pose = HOME_POSE_SWEEP
crob.home_dict = list2dict(crob.home_pose, 
                           gscene.joint_names)
floor_ws = gscene.NAME_DICT["floor_ws"]    

VEL_LIMS = 0.4
ACC_LIMS = 0.4
SWP_VEL_LIMS = 0.2
SWP_ACC_LIMS = 0.2
RADI_DEG = 1

if CONNECT_INDY:
    indy.QVEL_LEVEL = 1
#     indy.collision_policy = indy_trajectory_client_nosdk.POLICY_NO_COLLISION_DETECTION
    indy.reset()

gtimer = GlobalTimer()
    
mode_switcher=ModeSwitcherLED(pscene, robot_name=ROBOT_NAME, brush_face=brush_face.geometry)

def no_offset(gxter, crob, mplan, robot_name, Qref):
    return Qref, Qref

def cleaning_fun(surface, tip_dir, sweep_dir, tool_dir, Qcur,
                 swp_vel_lims=SWP_VEL_LIMS, swp_acc_lims=SWP_ACC_LIMS,
                 div_num=None):
    if sweep_dir.lower() == "z":
        EE_HEIGHT = None
    else:
        T_e_brush = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)
        T_brush_e = SE3_inv(T_e_brush)
        EE_HEIGHT = round(surface.get_tf(HOME_DICT)[2,3] + surface.dims[2]/2, 5)                         + T_brush_e[2, 3] - INDY_BASE_OFFSET[2]
    gtimer.reset(scale=1, timeunit='s', stack=True)
    gxter = GreedyExecuter(ppline, brush_face, TOOL_DIM, Qcur, 
                           vel_lims=VEL_LIMS, acc_lims=ACC_LIMS, 
                           swp_vel_lims=swp_vel_lims, swp_acc_lims=swp_acc_lims
                          )

    gxter.set_test_kwargs(multiprocess=PLANNING_MULTIPROC, N_agents=N_AGENTS,
                          timeout=TIMEOUT_MOTION, timeout_loop=TIMEOUT_FULL, 
                          verbose=VERBOSE, max_solution_count=MAX_SOL_NUM)

    gxter.get_division_dict(surface, tip_dir, sweep_dir, EE_HEIGHT,div_num=div_num)
    gxter.init_base_divs(Qcur)
    # gxter.mark_tested(None, None, covereds_all, [])
    snode_schedule_list, Qcur, covereds = gxter.greedy_execute(
        Qcur, tool_dir, mode_switcher, no_offset, cost_cut=BASE_COST_CUT)

    gxter.test_clear()
    print(gtimer)
    gscene.clear_virtuals()
    return snode_schedule_list, Qcur


# In[ ]:





# ## Load Schedules

# In[14]:


snode_schedule_list_table_all, snode_schedule_list_sofa_all = load_schedules()


# In[ ]:





# ## Loop table

# In[ ]:

import cv2
cv2.imshow("terminator", np.zeros((100,100,3), dtype=np.uint8))
try:
    while True:
        key = cv2.waitKey(5000)
        print("key input: {}".format(key))
        if key==27:
            break
        print("=========== START ================")
        for snode_schedule_list in snode_schedule_list_table_all:
            play_cleaning_schedule(ppline, table_surf, snode_schedule_list, mode_switcher, TOOL_DIM)
        print("=========== FINISHED ================")
finally:
    print("=========== STOPPINGG ================")
    s_builder.xcustom.clear()
print("=========== STOPPED ================")
sys.exit()

