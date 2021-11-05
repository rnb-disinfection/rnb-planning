#!/usr/bin/env python
# coding: utf-8

# # Demo Script for Milestone 10.15

# ## 0 Prepare task

# ### 0.1 prepare planning scene

# #### Run camera server on the camera computer (192.168.0.10, use vnc viewer)
# ```bash
# python stream_server.py --ip='192.168.0.10' # camera pc ip
# ```
# 
# #### Run shared detector on bash
# ```bash
# python3 /home/jhkim/Projects/rnb-planning/src/scripts/milestone_202110/utils/shared_detector.py
# ```
# 
# #### Check and request ip setting from mobile udp client (robot-side)

# #### 0.1.1 Set parameters and create planning scene

# In[ ]:


import numpy as np

CONNECT_CAM = False
ENABLE_DETECT = False

CONNECT_INDY = False
CONNECT_MOBILE = False

VISUALIZE = True
PLANNING_MULTIPROC = False

ROS_MASTER_ON_MOBILE = False
# Tool dimensions
TOOL_DIM = [0.08, 0.32]
# TOOL_OFFSET = 0.01
TOOL_OFFSET = 0.1 # tested one
MARGIN = 0
TRACK_THICKNESS = 0.001

INDY_BASE_OFFSET = (0.172,0,0.439)
INDY_BASE_RPY = (0,0,0)
TOOL_NAME = "brush_face"
WALL_THICKNESS = 0.01
CLEARANCE = 0.001

IP_CUR = "192.168.0.8"# get_ip_address()
MOBILE_IP = "192.168.0.102"
INDY_IP = "192.168.0.3"
CAM_HOST = '192.168.0.10'

print("Current PC IP: {}".format(IP_CUR))
print("Mobile ROB IP: {}".format(MOBILE_IP))
print("CAM SERVER IP: {}".format(CAM_HOST))


# In[ ]:


import os
import sys
sys.path.append(os.path.join(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src')))
sys.path.append(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src/scripts/milestone_202110'))

from pkg.global_config import RNB_PLANNING_DIR
from pkg.utils.utils import *    
from pkg.utils.rotation_utils import *
from pkg.controller.combined_robot import *
from demo_utils.streaming import *
from demo_utils.detect_table import *
from demo_utils.area_select import *
from pkg.detector.aruco.marker_config import get_aruco_map
aruco_map = get_aruco_map()

from pkg.geometry.builder.scene_builder import SceneBuilder
from demo_utils.environment import *
from demo_utils.area_select import DATASET_DIR, SweepDirections

from utils.streaming import *
from utils.detection_util import *

mobile_config = RobotConfig(0, RobotType.kmb, ((0,0,0), (0,0,0)),
                "{}/{}".format(MOBILE_IP, IP_CUR), 
                            specs={"dummy":not CONNECT_MOBILE})
robot_config = RobotConfig(1, RobotType.indy7, 
                           (INDY_BASE_OFFSET, INDY_BASE_RPY),
                INDY_IP, root_on="kmb0_platform", 
                           specs={"no_sdk":True})
MOBILE_NAME = mobile_config.get_indexed_name()
ROBOT_NAME = robot_config.get_indexed_name()
crob = CombinedRobot(robots_on_scene=[mobile_config, robot_config]
              , connection_list=[True, CONNECT_INDY])

s_builder = SceneBuilder(None)
SceneBuilder.autostart_roscore = not ROS_MASTER_ON_MOBILE
gscene = s_builder.create_gscene(crob)

gtems = s_builder.add_robot_geometries(
    color=(0,1,0,0.5), display=True, collision=True)
gscene.set_workspace_boundary(
    -4, 8, -7, 7, -CLEARANCE, 3, thickness=WALL_THICKNESS)


from pkg.planning.scene import PlanningScene
pscene = PlanningScene(gscene, combined_robot=crob)

ROBOT_BASE = pscene.robot_chain_dict[ROBOT_NAME]['link_names'][0]
TIP_LINK = pscene.robot_chain_dict[ROBOT_NAME]["tip_link"]
MOBILE_BASE = pscene.robot_chain_dict[MOBILE_NAME]["tip_link"]
HOLD_LINK = MOBILE_BASE

viewpoint = add_cam(gscene, tool_link=TIP_LINK)
# add_indy_tool_kiro(gscene, tool_link=TIP_LINK, 
# face_name=TOOL_NAME, zoff=TOOL_OFFSET)

HOME_POSE = -crob.home_pose
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

mplan.reset_PRQdict(enable_PRQ=True, radii=5e-2)
for tip_dir, SWEEP_AXIS in [
    (SweepDirections.front, "Z"), (SweepDirections.front, "X"), (SweepDirections.up, "Z"), (SweepDirections.down, "Z")]:
    filename = SweepDirections.get_file_name(RobotType.indy7, tip_dir.name+SWEEP_AXIS)+"-PRQ.pkl"
    PRQ_PATH = os.path.join(DATASET_DIR, filename)
    try:
        Pos_Rotvec_Qlist_dict = load_pickle(PRQ_PATH)
        mplan.register_PRQ(ROBOT_NAME, Pos_Rotvec_Qlist_dict, decimal=2)
        print("Loaded: {}".format(filename))
    except:
        print("File not exist: {}".format(filename))
        continue

from pkg.planning.task.rrt import TaskRRT
tplan = TaskRRT(pscene)
tplan.prepare()
ppline.set_motion_planner(mplan)
ppline.set_task_planner(tplan)

from pkg.ui.ui_broker import *

# start UI
ui_broker = UIBroker.instance()
ui_broker.initialize(ppline, s_builder)
ui_broker.start_server()

ui_broker.set_tables()

add_kiro_indytool_down(gscene, zoff=TOOL_OFFSET, 
                       tool_link=TIP_LINK, face_name=TOOL_NAME)

# Register binders
from pkg.planning.constraint.constraint_actor import VacuumTool,     Gripper2Tool, PlacePlane, SweepFramer, WayFramer

brush_face = pscene.create_binder(
    bname=TOOL_NAME, gname=TOOL_NAME, _type=SweepFramer, 
    point=(0,0, -gscene.NAME_DICT['brush_face'].dims[2]/2-CLEARANCE), 
    rpy=(0,0,0))


# waypoint
WP_DIMS = (0.6,0.4,WALL_THICKNESS)
gscene.create_safe(
    gtype=GEOTYPE.BOX, name="wayframer", link_name=HOLD_LINK,
    dims=WP_DIMS, center=(0,0,WP_DIMS[2]/2), rpy=(0,0,0), 
    color=(1, 0, 0, 1), display=True,
    collision=False, fixed=True)
wayframer = pscene.create_binder(
    bname="wayframer", gname="wayframer", _type=WayFramer, 
    point=(0,0,-WP_DIMS[2]/2-CLEARANCE), rpy=(0,0,0))

gscene.create_safe(
    gtype=GEOTYPE.BOX, name="floor_box", link_name="base_link",
    dims=(15,15,0.4), center=(0,0,0), rpy=(0,0,0), 
    color=(1, 1, 1, 0.1), display=True, collision=False, fixed=True)

gscene.add_highlight_axis("hl", "base_coord", T=np.identity(4), dims=(0.5,0.1,0.1))

kmb = crob.robot_dict["kmb0"]
indy = crob.robot_dict["indy1"]
mobile_box = gscene.NAME_DICT['kmb0_platform_Box_2']


# #### 0.1.2 Load saved environment map

# In[ ]:


from demo_utils.ros_map_utils import KiroMobileMap
kmm = KiroMobileMap(MOBILE_IP, IP_CUR, CONNECT_MOBILE)
            
VALID_BOX_SCALE = 0.8
VALID_SCORE_CUT = 80 
kmb.validifier = (lambda Q: 
                  np.max(
                      kmm.get_box_costs(mobile_box, Q, kmm.T_bil, kmm.lcost_im, kmm.lresolution, 
                                        scale=VALID_BOX_SCALE))<VALID_SCORE_CUT)


# In[ ]:


def update_map(timeout=5, try_num=5, time_wait=0):
    time.sleep(time_wait)
    maps = None
    for i in range(try_num):
        maps = kmm.get_maps(timeout=timeout)
        Q_map = crob.get_real_robot_pose()
        if not CONNECT_MOBILE:
            Q_map[:6] = kmb.xyzw2joints(load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cur_xyzw_view.pkl")))
        Tbm_map = gscene.get_tf(MOBILE_BASE, Q_map)
        kmm.set_maps(*maps, T_bm=Tbm_map, canny_ksize=10)
        
        gscene.show_pose(Q_map)    
        pt_list, costs = kmm.convert_im2scene(kmm.cost_im>0, kmm.resolution, kmm.T_bi, img_cost=kmm.cost_im)
        pt_list = np.subtract(pt_list, (0,0,kmm.resolution))
        YlOrRd = plt.get_cmap("YlOrRd")
        gcost_mesh = kmm.add_to_scene("global_cost", gscene, pt_list, kmm.resolution/2, costs, 
                                      colormap=lambda x: YlOrRd(x/2))

        lpt_list, lcosts = kmm.convert_im2scene(kmm.lcost_im>0, kmm.lresolution, kmm.T_bil, img_cost=kmm.lcost_im)
        lpt_list = np.subtract(lpt_list, (0,0,kmm.resolution*2))
        lcost_mesh = kmm.add_to_scene("local_cost", gscene, lpt_list, kmm.lresolution, lcosts)
        if  maps is not None:
            break


# In[ ]:


update_map()


# ## 1. Detect scene

# ### 1.0 Wait task start queue

# ### 1.1 Detect bed

# #### 1.1.1 Move to bed-seek pose 

# In[ ]:


Q_CUR = kmb.get_qcur()

VIEW_POSE = np.deg2rad([  0., 50.,  -70.,  -0.,  -90., 0])
VIEW_LOC = list(Q_CUR[:6])
VIEW_POSE_EXT = np.array(VIEW_LOC + list(VIEW_POSE))
if CONNECT_INDY:
    with indy:
        indy.joint_move_to(np.rad2deg(VIEW_POSE))
        time.sleep(0.5)
        indy.wait_for_move_finish()
        Qcur = np.deg2rad(indy.get_joint_pos())
else:
    Qcur = VIEW_POSE
gscene.show_pose(VIEW_POSE_EXT)


# In[ ]:


if ENABLE_DETECT:
    attacth_to_server()
    
turn_dir = 1
Q0 = np.rad2deg(VIEW_POSE_EXT[6:])
dQ = np.zeros(6)
while ENABLE_DETECT:
    # Take a picture again after rotate
    if CONNECT_CAM:
        rdict = stream_capture_image(
            ImageType.FirstView, obj_type="bed", host=CAM_HOST,
            crob=crob)
    else:
        rdict, VIEW_POSE_EXT = load_rdict("bed")
        

    cdp = rdict2cdp(rdict)
        
    
    # Output of inference(mask for detected table)
    mask_out = detect_from_server(rdict['color'])
    if np.any(mask_out):
        cdp_masked = apply_mask(cdp, mask_out)
        plt.imshow(cdp_masked.color[:,:,[2,1,0]])
        break
    if CONNECT_INDY:
        with indy:
            turn_dir *= -1
            dQ = np.add(dQ, [5,0,0,0,0,0])
            Qto = Q0+turn_dir*dQ
            Qto[0] = (Qto[0]+180/2)%180-180/2
            indy.joint_move_to(Qto)
            indy.wait_motion()
            VIEW_POSE_EXT[6:] = np.deg2rad(indy.get_joint_pos())


# #### 1.1.2  detect bed and add to the scene

# In[ ]:


VISUALIZE = False
T_bc = viewpoint.get_tf(VIEW_POSE_EXT)
T_cb = SE3_inv(T_bc)
gscene.show_pose(VIEW_POSE_EXT)
if ENABLE_DETECT:
    icp_bed = MultiICP(model=MODEL_DIR + '/bed/bed.STL', 
                       Toff=SE3([[0,1,0],[0,0,1],[1,0,0]], [0.455,0,1.02]))
    # Try ICP1
    if np.any(mask_out):
        pcd = icp_bed.add_image(cdp_masked, Tc=None)

        Tguess = icp_bed.get_initial_by_center(R=np.matmul(T_cb[:3,:3], Rot_axis(3,np.pi)), 
                                               offset=np.matmul(T_cb[:3,:3], (1.1*0.7,0,-0.6)))
        Tbs1, fitness1 = icp_bed.compute_ICP(To=Tguess, thres=0.15, visualize=VISUALIZE)
        Tbs2, fitness2 = icp_bed.compute_ICP(None, thres=0.15, visualize=VISUALIZE)
        
    # Better result is adopted
    T_co_bed = Tbs1 if fitness1 > fitness2 else Tbs2
    T_bo_bed = np.matmul(T_bc, T_co_bed)

    bed_center = T_bo_bed[:3,3]
    bed_rpy = Rot2rpy(T_bo_bed[:3,:3])
    COLOR_BED_COL = (0,1,0,0.3)
    T_bo_new = align_z(T_bo_bed)
    bed_rpy = Rot2rpy(T_bo_new[:3,:3])

    # adjust
    bed_center[2]=0
    if Rot_rpy(bed_rpy)[0,0] > 0:
        bed_rpy[2] += np.pi

    bed_mat = add_bed(gscene, bed_center, bed_rpy, COLOR_BED_COL)
    if VISUALIZE: # show final result
        icp_bed.draw(np.matmul(T_cb, gscene.NAME_DICT["bed_vis"].get_tf(VIEW_POSE_EXT)))
    
else:
    bed_center = (5.1,-0.1,0)
    bed_rpy = (0,0,np.pi)
    COLOR_BED_COL = (0,1,0,0.3)
    bed_mat = add_bed(gscene, bed_center, bed_rpy, COLOR_BED_COL)

bed_vis = gscene.NAME_DICT["bed_vis"]


# ### 1.2 Detect Closet

# #### 1.2.0 set checker

# In[ ]:


wp_task, wp_hdl = add_waypoint_task(
    pscene, "waypoint", WP_DIMS, (0,0,0), (0,0,0), 
    parent="floor_ws", color=(0, 0, 1, 1))
ccheck = CachedCollisionCheck(gcheck, wp_task, wp_hdl, wayframer)


# #### 1.2.1  move to full view position

# ##### calc fule view pose

# In[ ]:


VIEW_MOVED = np.deg2rad([  0., 50.,  -70.,  -0.,  -80., 0])
# VIEW_MOVED = np.deg2rad([  0., -10.,  -0.,  -0.,  -100., 0])
VIEW_POSE_EXT[crob.idx_dict[ROBOT_NAME]] = VIEW_MOVED

bed_vis = gscene.NAME_DICT["bed_vis"]
T_bo = bed_vis.get_tf(list2dict(VIEW_POSE_EXT, gscene.joint_names))

if ENABLE_DETECT:
    h_fov_hf = np.arctan2(cdp.intrins[0], 2*cdp.intrins[2])
    # Determine the location of closet
    CLOSET_LOCATION = check_location_top_table(
        cdp2pcd(cdp), cdp2pcd(cdp_masked), T_bc, T_bo, 
        bed_dims=bed_mat.dims, visualize=False)
    print("CLOSET on {}".format(CLOSET_LOCATION))
else:
    h_fov_hf = np.pi/4
    CLOSET_LOCATION = "LEFT"
    
if CLOSET_LOCATION == "LEFT":
    angle_ref = 150
elif CLOSET_LOCATION == "RIGHT":       
    angle_ref = -150
    
bed_dim = np.linalg.norm(bed_mat.dims)
x_z_ratio = np.tan(h_fov_hf)
bed_dist = (bed_dim/2) / x_z_ratio * 3 
while True:
    angle_view = angle_ref + np.random.uniform(-10, 10)
    dist_view = bed_dist + np.random.uniform(-1, 1)*bed_dist/8
    Tbs = bed_mat.get_tf(VIEW_POSE_EXT)
    Tbs = np.matmul(Tbs, 
                    SE3(np.identity(3), (-bed_mat.dims[0]/2, 0,0)))
    Tsc = np.matmul(SE3(Rot_axis(3, np.deg2rad(angle_view)), (0,)*3), 
                    SE3(np.identity(3), (-dist_view, 0,0)))
    Tbc = np.matmul(Tbs, Tsc)
    Tmc = viewpoint.get_tf(VIEW_POSE_EXT, from_link=MOBILE_BASE)
    Tmc[:3,:3] = np.identity(3)
    Tbm = np.matmul(Tbc, SE3_inv(Tmc))
    full_view_ext = np.copy(VIEW_POSE_EXT)
    full_view_ext[:2] = Tbm[:2,3]
    full_view_ext[2] = Rot2axis(Tbm[:3, :3], 3)
    gscene.show_pose(full_view_ext)
    ccheck.clear()
    res = ccheck(T_loal=Tbm, 
                 Q_dict=list2dict(full_view_ext, gscene.joint_names))
    if res:
        VIEW_MOVED_EXT = full_view_ext
        break


# ##### move to full view pose

# In[ ]:


if CONNECT_INDY:
    with indy:
        indy.joint_move_to(np.rad2deg(VIEW_MOVED))
kmb.joint_move_make_sure(VIEW_MOVED_EXT[:6])
VIEW_MOVED_EXT = crob.get_real_robot_pose()
if not CONNECT_INDY:
    VIEW_MOVED_EXT[6:] = VIEW_MOVED.copy()
gscene.show_pose(VIEW_MOVED_EXT)


# #### 1.2.2 redetect bed

# In[ ]:


VISUALIZE = False
T_bc = viewpoint.get_tf(VIEW_MOVED_EXT)
# capture image of full view
if CONNECT_CAM:
    rdict = stream_capture_image(
        ImageType.FullView, obj_type="full_view", host=CAM_HOST,
        crob=crob)
    
else:
    rdict, VIEW_MOVED_EXT = load_rdict("full_view")
    gscene.show_pose(VIEW_MOVED_EXT)
    
cdp = rdict2cdp(rdict)

if ENABLE_DETECT:
    # Output of inference(mask for detected table)
    mask_out = detect_from_server(cdp.color)
    if np.any(mask_out):
        cdp_masked = apply_mask(cdp, mask_out)
        plt.imshow(cdp_masked.color[:,:,[2,1,0]])

        icp_bed.clear()
        pcd = icp_bed.add_image(cdp_masked, Tc=T_bc)
        T_bs_pre = gscene.NAME_DICT["bed_vis"].get_tf(VIEW_MOVED_EXT)
        T_bo_bed, fitness = icp_bed.compute_ICP(To=T_bs_pre, thres=0.15, visualize=VISUALIZE)
        #adjust
        T_bo_bed[2]=0
        T_bo_bed[:3, :3]=Rot_axis(3, Rot2axis(T_bo_bed[:3, :3], 3))
        move_bed(gscene, T_bo_bed[:3,3], Rot2rpy(T_bo_bed[:3,:3]))

        if VISUALIZE: # show final result
            icp_bed.draw(np.matmul(T_cb, gscene.NAME_DICT["bed_vis"].get_tf(VIEW_MOVED_EXT)))
    else:
        raise(RuntimeError("bed not detected"))


# #### 1.2.3  detect and add closet

# In[ ]:


VISUALIZE = True
if ENABLE_DETECT:
    icp_closet = MultiICP(model=MODEL_DIR + '/top_table/top_table.STL', 
                       Toff=SE3([[1,0,0],[0,0,1],[0,-1,0]], [0.3,0,0.2725]))
    
    if CONNECT_CAM:
        rdict = stream_capture_image(ImageType.FirstView, obj_type="top_table", host=CAM_HOST,
                                     crob=crob)
    else:
        rdict, VIEW_MOVED_EXT = load_rdict("top_table")

    T_bc = viewpoint.get_tf(VIEW_MOVED_EXT)
    
    cdp = rdict2cdp(rdict)
    pcd_closet = cdp2pcd(cdp, T_bc)
    
    
    pcd_masked = mask_boxes(pcd_closet, 
                        boxes=[gscene.NAME_DICT["bed_{}_space".format(CLOSET_LOCATION.lower())]], 
                        Q=VIEW_MOVED_EXT, inside=True, 
                        merge_rule=np.all, link_ref="base_link")
    pcd_masked = mask_boxes(pcd_masked, 
                        boxes=[gscene.NAME_DICT["bed_box"], 
                               gscene.NAME_DICT["bed_wall"], 
                               gscene.NAME_DICT["floor_box"]], 
                        Q=VIEW_MOVED_EXT, inside=False, 
                        merge_rule=np.all, link_ref="base_link")
    pcd_masked, ind = pcd_masked.remove_radius_outlier(nb_points=20, radius=0.06)
    icp_closet.add_pointcloud(pcd_masked, T_bc)
    
    Tbs = bed_vis.get_tf(VIEW_MOVED_EXT)
    initial_guess = icp_closet.get_initial_by_median(
        Tbs[:3, :3], (0.3,0,-1))
    T_bo, fitness = icp_closet.compute_ICP(initial_guess, thres=0.05, visualize=VISUALIZE)
    
    #adjust
    T_bo[2]=0
    T_bo[:3, :3]=Rot_axis(3, Rot2axis(T_bo[:3, :3], 3))
#     gscene.show_point_cloud(pcd_closet.points, "allpoints", color=(0,0,0,0.5), dims=(0.01,0.01,0.01), sample_to=500)
#     gscene.show_point_cloud(pcd_masked.points, "masked", color=(1,0,0,1), dims=(0.02,0.02,0.02))
else:
    T_bo = T_xyzrpy((np.matmul(Rot_rpy(bed_rpy), (-0.75,-1.5,0))+bed_center, 
                     bed_rpy))
    
closet_leftup, closet_rightup, closet_down = add_closet(
    gscene, closet_center=T_bo[:3,3], closet_rpy=Rot2rpy(T_bo[:3,:3]), 
    COLOR_CLOSET_COL=(0,1,0,0.3))


# In[ ]:


if CONNECT_MOBILE and CONNECT_CAM:
    save_pickle("gargs_{}.pkl".format(get_now()), 
                gscene.get_gtem_args())
else:
    filelist = sorted([fname for fname in os.listdir(os.getcwd()) if fname.startswith("gargs_")])
    gtem_args = load_pickle(filelist[-1])      
    for garg in gtem_args:
        if garg['parent'] == None and garg['link_name']=="base_link":
            if garg["name"] in gscene.NAME_DICT:
                gscene.NAME_DICT[garg["name"]].set_offset_tf(center=garg["center"], orientation_mat=Rot_rpy(garg["rpy"]))
gscene.update_markers_all()  


# In[ ]:





# 
# ## 2. Closet cleaning

# ### 2.1. Make closet cleaning plan

# In[ ]:


from pkg.planning.constraint.constraint_common import *
from pkg.planning.constraint.constraint_actor import *
from pkg.planning.constraint.constraint_subject import *
from pkg.utils.code_scraps import get_look_motion


# In[ ]:


Q_CUR = VIEW_MOVED_EXT
HOME_POSE_SWEEP = np.copy(Q_CUR)
# HOME_POSE_SWEEP[6:] = 0
crob.home_pose = HOME_POSE_SWEEP
crob.home_dict = list2dict(crob.home_pose, gscene.joint_names)
floor_ws = gscene.NAME_DICT["floor_ws"]    

add_kiro_indytool_down(gscene, zoff=TOOL_OFFSET, tool_link=TIP_LINK, face_name=TOOL_NAME)
brush_face = pscene.create_binder(bname=TOOL_NAME, gname=TOOL_NAME, _type=SweepFramer, 
                                  point=(0,0,-gscene.NAME_DICT['brush_face'].dims[2]/2-CLEARANCE), 
                                  rpy=(0,0,0))

def make_plan_fun(surface, Q_CUR, tip_dir, sweep_dir, tool_dir, plane_val, xout_cut=False):
    ccheck.clear()
    div_base_dict, Tsm_keys, surface_div_centers, div_num, (ax_step, ax_swp, ax_pln) = \
        get_division_dict(surface, brush_face, robot_config,
                                              plane_val=plane_val, tip_dir=tip_dir, sweep_dir=sweep_dir,
                                              TOOL_DIM=TOOL_DIM, ccheck=ccheck, 
                                              resolution=0.02, xout_cut=xout_cut)
    ax_swp_base = ax_swp
    Rre = SweepDirections.get_dcm_re(tip_dir)
    Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)  ## get data
    Rrt = np.matmul(Rre, Tet[:3,:3])
    ax_swp_tool = np.where(np.abs(Rrt.transpose()[:,ax_swp_base]).astype(np.int))[0][0]

    HOME_POSE_MOVE = np.copy(Q_CUR[6:])
    test_fun = TestBaseDivFunc(ppline, floor_ws, surface, ax_swp_tool, ax_swp_base,
                               WP_DIMS, TOOL_DIM, crob.home_dict, tool_dir=tool_dir,
                               multiprocess=PLANNING_MULTIPROC, timeout=0.3, timeout_loop=3)
    test_fun.clear()

    idx_bases, idc_divs, covered_all, snode_schedule_list = select_max_cover_bases(
        div_base_dict, Tsm_keys, surface_div_centers, div_num, ax_step, 
        test_fun=test_fun, lazy_base_thresh=np.max(TOOL_DIM)/2)

    snode_schedule_list, idx_bases, idc_divs, scene_args_list, scene_kwargs_list = refine_order_plan(
        ppline, snode_schedule_list, idx_bases, idc_divs, Q_CUR, 
        floor_ws, wayframer, surface, Tsm_keys, surface_div_centers,  
        WP_DIMS, TOOL_DIM, ROBOT_NAME, MOBILE_NAME, HOME_POSE_MOVE, 
        ax_swp_tool, ax_swp_base, tool_dir=1)
    test_fun.clear()
    if len(snode_schedule_list)>0:
        Q_CUR = snode_schedule_list[-1][-1].state.Q
    return snode_schedule_list, scene_args_list, scene_kwargs_list, Q_CUR, test_fun


# In[ ]:


snode_schedule_list_rightup, scene_args_list_rightup, scene_kwargs_list_rightup, Q_CUR, test_fun =     make_plan_fun(closet_rightup, Q_CUR, "up", "Z", 1, plane_val=None)


# In[ ]:


snode_schedule_list_leftup, scene_args_list_leftup, scene_kwargs_list_leftup, Q_CUR, test_fun=     make_plan_fun(closet_leftup, Q_CUR, "up", "Z", 1, plane_val=None)


# In[ ]:


snode_schedule_list_down, scene_args_list_down, scene_kwargs_list_down, Q_CUR, test_fun =     make_plan_fun(closet_down, Q_CUR, "down", "Z", -1, plane_val=None)


# In[ ]:


for snode_schedule in snode_schedule_list_leftup:
    ppline.play_schedule(snode_schedule)


# In[ ]:


for snode_schedule in snode_schedule_list_rightup:
    ppline.play_schedule(snode_schedule)


# In[ ]:


for snode_schedule in snode_schedule_list_down:
    ppline.play_schedule(snode_schedule)


# In[ ]:





# In[ ]:


surface, Q_CUR, tip_dir, sweep_dir, tool_dir = closet_leftup, Q_CUR, "up", "Z", 1
plane_val=None
xout_cut=False 
# def make_plan_fun(surface, Q_CUR, tip_dir, sweep_dir, tool_dir, plane_val, xout_cut=False):
# ccheck.clear()
# div_base_dict, Tsm_keys, surface_div_centers, div_num, (ax_step, ax_swp, ax_pln) = \
#                         get_division_dict(surface, brush_face, robot_config, 
#                                           plane_val=plane_val, tip_dir=tip_dir, sweep_dir=sweep_dir,
#                                           TOOL_DIM=TOOL_DIM, ccheck=ccheck, 
#                                           resolution=0.02, xout_cut=xout_cut)
# ax_swp_base = ax_swp
# Rre = SweepDirections.get_dcm_re(tip_dir)
# Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)  ## get data
# Rrt = np.matmul(Rre, Tet[:3,:3])
# ax_swp_tool = np.where(np.abs(Rrt.transpose()[:,ax_swp_base]).astype(np.int))[0][0]

# HOME_POSE_MOVE = np.copy(Q_CUR[6:])
# test_fun = TestBaseDivFunc(ppline, floor_ws, surface, ax_swp_tool, ax_swp_base,
#                            WP_DIMS, TOOL_DIM, crob.home_dict, tool_dir=tool_dir,
#                            multiprocess=PLANNING_MULTIPROC, timeout=0.3, timeout_loop=3)
# test_fun.clear()

# idx_bases, idc_divs, covered_all, snode_schedule_list = select_max_cover_bases(
#     div_base_dict, Tsm_keys, surface_div_centers, div_num, ax_step, 
#     test_fun=test_fun, lazy_base_thresh=np.max(TOOL_DIM)/2)

# snode_schedule_list, idx_bases, idc_divs, scene_args_list, scene_kwargs_list = refine_order_plan(
#     ppline, snode_schedule_list, idx_bases, idc_divs, Q_CUR, 
#     floor_ws, wayframer, surface, Tsm_keys, surface_div_centers,  
#     WP_DIMS, TOOL_DIM, ROBOT_NAME, MOBILE_NAME, HOME_POSE_MOVE, 
#     ax_swp_tool, ax_swp_base, tool_dir=1)
# test_fun.clear()
# if len(snode_schedule_list)>0:
#     Q_CUR = snode_schedule_list[-1][-1].state.Q
# # return snode_schedule_list, scene_args_list, scene_kwargs_list, Q_CUR, test_fun


# In[ ]:


surface, brush_face, robot_config = surface, brush_face, robot_config
plane_val=plane_val
tip_dir=tip_dir
sweep_dir=sweep_dir
TOOL_DIM=TOOL_DIM
ccheck=ccheck
resolution=0.02
sweep_margin=0
io_margin=0.2
xout_cut=False
div_num=None
# def get_division_dict(surface, brush_face, robot_config, plane_val, tip_dir, sweep_dir, TOOL_DIM, ccheck, resolution,
#                       sweep_margin=0, io_margin=0.2, xout_cut=False, div_num=None):
gcheck = ccheck.gcheck
pscene = gcheck.pscene
gscene = pscene.gscene
crob = pscene.combined_robot

## divide surface with squares
rect_div_size_ref = np.max(TOOL_DIM)
# effective surface dims - surface to divide, except most outside tool dimension
surface_dim_eff = np.clip(np.subtract(surface.dims[:2], rect_div_size_ref), 0, 1e5)

if div_num is None:
    div_num_ = np.ceil(surface_dim_eff / rect_div_size_ref).astype(np.int)
else:
    div_num_ = np.subtract(div_num, 1)
div_num_denom = np.copy(div_num_)
if np.min(div_num_) == 0:
    div_num_denom[np.where(div_num_ == 0)] = 1  # to resolve numerical singularity
rect_div_size = surface_dim_eff / div_num_denom  # reduce square size to fit the closed ends
if np.min(rect_div_size) == 0:
    rect_div_size[np.where(rect_div_size == 0)] = np.max(rect_div_size)  # to resolve numerical singularity
div_num = div_num_ + 1

surface_div_centers = [tuple(np.round(-surface_dim_eff / 2 + np.multiply(rect_div_size, (i, j)), 5))
                       for i, j in product(range(div_num[0]), range(div_num[1]))]

## make feasible base-div dictionary
ROBOT_BASE = pscene.robot_chain_dict[robot_config.get_indexed_name()]["link_names"][0]
TIP_LINK = brush_face.geometry.link_name
Tbs = surface.get_tf(crob.home_dict)
Tmr = gcheck.gscene.get_tf(to_link=ROBOT_BASE, from_link=robot_config.root_on, Q=crob.home_dict)
Trm = SE3_inv(Tmr)
Rre = SweepDirections.get_dcm_re(tip_dir)
Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)  ## get data
rtype = robot_config.type.name
sweep_path = os.path.join(SWEEP_DAT_PATH, SweepDirections.get_file_name(rtype, tip_dir+sweep_dir))
sweep_max = np.loadtxt(sweep_path + "-max.csv", delimiter=",")
sweep_min = np.loadtxt(sweep_path + "-min.csv", delimiter=",")

ax_swp = "XYZ".index(sweep_dir)  # sweep axis in robot coord
Rre, ax_step , ax_pln = SweepDirections.get_Re_step_level(tip_dir, ax_swp)

if ax_pln == 2:
    assert plane_val is not None, "Height reference must be provided when plane axis = 2"
    print("Height Reference: ", plane_val)
    idx_pln = np.argmin(np.abs(sweep_max[:, ax_pln] - plane_val))
    val_pln = sweep_max[idx_pln, ax_pln]
    idc_pln = np.where(sweep_max[:, ax_pln] == val_pln)[0]
    sweep_max = sweep_max[idc_pln, :]
    sweep_min = sweep_min[idc_pln, :]
else:
    ## get all sweep points
    Tbs = surface.get_tf(crob.home_dict)
    robot_base = crob.get_robot_base_dict()[robot_config.get_indexed_name()]
    Tbr = gscene.get_tf(robot_base, crob.home_dict)
    Trs = np.matmul(SE3_inv(Tbr), Tbs)
    Hoff_et = np.matmul(Rre, Tet[:3, 3])[2]  # Rre*Pet
    div_heights_r = np.matmul(
        Trs[:3, :2], np.transpose(surface_div_centers)
    ).transpose()[:, 2] + Trs[2, 3]  # Prt[2]
    if ax_step==2:  # height is not fixed and should be decided by step
        idc_h_matchs = []
        for div_rh in div_heights_r:  # for each division heights, collect according indices
            h_diffs = np.abs(sweep_min[:, 2] + Hoff_et - div_rh)  # sweep_min: Pre
            idx_min = np.argmin(h_diffs)
            if h_diffs[idx_min] > rect_div_size_ref * 0.1:
                continue
            val_min = sweep_min[idx_min, 2]
            idc_min = np.where(sweep_min[:, 2] == val_min)[0]
            idc_h_matchs += list(idc_min)
        idc_h_matchs = sorted(set(idc_h_matchs))
        sweep_min = sweep_min[idc_h_matchs]
        sweep_max = sweep_max[idc_h_matchs]
    elif ax_swp==2:  #  # height is not fixed and should be decided by sweep points
        sweep_min = sweep_min
        sweep_max = sweep_max
    else:
        raise(RuntimeError("Z axis is not decided to be any of sweep, step, plane axes"))


## cut margins at the edge
sweep_max[:, ax_swp] -= (np.max(TOOL_DIM) / 2 + sweep_margin)
sweep_min[:, ax_swp] += (np.max(TOOL_DIM) / 2 + sweep_margin)
idx_ok = np.where(sweep_max[:, ax_swp] > sweep_min[:, ax_swp])[0]
sweep_max = sweep_max[idx_ok]
sweep_min = sweep_min[idx_ok]

## apply median
sweep_max[:, ax_swp] = moving_median(sweep_max[:, ax_swp])
sweep_min[:, ax_swp] = moving_median(sweep_min[:, ax_swp])

swp_points_dict = {0: [], 1: []}
for ax_swp_s in range(2):
    div_size_swp = rect_div_size[ax_swp_s]
    div_size_nswp_grid = rect_div_size[(ax_swp_s+1)%2] / 2
    step_points_dict = defaultdict(lambda: defaultdict(list))
    sweep_num_list = []
    for step_val, pln_val, min_val_ful, max_val_ful in zip(sweep_min[:, ax_step], sweep_min[:, ax_pln],
                                               sweep_min[:, ax_swp], sweep_max[:, ax_swp]):
        if ax_swp != 2: # grid divide step with div_size, offset = 0 or div_size/2
        # div sweep range with grid size = div_size/2
            min_grid, max_grid = np.divide([min_val_ful, max_val_ful], (div_size_swp/2))
            min_grid = np.ceil(min_grid).astype(int)
            max_grid = np.floor(max_grid).astype(int)
            diff_grid = max_grid - min_grid # in div_size/2 grid

            if diff_grid % 2 != 0:  # cut the point with smaller margin if sweep range is not divided by div_size
                if max_val_ful - (max_grid*div_size_swp/2) >= (min_grid*div_size_swp/2) - min_val_ful:
                    max_grid -= 1
                else:
                    min_grid += 1
                diff_grid = max_grid - min_grid
            assert diff_grid % 2 == 0

            if diff_grid < 0:
                continue
            min_val, max_val = np.multiply([min_grid, max_grid], div_size_swp/2) # in real value
            diff_val = max_val - min_val # in real value
            sweep_num = int(diff_grid / 2)+1 # in div_size grid
            sweep_num_list.append(sweep_num)

            swp_points = np.zeros((sweep_num, 3))
            swp_points[:, ax_swp] = min_val + np.arange(sweep_num) * div_size_swp
        else:
            h_in_range = div_heights_r[
                np.where(np.logical_and(min_val_ful+Hoff_et< div_heights_r, 
                                        div_heights_r < max_val_ful+Hoff_et))]-Hoff_et
            if len(h_in_range)==0:
                continue
            swp_points = np.zeros((len(h_in_range), 3))
            swp_points[:, ax_swp] = h_in_range

        lv_stp = int(np.round(step_val/div_size_nswp_grid))
        lv_pln = int(np.round(pln_val/div_size_nswp_grid))
        off_val = int(np.round((step_val-lv_stp*div_size_nswp_grid
                                if plane_val is not None else
                                pln_val-lv_pln*div_size_nswp_grid) / resolution))
        swp_points[:, ax_step] = step_val if ax_step==2 else div_size_nswp_grid*lv_stp+off_val*resolution
        swp_points[:, ax_pln] = plane_val if plane_val is not None else div_size_nswp_grid*lv_pln+off_val*resolution
        if len(step_points_dict[off_val][(lv_stp, lv_pln)])<len(swp_points):
            step_points_dict[off_val][(lv_stp, lv_pln)] = swp_points
        else:
            step_points_dict[off_val][(lv_stp, lv_pln)] = swp_points
    step_points_list = list(step_points_dict.values())
    if len(step_points_list)>0:
        swp_points_dict[ax_swp_s] = np.concatenate(max(step_points_list, key=lambda lv_dict: np.sum(map(len, lv_dict.values()))).values())
    else:
        swp_points_dict[ax_swp_s] = []

## get base-sweep combinations
div_base_dict = defaultdict(lambda: defaultdict(list))
Tbm_in_all = []
Tbm_float_all = []
Tbm_fail_all = []
Tbm_succ_all = []
Tsm_swp_pairs = []
for i_div, div_center in list(enumerate(surface_div_centers)):  # for each division
    Tsc = SE3(np.identity(3), tuple(div_center) + (surface.dims[2] / 2,))
    for i in range(4) if SweepDirections.check_fourway(tip_dir) else [0, 2]:
        Tct = SE3(Rot_axis(3, i * np.pi / 2), [0, 0, 0])
        Tst = matmul_series(Tsc, Tct)
        Rsr = matmul_series(Tst[:3, :3], Tet[:3, :3].transpose(), Rre.transpose())
        ax_swp_s = np.where(np.abs(Rsr[:3, ax_swp]) > 0.5)[0][0]
        Tbt = np.matmul(Tbs, Tst)
        for i_s, swp_point in enumerate(swp_points_dict[ax_swp_s]):  # for each valid swp points from robot
            Tre = SE3(Rre, swp_point)
            Trt = matmul_series(Tre, Tet)
            Tsr = matmul_series(Tst, SE3_inv(Trt))
            Tsm = np.matmul(Tsr, Trm)  # make base position for each valid swp points from robot
            Tbm = np.matmul(Tbs, Tsm)
            if (np.all(np.abs(Tsm[:3, 3]) < np.divide(surface.dims[:3], 2)+io_margin)
                    or (xout_cut and (np.abs(Tsm[0, 3]) > np.divide(surface.dims[0],
                                                                    2) + io_margin))):  # mobile loc inside surface
                Tbm_in_all.append(Tbm)
                continue
            if abs(Tbm[2, 3]) > resolution:
                Tbm_float_all.append(Tbm)
                continue
            Tbm[2, 3] = 0
            if ccheck(T_loal=Tbm, Q_dict=crob.home_dict):  # check feasible
                Tsm_swp_pairs.append((Tsm, swp_point))
                Tbm_succ_all.append(Tbm)
                Tsm_xq = T2xyzquat(Tsm)
                Tsm_key = tuple(np.round(Tsm_xq[0], 3)), tuple(np.round(Tsm_xq[1], 3))
                div_base_dict[Tsm_key][i].append(i_div)
#                 gscene.add_highlight_axis("hl", "tbt_{}_{}".format(i_div, i),
#                                           link_name="base_link",
#                                           center=Tbt[:3, 3], orientation_mat=Tbt[:3, :3])
#                 gscene.add_highlight_axis("hl", "tbm_{}_{}_{}".format(i_div, i, i_s),
#                                           link_name="base_link",
#                                           center=Tbm[:3, 3], orientation_mat=Tbm[:3, :3])
            else:
                Tbm_fail_all.append(Tbm)
#     raise(RuntimeError("error"))
#     gscene.clear_highlight()
#     time.sleep(1)
Tsm_keys = sorted(div_base_dict.keys())
# return div_base_dict, Tsm_keys, surface_div_centers, div_num, (ax_step, ax_swp, ax_pln)


# In[ ]:


np.array(sorted(h_set))+Hoff_et


# In[ ]:


h


# In[ ]:


x = {1,2,3}


# In[ ]:


x.update((4,5))


# In[ ]:


x


# In[ ]:


div_heights_r


# In[ ]:


div_heights_r


# In[ ]:


h_in_range


# In[ ]:





# In[ ]:


div_heights_r


# In[ ]:


[pair[1][2] for pair in Tsm_swp_pairs]


# In[ ]:


div_base_dict


# In[ ]:


div_base_dict.values()


# In[ ]:


heights = set()
for sp in swp_points_dict.values():
    heights.update(sp[:,2])


# In[ ]:


heights


# In[ ]:


plt.plot(sweep_max[:,2])
plt.plot(sweep_min[:,2])


# In[ ]:


div_heights_r


# In[ ]:





# In[ ]:


for i_h, h in enumerate(div_heights_r):
    center = np.zeros(3)
    center[2] = h
    gscene.add_highlight_axis("hl", "hs_{}".format(i_h), link_name=ROBOT_BASE, center=center)


# In[ ]:





# In[ ]:



ax_swp_base = ax_swp
Rre = SweepDirections.get_dcm_re(tip_dir)
Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)  ## get data
Rrt = np.matmul(Rre, Tet[:3,:3])
ax_swp_tool = np.where(np.abs(Rrt.transpose()[:,ax_swp_base]).astype(np.int))[0][0]

HOME_POSE_MOVE = np.copy(Q_CUR[6:])
test_fun = TestBaseDivFunc(ppline, floor_ws, surface, ax_swp_tool, ax_swp_base,
                           WP_DIMS, TOOL_DIM, crob.home_dict, tool_dir=tool_dir,
                           multiprocess=PLANNING_MULTIPROC, timeout=0.3, timeout_loop=3)
test_fun.clear()

idx_bases, idc_divs, covered_all, snode_schedule_list = select_max_cover_bases(
    div_base_dict, Tsm_keys, surface_div_centers, div_num, ax_step, 
    test_fun=test_fun, lazy_base_thresh=np.max(TOOL_DIM)/2)


# In[ ]:


div_heights_r


# In[ ]:


div_heights_r


# In[ ]:


div_base_dict


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


snode_schedule_list_down


# In[ ]:


ppline.play_schedule(snode_schedule_list_down[1])


# In[ ]:





# In[ ]:



snode_schedule_list_down, scene_args_list_down, scene_kwargs_list_down, Q_CUR, test_fun =     make_plan_fun(closet_down, Q_CUR, "down", -1, plane_val=None)
snode_schedule_list = snode_schedule_list_leftup + snode_schedule_list_rightup + snode_schedule_list_down
scene_args_list = scene_args_list_leftup + scene_args_list_rightup + scene_args_list_down
scene_kwargs_list = scene_kwargs_list_leftup + scene_kwargs_list_rightup + scene_kwargs_list_down


# ## DEV begin

# In[ ]:


surface, Q_CUR, tip_dir, sweep_dir, tool_dir = closet_leftup, Q_CUR, "up", "Z", 1
plane_val=None
xout_cut=False
ccheck.clear()
div_base_dict, Tsm_keys, surface_div_centers, div_num, (ax_step, ax_swp, ax_pln) =                         get_division_dict(surface, brush_face, robot_config, 
                                          plane_val=plane_val, tip_dir=tip_dir, sweep_dir=sweep_dir,
                                          TOOL_DIM=TOOL_DIM, ccheck=ccheck, 
                                          resolution=0.02, xout_cut=xout_cut)
ax_swp_base = ax_swp
Rre = SweepDirections.get_dcm_re(tip_dir)
Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)  ## get data
Rrt = np.matmul(Rre, Tet[:3,:3])
ax_swp_tool = np.where(np.abs(Rrt.transpose()[:,ax_swp_base]).astype(np.int))[0][0]

HOME_POSE_MOVE = np.copy(Q_CUR[6:])
test_fun = TestBaseDivFunc(ppline, floor_ws, surface, ax_swp_tool, ax_swp_base,
                           WP_DIMS, TOOL_DIM, crob.home_dict, tool_dir=tool_dir,
                           multiprocess=PLANNING_MULTIPROC, timeout=0.3, timeout_loop=3)
test_fun.clear()

idx_bases, idc_divs, covered_all, snode_schedule_list = select_max_cover_bases(
    div_base_dict, Tsm_keys, surface_div_centers, div_num, ax_step, 
    test_fun=test_fun, lazy_base_thresh=np.max(TOOL_DIM)/2)


# In[ ]:


show_base_div(gscene, closet_leftup, surface_div_centers, div_base_dict, VIEW_MOVED_EXT)


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


np.max(TOOL_DIM)/2


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




HOME_POSE_MOVE = np.copy(Q_CUR[6:])
test_fun = TestBaseDivFunc(ppline, floor_ws, surface, WP_DIMS, TOOL_DIM, crob.home_dict, tool_dir=tool_dir,
                              multiprocess=PLANNING_MULTIPROC, 
                              timeout=0.3, timeout_loop=3)
test_fun.clear()

idx_bases, idc_divs, covered_all, snode_schedule_list = select_max_cover_bases(
    div_base_dict, Tsm_keys, surface_div_centers, div_num, ax_step, 
    test_fun=test_fun, lazy_base_thresh=np.max(TOOL_DIM)/2)

snode_schedule_list, idx_bases, idc_divs, scene_args_list, scene_kwargs_list = refine_order_plan(
    ppline, snode_schedule_list, idx_bases, idc_divs, Q_CUR, 
    floor_ws, wayframer, surface, Tsm_keys, surface_div_centers,  
    WP_DIMS, TOOL_DIM, ROBOT_NAME, MOBILE_NAME, HOME_POSE_MOVE)
test_fun.clear()
if len(snode_schedule_list)>0:
    Q_CUR = snode_schedule_list[-1][-1].state.Q


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# ## DEV end

# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# ### 2.2. Execute closet cleaning sequence

# In[ ]:


adjust_list = []
VISUALIZE = True
save_list = []
def adjust_base_on(snode, closet_gtem):
    Qref = snode.state.Q
    traj, succ = get_look_motion(mplan, ROBOT_NAME, Qref, 
                                 target_point=closet_gtem,
                                 com_link = pscene.robot_chain_dict[ROBOT_NAME]['link_names'][-1],
                                 view_dir = [0,0,1],timeout = 1)
    traj_rev = np.array(list(reversed(traj)))

    if not succ:
        traj = [Qref, Qref]
        traj_rev = [Qref, Qref]

    Qref[6:] = traj[-1][6:]
    gscene.show_pose(Qref)
    if CONNECT_INDY:
        with indy: # move to look
            crob.move_joint_traj(traj, one_by_one=True)

        time.sleep(1)

    if ENABLE_DETECT:
        icp_closet = MultiICP(model=MODEL_DIR + '/top_table/top_table.STL', 
                           Toff=SE3([[1,0,0],[0,0,1],[0,-1,0]], [0.3,0,0.2725]))

        save_list.append(1)
        if CONNECT_CAM:
            rdict = stream_capture_image(ImageType.FirstView, 
                                         obj_type="closet_{}".format(len(save_list)), 
                                         host=CAM_HOST, crob=crob)
        else:
            rdict, Qref = load_rdict("closet_{}".format(len(save_list)))

        T_bc, T_bs_closet = viewpoint.get_tf(Qref), gscene.NAME_DICT["closet_vis"].get_tf(Qref)

        cdp = rdict2cdp(rdict)
        pcd_closet = cdp2pcd(cdp, T_bc)


        pcd_masked = mask_boxes(pcd_closet, 
                            boxes=[gscene.NAME_DICT["closet_box"]], 
                            Q=Qref, inside=True, 
                            merge_rule=np.all, link_ref="base_link")
        pcd_masked = mask_boxes(pcd_masked, 
                            boxes=[gscene.NAME_DICT["bed_box"], 
                                   gscene.NAME_DICT["bed_wall"], 
                                   gscene.NAME_DICT["floor_box"]], 
                            Q=Qref, inside=False, 
                            merge_rule=np.all, link_ref="base_link")
        pcd_masked, ind = pcd_masked.remove_radius_outlier(nb_points=20, radius=0.06)
        icp_closet.add_pointcloud(pcd_masked, T_bc)

    #     gscene.show_point_cloud(pcd_closet.points, "allpoints", color=(0,0,0,0.5), dims=(0.01,0.01,0.01), sample_to=500)
    #     gscene.show_point_cloud(pcd_masked.points, "masked", color=(1,0,0,1), dims=(0.02,0.02,0.02))

        initial_guess = gscene.NAME_DICT["closet_vis"].get_tf(Qref)
        T_bo_close, fitness = icp_closet.compute_ICP(initial_guess, thres=0.05, visualize=VISUALIZE)

        # calculate transform based on obtained points
        pcd_center_prev = pcd_masked.get_center()
        pcd_center_transformed_prev = np.matmul(T_bc[:3,:3], pcd_center_prev).transpose() + T_bc[:3,3]

        T_bo_p = SE3(T_bo_close[:3,:3], pcd_center_transformed_prev)
        T_pooc = np.matmul(SE3_inv(T_bo_p), T_bo_close)
        T_bo_p[:3,:3] = Rot_axis(3, Rot2axis(T_bo_close[:3,:3], 3))
        T_bo_c_fix = np.matmul(T_bo_p, T_pooc)
        T_bo_c_fix[2,3] = 0

        # get Twoff from redetection
        Tbo0, Tbo1 = T_bs_closet, T_bo_c_fix

        Tbw0 = wayframer.get_tf_handle(Qref)
        Tow = np.matmul(SE3_inv(Tbo0), Tbw0)
        Tbw1 = np.matmul(Tbo1, Tow)

        Qtar = np.copy(Qref)
        Qtar[:2] = Tbw1[:2,3]
        Qtar[2] = Rot2axis(Tbw1[:3,:3], 3)

        adjust_list.append((kmb.get_qcur(), Qref, Qtar))
        try:
            Qdiff = Qtar - Qref
            Qadj = np.copy(snode.state.Q)
            Qadj[:2] -= Qdiff[:2]
            idx_s = snode_schedule.index(snode)
            snode_schedule_new = ppline.get_updated_schedule(snode_schedule[idx_s:], Qadj, timeout=1)
            ppline.Qdiff = Qdiff
            snode_schedule[idx_s:] = snode_schedule_new
        except:
            ppline.Qdiff = None
#             kmb.joint_move_make_sure(Qtar[:6])

    if CONNECT_INDY:
        with indy: # retrieve motion
            crob.move_joint_traj(traj_rev, one_by_one=True)
        
class SwitchState(Enum):
    NONE = 0
    BASE_MOVED = 1
    SWEEP_APPROACH = 2
    SWEEP_RETRACT = 3
    SWEEPING = 4

class ModeSwitcherKMB:
    def __init__(self, pscene, gtem_ref, push_dist=0.05):
        self.pscene = pscene
        self.crob = pscene.combined_robot
        self.push_dist = push_dist
        self.pushed_before = False
        self.gtem_ref = gtem_ref
        ppline.Qdiff = None
        
    def switch_in(self, snode_pre, snode_new):
        crob.robot_dict["indy1"] = None
        crob.connection_list = (True, False)
        
        switch_state = SwitchState.NONE
        snode_pre = snode_pre.copy(pscene)
        snode_pre.traj = None
        from_state = snode_pre.state
        to_state = snode_new.state
        subjects, ok = pscene.get_changing_subjects(from_state, to_state)
        if len(subjects) == 2: # skip base move case - update schedule
            if ppline.Qdiff is not None:
                Qdiff = ppline.Qdiff
                Qadj = np.copy(snode_pre.state.Q)
                Qadj[:2] -= Qdiff[:2]
                idx_s = snode_schedule.index(snode_pre)
                snode_schedule_new = ppline.get_updated_schedule(snode_schedule[idx_s:], Qadj, timeout=1)
                snode_schedule[idx_s:] = snode_schedule_new
                switch_state = SwitchState.NONE
        if len(subjects) ==0: # joint motion: quit sweep and homing - retract before motion
            if self.pushed_before and self.push_dist > 1e-6:
                switch_state = SwitchState.SWEEP_RETRACT
                from_Q = kmb.get_qcur()
                Tbm = gscene.get_tf(MOBILE_BASE, from_Q)
                Tbm2 = np.matmul(Tbm, SE3(np.identity(3), 
                                          (-self.push_dist, 0, 0)))
                Qto = np.copy(from_Q[:6])
                Qto[:2] = Tbm2[:2,3]
                print("retract")
                kmb.joint_move_make_sure(Qto)
                self.pushed_before = False
        elif subjects[0] == "sweep": # sweep approach or sweeping
            i_swp = pscene.subject_name_list.index("sweep")
            if snode_pre.state.node[i_swp] == 0: # sweep approach - move forward after motoin
                if not self.pushed_before:
                    switch_state = SwitchState.SWEEP_APPROACH
                else:
                    switch_state = SwitchState.NONE
            else: # sweeping
#                 indy.collision_policy = POLICY_NO_COLLISION_DETECTION
                switch_state = SwitchState.SWEEPING
        elif subjects[0] == "waypoints":
            switch_state = SwitchState.BASE_MOVED
        return switch_state

    def switch_out(self, switch_state, snode_new):
        crob.robot_dict["indy1"] = indy
        crob.connection_list = (True, True)
        
        kmb = self.crob.robot_dict['kmb0']
        if switch_state == SwitchState.BASE_MOVED:
            print("adjust")
            adjust_base_on(snode_new, self.gtem_ref)
            t_map = Thread(target=update_map, kwargs=dict(time_wait=2))
            t_map.daemon = True
            t_map.start()
        elif switch_state == SwitchState.SWEEP_APPROACH: # move forward
            if not self.pushed_before and self.push_dist > 1e-6:
                print("push forward")
                from_Q = kmb.get_qcur()
                Tbm = gscene.get_tf(MOBILE_BASE, from_Q)
                Tbm2 = np.matmul(Tbm, SE3(np.identity(3), 
                                          (self.push_dist, 0, 0)))
                Qto = np.copy(from_Q[:6])
                Qto[:2] = Tbm2[:2,3]
                kmb.joint_move_make_sure(Qto)
                self.pushed_before = True
        elif switch_state == SwitchState.SWEEPING:
            pass
#             indy.collision_policy = POLICY_KEEP_PAUSE
                
mode_switcher=ModeSwitcherKMB(pscene, closet_leftup)


# In[ ]:


VEL_LEVEL = 3

if CONNECT_INDY:
    with indy:
        vel_level_bak = indy.get_joint_vel_level()
        print("vel_level_bak: {}".format(vel_level_bak))

    with indy:
        indy.set_joint_vel_level(VEL_LEVEL)
        
    indy.collision_policy = POLICY_NO_COLLISION_DETECTION
swp_fin_list = []


# In[ ]:


swp_fin_list = []
# mode_switcher.push_dist = 0.11
# mode_switcher.push_dist = 0.06
mode_switcher.push_dist = -0.06

for i_s, (snode_schedule, sargs, skwargs) in enumerate(zip(snode_schedule_list, scene_args_list, scene_kwargs_list)):
    print("motions: {}".format(len(snode_schedule[:-1])-1))
    set_base_sweep(*sargs, **skwargs)
#     mode_switcher.gtem_ref = \
#         gscene.NAME_DICT[pscene.subject_dict['sweep'].geometry.parent]
    if CONNECT_INDY and CONNECT_MOBILE:
        ppline.execute_schedule(snode_schedule, one_by_one=True, 
                                mode_switcher=mode_switcher)
    else:
        ppline.play_schedule(snode_schedule)
        
    # leave highlight on cleared area
    swp_fin = gscene.copy_from(gscene.NAME_DICT["sweep"], new_name="sweep_fin_{}".format(i_s), color=(0,0,1,1))
    swp_fin.dims = (swp_fin.dims[0], swp_fin.dims[1], swp_fin.dims[2]+0.002)
    gscene.update_marker(swp_fin)
    swp_fin_list.append(swp_fin)


# ### 2.3. Clear highlight

# In[ ]:


test_fun.clear()
for swp_fin in swp_fin_list:
    gscene.remove(swp_fin)
swp_fin_list = []
pscene.remove_subject(pscene.subject_dict["sweep"])
for child in copy.copy(closet_leftup.children):
    gscene.remove(gscene.NAME_DICT[child])
for child in copy.copy(closet_rightup.children):
    gscene.remove(gscene.NAME_DICT[child])
for child in copy.copy(closet_down.children):
    gscene.remove(gscene.NAME_DICT[child])


# In[ ]:





# ## 3. Bed cleaning

# ### 3.1 Make bed cleaning plan

# #### 3.1.1 make plan

# In[ ]:


from pkg.planning.constraint.constraint_common import *
from pkg.planning.constraint.constraint_actor import *
from pkg.planning.constraint.constraint_subject import *

BED_OFFSET = 0.01
brush_face = pscene.create_binder(bname=TOOL_NAME, gname=TOOL_NAME, _type=SweepFramer, 
                                  point=(0,0,-gscene.NAME_DICT['brush_face'].dims[2]/2-CLEARANCE-BED_OFFSET), 
                                  rpy=(0,0,0))

T_e_brush = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)
T_brush_e = SE3_inv(T_e_brush)
EE_HEIGHT = round(bed_mat.get_tf(HOME_DICT)[2,3] + bed_mat.dims[2]/2, 5)                 + T_brush_e[2, 3] - INDY_BASE_OFFSET[2]

snode_schedule_list, scene_args_list, scene_kwargs_list, Q_CUR, test_fun =     make_plan_fun(bed_mat, Q_CUR, "front", 1, EE_HEIGHT, xout_cut=True)


# ### 3.3 Execute bed cleaning sequence

# In[ ]:


swp_fin_list = []
mode_switcher.push_dist = 0
mode_switcher.gtem_ref = closet_leftup

for i_s, (snode_schedule, sargs, skwargs) in enumerate(zip(snode_schedule_list, scene_args_list, scene_kwargs_list)):
    print("motions: {}".format(len(snode_schedule[:-1])-1))
    set_base_sweep(*sargs, **skwargs)
    if CONNECT_INDY and CONNECT_MOBILE:
        ppline.execute_schedule(snode_schedule, one_by_one=True, 
                                mode_switcher=mode_switcher)
    else:
        ppline.play_schedule(snode_schedule)
        
    # leave highlight on cleared area
    swp_fin = gscene.copy_from(gscene.NAME_DICT["sweep"], new_name="sweep_fin_{}".format(i_s), color=(0,0,1,1))
    swp_fin.dims = (swp_fin.dims[0], swp_fin.dims[1], swp_fin.dims[2]+0.002)
    gscene.update_marker(swp_fin)
    swp_fin_list.append(swp_fin)


# In[ ]:





# ### 3.4 Clear highlight

# In[ ]:


test_fun.clear()
for swp_fin in swp_fin_list:
    gscene.remove(swp_fin)
swp_fin_list = []
pscene.remove_subject(pscene.subject_dict["sweep"])
for child in copy.copy(bed_mat.children):
    gscene.remove(gscene.NAME_DICT[child])


# In[ ]:





# In[ ]:


# save_pickle("adjust_list.pkl", adjust_list)
# save_pickle("Q_all.pkl", [[snode.state.Q for snode in snode_schedule] for snode_schedule in snode_schedule_list])


# ## Load exp dat

# In[ ]:


# adjust_list = load_pickle("adjust_list.pkl")
# gtem_args = load_pickle("gtem_args.pkl")
# Q_all = load_pickle("Q_all.pkl")


# In[ ]:


# for garg in gtem_args:
#     if garg['parent'] == None and garg['link_name']=="base_link":
#         if garg["name"] in gscene.NAME_DICT:
#             gscene.NAME_DICT[garg["name"]].set_offset_tf(center=garg["center"], orientation_mat=Rot_rpy(garg["rpy"]))
# gscene.update_markers_all()        


# In[ ]:


# VISUALIZE = True
# adjust_base_on(Q_all[4][0], closet_leftup)


# In[ ]:





# In[ ]:





# In[ ]:




