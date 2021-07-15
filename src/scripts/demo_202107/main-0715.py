#!/usr/bin/env python
# coding: utf-8
# 

# In[1]:


import os
import sys
os.chdir(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/demo_202107'))

from pkg.global_config import RNB_PLANNING_DIR
from demo_utils.kiro_udp_send import start_mobile_udp_thread, send_pose_wait, get_xyzw_cur, get_reach_state
from pkg.utils.utils import *    
from pkg.utils.rotation_utils import *
from pkg.controller.combined_robot import *
from pkg.project_config import *
from demo_utils.streaming import *
from demo_utils.detect_table import *
from demo_utils.area_select import *


CONNECT_CAM = False
ENABLE_DETECT = True
CONNECT_INDY = False
CONNECT_MOBILE = False

ip_cur =  get_ip_address()
MOBILE_IP = "192.168.0.102"
INDY_IP = "192.168.0.3"
CAM_HOST = '192.168.0.10'

print("Current PC IP: {}".format(ip_cur))
print("Mobile ROB IP: {}".format(MOBILE_IP))
print("CAM SERVER IP: {}".format(CAM_HOST))


# ## Set table dimension

# In[2]:


# Table dimension
T_Width = 1.8
T_Height = 0.785
T_Depth = 0.734
TOOL_DIM = [0.32, 0.08]
ROBOT_Z_ANGLE = np.pi
MARGIN = 0
OFFSET_MOBILE=[0.172, 0, 0.439]


# In[ ]:





# ## Prepare robot and pipeline setting

# In[3]:


from pkg.geometry.builder.scene_builder import SceneBuilder
from demo_utils.environment import *

sock_mobile, server_thread = start_mobile_udp_thread(recv_ip=ip_cur)

robot_config = RobotConfig(0, RobotType.indy7, None, INDY_IP, specs={"no_sdk":True})
ROBOT_NAME = robot_config.get_indexed_name()
crob = CombinedRobot(robots_on_scene=[robot_config]
              , connection_list=[CONNECT_INDY])

s_builder = SceneBuilder(None)
# s_builder.reset_reference_coord(ref_name="floor")

# xyz_rpy_robots = s_builder.detect_items(level_mask=[DetectionLevel.ROBOT])
xyz_rpy_robots = {"indy0": ((0,0,0), (0,0,ROBOT_Z_ANGLE))}
crob.update_robot_pos_dict(xyz_rpy_robots=xyz_rpy_robots)
gscene = s_builder.create_gscene(crob)

gtems = s_builder.add_robot_geometries(color=(0,1,0,0.5), display=True, collision=True)
gscene.set_workspace_boundary( -1.5, 1.5, -1, 1, -0.1, 1.75)

add_env(gscene)
viewpoint = add_cam(gscene, tool_link="indy0_tcp")
add_indy_tool_kiro(gscene, tool_link="indy0_tcp", face_name="brush_face", zoff=-0.04)

from pkg.planning.scene import PlanningScene
pscene = PlanningScene(gscene, combined_robot=crob)
BASE_LINK = "base_link"
TIP_LINK = pscene.robot_chain_dict[ROBOT_NAME]["tip_link"]

from pkg.planning.pipeline import PlanningPipeline
ppline = PlanningPipeline(pscene)

from pkg.ui.ui_broker import *

# start UI
ui_broker = UIBroker.instance()
ui_broker.initialize(ppline, s_builder)
ui_broker.start_server()

ui_broker.set_tables()

# Register binders
from pkg.planning.constraint.constraint_actor import Gripper2Tool, PlacePlane, SweepFramer, FixtureSlot
brush_face = pscene.create_binder(bname="brush_face", gname="brush_face", _type=SweepFramer, point=(-gscene.NAME_DICT['brush_face'].dims[0]/2,0,0), 
                     rpy=(0,np.pi/2*1,0))

# Set planner
from pkg.planning.motion.moveit.moveit_planner import MoveitPlanner
mplan = MoveitPlanner(pscene)
mplan.update_gscene()
from pkg.planning.task.rrt import TaskRRT
tplan = TaskRRT(pscene)
tplan.prepare()
ppline.set_motion_planner(mplan)
ppline.set_task_planner(tplan)

from pkg.planning.filtering.grasp_filter import GraspChecker
from pkg.planning.filtering.reach_filter import ReachChecker
from pkg.planning.filtering.latticized_filter import LatticedChecker
from pkg.planning.filtering.task_clearance_filter import TaskClearanceChecker

# gcheck = GraspChecker(pscene)
# rcheck = ReachChecker(pscene)
# tcheck = TaskClearanceChecker(pscene, gcheck)
# checkers_all = [tcheck, rcheck, gcheck]
# # lcheck = LatticedChecker(pscene, gcheck)
# # checkers_all.append(lcheck)

# mplan.motion_filters = checkers_all

indy = crob.robot_dict["indy0"]
# if CONNECT_INDY:
#     with indy:
#         indy.reset_robot()


# In[ ]:





# ## move indy to viewing pose

# In[4]:


VIEW_POSE = np.deg2rad([  0., -28.,  85.,  -0.,  57., -180])
if CONNECT_INDY:
    with indy:
        indy.joint_move_to(np.rad2deg(VIEW_POSE))
        time.sleep(0.5)
        indy.wait_for_move_finish()
        Qcur = np.deg2rad(indy.get_joint_pos())
else:
    Qcur = VIEW_POSE
gscene.show_pose(Qcur)


# In[ ]:





# ## Attach to detection server

# In[5]:


if ENABLE_DETECT:
    attacth_to_server()


# In[ ]:





# ## Get image

# In[6]:


if CONNECT_CAM:
#     rdict = send_recv_demo_cam({1:1}, host=CAM_HOST)
    rdict = stream_capture_image(ImageType.FirstView, host=CAM_HOST)
    cam_intrins, d_scale = [rdict[key] for key in ["intrins", "depth_scale"]]
else:
    cam_intrins = [1280, 720, 899.05322265625,  899.21044921875, 654.8836669921875, 352.9295654296875]
    d_scale = 0.0002500000118743628
    # Set color, depth image path
    
if CONNECT_CAM:
    color_img_path = SAVE_DIR + '/color.jpg'
    depth_img_path = SAVE_DIR + '/depth.png'
else:
    color_img_path = DATASET_CAM_DIR + '/color.jpg'
    depth_img_path = DATASET_CAM_DIR + '/depth.png'

# Read color, depth image file, keep 16bit information
color_img_read = cv2.imread(color_img_path, flags=cv2.IMREAD_UNCHANGED)
depth_img_read = cv2.imread(depth_img_path, flags=cv2.IMREAD_UNCHANGED)

# Output of inference(mask for detected table)
mask_out = detect_from_server(color_img_read)
ICP_result1 = None
if mask_out is not None:
    plt.imshow(mask_out)
    # Crop masking part
    vis_mask = (mask_out * 255).astype('uint8')
    color_instance = cv2.bitwise_and(color_img_read, color_img_read, mask=vis_mask).astype(np.uint16)
    depth_instance = cv2.bitwise_and(depth_img_read, depth_img_read, mask=vis_mask).astype(np.uint16)
    cv2.imwrite(CROP_DIR + '/color_crop.jpg', color_instance)
    cv2.imwrite(CROP_DIR + '/depth_crop.png', depth_instance)
    
    set_cam_params(cam_intrins, d_scale)
    model_mesh, pcd_out = preprocessing()
    ICP_result1 = compute_ICP(model_mesh, pcd_out, (-T_Height, -T_Depth, 0.0))
    #ICP_result2 = compute_ICP(model_mesh, pcd2)
    
if ICP_result1 is None: # test
    ICP_result1 = np.array([[ 0.97952723,  0.00354742,  0.20128047, -0.63449415],
                            [ 0.08693341,  0.89435887, -0.43882204, -0.18267728],
                            [-0.18157366,  0.44733614,  0.87574048,  1.77040063],
                            [ 0.        ,  0.        ,  0.        ,  1.        ]])


# In[7]:


gscene.show_pose(VIEW_POSE)
viewpoint.draw_traj_coords([VIEW_POSE])


# In[8]:


TABLE_DIMS = np.array((T_Depth,T_Width,T_Height))
OFF_DIR = np.array([1,1,-1])
# OFF_DIR = np.array([1,1,0])
#TABLE_DIMS[[0,1,2]]
#np.hstack([TABLE_DIMS[[0,1]], [0]])
T_toff = SE3(Rot_axis(3,np.pi), np.divide(TABLE_DIMS[[0,1,2]]*OFF_DIR,2))
T_co = np.matmul(np.matmul(ICP_result1, SE3(Rot_axis(1, np.pi/2), [0]*3)), T_toff)
T_lo = np.matmul(viewpoint.Toff, T_co)
T_bc = viewpoint.get_tf(list2dict(VIEW_POSE, gscene.joint_names))
T_bo=np.matmul(T_bc, T_co)

# fit to plane
floor_g = gscene.NAME_DICT["floor"]
floor_height = floor_g.center[2] + floor_g.dims[2]/2

# Floor Fitting
T_bo[2,3] =  floor_height+TABLE_DIMS[2]/2

# Depth scaling
# T_bfcenter = SE3(np.identity(3), [0,0,floor_height+TABLE_DIMS[2]/2])
# T_fc = np.matmul(SE3_inv(T_bfcenter), T_bc)
# T_fo = np.matmul(SE3_inv(T_bfcenter), T_bo)
# dz_cam = T_fc[2,3]
# dz_obj = -T_fo[2,3]
# dz_co = dz_cam + dz_obj
# depth_newscale = dz_cam/dz_co
# T_co_new = T_co.copy()
# T_co_new[:3,3] *= depth_newscale
# T_bo_new = np.matmul(T_bc, T_co_new)
# T_co = T_co_new
# T_bo = T_bo_new

azimuth, zenith = mat2hori(T_bo[:3,:3])
zenith_up = np.pi-zenith
Raz = Rot_axis(3, azimuth)
Rzn = Rot_axis(2, -zenith_up)
Roff = np.matmul(np.matmul(Raz, Rzn), Raz.transpose())
T_bo[:3,:3] = np.matmul(Roff, T_bo[:3,:3])

gscene.add_highlight_axis("table", "center", link_name="base_link", center=T_bo[:3,3], orientation_mat=T_bo[:3,:3])

# geometry 추가
table = gscene.create_safe(gtype=GEOTYPE.BOX, name="table", link_name="base_link", 
                   dims=TABLE_DIMS, center=T_bo[:3,3], rpy=Rot2rpy(T_bo[:3,:3]), 
                   color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=False)


# In[ ]:





# ## select task area

# In[9]:


TABLE_HEIGHT = floor_height+TABLE_DIMS[2]
T_e_brush = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)
T_brush_e = SE3_inv(T_e_brush)
P_floor_e = T_brush_e[:3,3] + [0,0,TABLE_HEIGHT]
TOOL_DEPTH_MIN = 0.6
TOOL_DEPTH_MAX = 1.0
EE_DEPTH_OFF, _, EE_HEIGHT = P_floor_e
print("reference height: {}".format(EE_HEIGHT))
if ROBOT_Z_ANGLE == np.pi:
    flipper = -1
elif ROBOT_Z_ANGLE == 0:
    flipper = 1
else:
    raise(NotImplementedError("Unexpected robot orientation"))

sweep_data = load_sweep_data(robot_config.type.name).reshape((1,))[0]
range_list_dict, best_range_dict = sweep_data['range_list_dict'], sweep_data['best_range_dict']
heights = sorted(set([key[2] for key in best_range_dict.keys()]))
i_high = np.where(np.subtract(heights, EE_HEIGHT)>=0)[0][0]
i_low = i_high-1
h_high = heights[i_high]
h_low = heights[i_low]

match_range_dict_high = defaultdict(lambda:(-1e5, 1e5))
match_range_dict_low = defaultdict(lambda:(-1e5, 1e5))
match_range_all = {}
for key, brange in best_range_dict.items():
    hkey = round(key[2], 4)
    key_new = round(flipper*key[0], 4)
    if hkey == h_low: # get minimum range from the upper and lower layers
        brange_new = tuple(reversed(np.multiply(flipper,brange)))
        range_old = -np.subtract(*match_range_dict_low[key_new])
        range_new = -np.subtract(*brange_new)
        if range_new < range_old:
            match_range_dict_low[key_new] = brange_new
            match_range_all[key_new] = range_list_dict[key]
    if hkey == h_high: # get minimum range from the upper and lower layers
        brange_new = tuple(reversed(np.multiply(flipper,brange)))
        range_old = -np.subtract(*match_range_dict_high[key_new])
        range_new = -np.subtract(*brange_new)
        if range_new < range_old:
            match_range_dict_high[key_new] = brange_new
            match_range_all[key_new] = range_list_dict[key]

match_range_dict = {}
best_keys = set(match_range_dict_high.keys()).intersection(match_range_dict_low.keys())
for key in best_keys:
    range_high = match_range_dict_high[key]
    range_low = match_range_dict_high[key]
    match_range_dict[key] = range_high if -np.subtract(*range_high) < -np.subtract(*range_low) else range_low
                
            
# plot available area
plt.figure(figsize=(15,5))
plt.subplot(1,2,1)
plt.title("best only")
for k,v in match_range_dict.items():
    plt.plot(v, [k, k], '-s')
plt.axis([-1,1,0.2,1.1])
plt.grid()
plt.subplot(1,2,2)
for k,v in match_range_all.items():
    for vv in v:
        plt.plot(vv, [k, k], '-o')
plt.title("all ranges")
plt.axis([-1,1,0.2,1.1])
plt.grid()

DEPTH_MIN = TOOL_DEPTH_MIN+EE_DEPTH_OFF
DEPTH_MAX = TOOL_DEPTH_MAX+EE_DEPTH_OFF
division_dict_1 = get_division_dict(match_range_dict, 1, TABLE_DIMS, TOOL_DIM, 
                                    DEPTH_MIN=DEPTH_MIN, DEPTH_MAX=DEPTH_MAX, MARGIN=MARGIN)
division_dict_2 = get_division_dict(match_range_dict, 2, TABLE_DIMS, TOOL_DIM, 
                                    DEPTH_MIN=DEPTH_MIN, DEPTH_MAX=DEPTH_MAX, MARGIN=MARGIN)

divisions1_sorted = sorted(division_dict_1.items(), key=lambda item_: item_[1][-1])
divisions2_sorted = sorted(division_dict_2.items(), key=lambda item_: item_[1][-1])
divisions_sorted = sorted(division_dict_1.items()+division_dict_2.items(), key=lambda item_: item_[1][-1])
assert len(divisions_sorted) > 0, "no available table division solution"

division_sol = divisions_sorted[0]
depths = division_sol[0]
sweep_width, (area_width, area_depth), width_range, divisions, div_num = division_sol[1]
corner_center = ((max(*depths)+min(*depths))/2-EE_DEPTH_OFF, np.mean(width_range))
print("sweep depths: {}".format(depths))
print("divisions: {}".format(divisions))


# In[ ]:





# # Main Loop

# In[10]:


table_front = table
for i_cn in range(4):
    cn_cur = CornerRev[i_cn]
#     raise(NotImplementedError("copy loop to here"))

    print("== Current workspace section: {}".format(cn_cur.name))


    # ## Add table on relative target location

    # In[38]:


    from pkg.geometry.geotype import GEOTYPE

    table = table_front
    track = gscene.create_safe(GEOTYPE.BOX, "track", "base_link", (abs(area_depth),(area_width),0.01),
                               tuple(corner_center)+(TABLE_HEIGHT-0.005,), rpy=(0,0,0),
                               color=(0.0,0.8,0.8,0.2), display=True, fixed=True, collision=True)
    track_face = gscene.copy_from(track, new_name="track_face", collision=False, color=(0.8,0.8,0.8,0.0))
    TRACK_DIM = np.copy(track_face.dims)
    track_face.dims = (3, 3, track.dims[2])
    gscene.update_markers_all()


    section_size = np.concatenate([np.divide(table.dims[:2],  divisions), [track.dims[2]]])
    off_corner = np.divide(table.dims, 2)*corner_point_dirs[cn_cur]
    off_sect = (np.divide(table.dims, 2) - np.divide(section_size, 2))*corner_point_dirs[cn_cur]
    T_tf = SE3(corner_orientations[cn_cur], off_sect)
    T_ft = SE3_inv(T_tf)
    T_bf = track.Toff
    T_bt = np.matmul(T_bf, T_ft)
    T_tb = SE3_inv(T_bt)
    ## 6 DoF pose to go
    T_bb2 = np.matmul(table.Toff, T_tb)
    gscene.add_highlight_axis("target", "base", "base_link", T_bb2[:3,3], T_bb2[:3,:3])

    T_mb = SE3(Rot_axis(3, 0), OFFSET_MOBILE)
    T_bm = SE3_inv(T_mb)
    T_mm2 = np.matmul(SE3_inv(T_bm), np.matmul(T_bb2, SE3_inv(T_mb)))
    T_bm2 = np.matmul(T_bm, T_mm2)
    gscene.add_highlight_axis("target", "mobile", "base_link", T_bm2[:3,3], T_bm2[:3,:3])


    # In[ ]:





    # ## get_relative mobile pose

    # In[39]:


    xy_rel = T_mm2[:2,3]
    quat_rel = Rotation.from_dcm(T_mm2[:3, :3]).as_quat()
    quat_zw_rel = quat_rel[-2:]
    xyzw_rel = np.concatenate([xy_rel, quat_zw_rel])

    if CONNECT_MOBILE:
        cur_xyzw = get_xyzw_cur()
    else:
        cur_xyzw = (0,0,0,1)

    T_approach = SE3(np.identity(3), [-0.4,0,0])
    T_am_cur = T_xyzquat((cur_xyzw[:2]+(0,), (0,0)+cur_xyzw[2:]))
    T_am_to = np.matmul(T_am_cur, T_mm2)
    T_am_to_ready = np.matmul(T_am_to, T_approach)

    tar_xy = T_am_to[:2,3].tolist()
    tar_qzw = Rotation.from_dcm(T_am_to[:3,:3]).as_quat()[-2:].tolist()

    tar_xy_rd = T_am_to_ready[:2,3].tolist()
    tar_qzw_rd = Rotation.from_dcm(T_am_to_ready[:3,:3]).as_quat()[-2:].tolist()

    tar_xyzw = tar_xy+tar_qzw
    tar_xyzw_rd = tar_xy_rd+tar_qzw_rd
    # raise(NotImplementedError("tar_xyzw: relative? absolute?"))
    print("curre  xyzw: {}".format(np.round(cur_xyzw, 2)))
    print("ready  xyzw: {}".format(np.round(tar_xyzw_rd, 2)))
    print("target xyzw: {}".format(np.round(tar_xyzw, 2)))


    # ```
    # curre  xyzw: [2.76 1.22 0.86 0.51]
    # ready  xyzw: [3.15 3.56 1.   0.01]
    # target xyzw: [2.75 3.57 1.   0.01]
    # ```

    # ## move mobile robot

    # In[40]:


    if CONNECT_MOBILE:
        delta_xyzw = np.subtract(tar_xyzw, cur_xyzw)
        if (cn_cur in [Corners.FrontRight, Corners.BackRight]
            and np.linalg.norm(delta_xyzw)<2.0
           ):
            move_steps = int(ceil(np.linalg.norm(delta_xyzw) / 0.7))
            xyzw_step = delta_xyzw/move_steps
            for i_stp in range(move_steps):
                if i_stp == move_steps-1:
                    tar_cur = tar_xyzw
                else:
                    tar_cur = np.add(cur_xyzw, xyzw_step*(i_stp+1))
                cur_xyzw = send_pose_wait(sock_mobile,
                                          tar_cur,
                                          send_ip=MOBILE_IP)
        else:
            cur_xyzw = send_pose_wait(sock_mobile, tar_xyzw_rd, send_ip=MOBILE_IP)
            cur_xyzw = send_pose_wait(sock_mobile, tar_xyzw, send_ip=MOBILE_IP)

    # # Go view loc
    # cur_xyzw = send_pose_wait(sock_mobile, [2.77, 1.,   0.86, 0.51], send_ip=MOBILE_IP)


    # ## update table location

    # In[41]:


    T_bt_front = np.matmul(T_bt, SE3(corner_orientations[cn_cur], [0,0,0]))
    ## add moved table in front of robot
    table_front = gscene.create_safe(GEOTYPE.BOX, "table_front", "base_link",
                                     dims=table.dims,
                                     center=T_bt_front[:3,3],
                                     rpy=Rot2rpy(T_bt_front[:3,:3]),
                                     color=(0.8,0.8,0.8,0.8), display=True, fixed=True, collision=True)

    table.color = (0.8,0.8,0.8,0.0)
    gscene.update_markers_all()

    # corner_center_rel = np.abs(np.subtract(tuple(corner_center)+(TABLE_HEIGHT,), table_front.center))


    # In[ ]:





    # ### calc gaze pose

    # In[42]:


    if CONNECT_INDY:
        with indy:
            Qcur = np.deg2rad(indy.get_joint_pos())
    else:
        Qcur = VIEW_POSE

    GAZE_DIST = 0.5

    if cn_cur in [Corners.BackLeft, Corners.FrontLeft]:
        off_corner_gaze = np.divide(table.dims, 2)*corner_point_dirs[Corners.FrontLeft]
    elif cn_cur in [Corners.BackRight, Corners.FrontRight]:
        off_corner_gaze = np.divide(table.dims, 2)*corner_point_dirs[Corners.FrontRight]
    else:
        raise(NotImplementedError("Unexpected corner"))
    T_tc = SE3(np.identity(3), off_corner_gaze)
    T_bc = np.matmul(T_bt_front, T_tc)

    if cn_cur in [Corners.FrontLeft, Corners.BackLeft]:
        T_cam_c_list = [SE3(Rot_rpy((0,-np.pi*3/4,np.pi/2)) , (0,0,GAZE_DIST))]
    elif cn_cur in [Corners.FrontRight, Corners.BackRight]:
        T_cam_c_list = [SE3(Rot_rpy((np.pi/8,-np.pi*7/8,np.pi/2)) , (0,0,GAZE_DIST))]
    for T_cam_c in T_cam_c_list:
        T_bcam = np.matmul(T_bc, SE3_inv(T_cam_c))
        T_ecam = viewpoint.Toff
        T_be = np.matmul(T_bcam, SE3_inv(T_ecam))
        gscene.add_highlight_axis("cam", "effector", "base_link", T_be[:3, 3], T_be[:3,:3])
        gaze_traj, success = mplan.planner.plan_py("indy0", "indy0_tcp", np.concatenate(T2xyzquat(T_be)), "base_link", Qcur)
        if success:
            gaze_pose = gaze_traj[-1]
            break

    print("GAZE: {}".format(success))
    if success:
        gscene.show_motion(gaze_traj)


    # In[ ]:





    # ## Move to gaze pose

    # In[43]:


    if CONNECT_INDY:
        indy.move_joint_wp(gaze_traj, None, None, None)

    # ## return from gaze pose
    # if CONNECT_INDY:
    #     indy.move_joint_wp(np.array(list(reversed(gaze_traj))), None, None, None)


    # In[ ]:





    # ## Refine plane

    # In[44]:


    from demo_utils.detect_table import *

    if CONNECT_CAM:
        # 모서리부분 근접 촬영
        rdict = stream_capture_image(ImageType.CloseView, host=CAM_HOST)
        set_cam_params(rdict['intrins'], rdict['depth_scale'])
        img_path = SAVE_DIR + '/table.png'
    else:
        img_path = DATASET_CAM_DIR + "/table_11.png"

    if CONNECT_INDY:
        with indy:
            Qcur = indy.get_qcur()
    else:
        try:
            Qcur = np.load(DATASET_CAM_DIR + '/tablepose_11.npy')
        except Exception as e:
            print(e)

    if CONNECT_CAM:
        p_inliers = get_inliers(img_path)

        T_bc = viewpoint.get_tf(list2dict(Qcur, gscene.joint_names))
        viewpoint.draw_traj_coords([Qcur])
        x_bo, y_bo = point_proj(T_bc, p_inliers)

        from pkg.utils.rotation_utils import *
        if cn_cur in [Corners.BackLeft, Corners.FrontLeft]:
            T_bo = left_corner(x_bo, y_bo)
        elif cn_cur in [Corners.BackRight, Corners.FrontRight]:
            T_bo = right_corner(x_bo, y_bo)

        gscene.add_highlight_axis("table", "center", link_name="base_link", center=T_bo[:3,3], orientation_mat=T_bo[:3,:3])

    else:
        T_bo = np.matmul(track.Toff, T_ft)

    T_bo = np.matmul(T_bo, SE3(np.identity(3), (0,0,0)))
    # geometry
    table = gscene.create_safe(gtype=GEOTYPE.BOX, name="table_front", link_name="base_link",
                       dims=TABLE_DIMS, center=T_bo[:3,3], rpy=Rot2rpy(T_bo[:3,:3]),
                       color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=False)


    # In[ ]:





    # ## adjust track

    # In[45]:


    from pkg.geometry.geotype import GEOTYPE
    T_tf = SE3_inv(T_ft)
    track = gscene.create_safe(GEOTYPE.BOX, "track", "base_link", section_size,
                               center = T_tf[:3,3], #corner_center_rel*corner_point_dirs[cn_cur],
                               rpy= Rot2rpy(T_tf[:3,:3]), #Rot2rpy(corner_orientations[cn_cur]),
                               color=(0.0,0.8,0.8,0.2), display=True, fixed=True, collision=True,
                               parent="table_front")
    track_face = gscene.copy_from(track, new_name="track_face", collision=False, color=(0.8,0.8,0.8,0.2))
    TRACK_DIM = np.copy(track_face.dims)
    track_face.dims = (3, 3, track.dims[2])
    gscene.update_markers_all()

    width_range_fit = (np.mean(width_range)-section_size[1]/2, np.mean(width_range)+section_size[1]/2)
    sweep_list, track_list = make_work_plane(pscene, TRACK_DIM, TOOL_DIM, EE_DEPTH_OFF, depths, width_range_fit, MARGIN=MARGIN)


    # In[46]:


    from pkg.planning.constraint.constraint_common             import sample_redundancy, combine_redundancy
    gtimer = GlobalTimer.instance()
    # initial_state = pscene.initialize_state(crob.home_pose)
    initial_state = pscene.initialize_state(VIEW_POSE)
    print(initial_state.node)

    # # remove place points except for the current one
    # use_current_place_point_only(pscene, initial_state)


    # In[ ]:





    # In[47]:


    from pkg.utils.traj_utils import simplify_schedule, mix_schedule
    mplan.reset_log(False)
    gtimer.reset()
    tplan.prepare()
    mplan.update_gscene()

    print(initial_state.node)

    obj_num = 0
    sweep_num = len(sweep_list)
    from_state = initial_state.copy(pscene)
    from_state.Q = np.array([0]*6)
    # from_state.Q = np.array([ 0.        , -0.48869219,  1.48352986, -0.        ,  0.99483767,
    #        3.14159265])
    t_exe = None
    snode_schedule_all = []
    for sweep_idx in range(sweep_num):
    #     gcheck.put_banned = [track_list[sweep_idx][2]]
        sweep_goal = tuple([int(i_s<=sweep_idx)*2 for i_s in range(sweep_num)])
    #     sweep_goal = tuple([int(i_s<=sweep_idx)*2 for i_s in range(2)])+(0,)
        goal_nodes = [("track_face",)*obj_num+sweep_goal]
        if sweep_idx < sweep_num-1:
            for i_s in range(obj_num):
                obj_goal = ["track_face"]*obj_num
                obj_goal[i_s] = "grip1"
                goal_nodes += [tuple(obj_goal)+sweep_goal]
        gtimer.tic("plan{}".format(sweep_idx))
        ppline.search(from_state, goal_nodes, verbose=True, display=False, dt_vis=0.01,
                      timeout_loop=20, multiprocess=False, timeout=1, timeout_constrained=2,
                      add_homing=False, post_optimize=False)
        gtimer.toc("plan{}".format(sweep_idx))
        schedules = ppline.tplan.find_schedules(False)
        schedules_sorted = ppline.tplan.sort_schedule(schedules)
        snode_schedule = ppline.tplan.idxSchedule2SnodeScedule(schedules_sorted[0])
        if sweep_idx == 0:
            snode_start = snode_schedule[1]
            pscene.set_object_state(snode_schedule[0].state)
            trajectory, success = mplan.planner.plan_joint_motion_py(
                ROBOT_NAME, tuple(snode_start.traj[-1]), tuple(gaze_pose), timeout=1)
            if success:
                snode_start.traj = trajectory
        if sweep_idx == sweep_num-1:
            added_list = ppline.add_return_motion(snode_schedule[-1], initial_state=initial_state, timeout=0.5, try_count=2)
            snode_schedule += added_list
        snode_schedule_ori = snode_schedule
        snode_schedule_simple = simplify_schedule(pscene, snode_schedule)
    #     snode_schedule_safe = calculate_safe_schedule(pscene, snode_schedule_simple, 5, 1)
    #     double_sweep_motions(snode_schedule_safe)
    #     snode_schedule = snode_schedule_safe
    #     snode_schedule = mix_schedule(mplan, snode_schedule_safe)
        snode_schedule = snode_schedule_simple
        from_state = snode_schedule[-1].state
        snode_schedule_all.append(snode_schedule)


    # In[ ]:





    # ## Refine sweep motion

    # In[48]:


    from demo_utils.refine_sweep import simplify_sweep
    for snode_schedule in snode_schedule_all:
        simplify_sweep(pscene, mplan, snode_schedule, len_traj=20)


    # ## Play plan

    # In[51]:


    for snode_schedule in snode_schedule_all:
        ppline.play_schedule(snode_schedule, period=0.1)


    # In[ ]:





    # ## Execute plan

    # In[33]:


    if CONNECT_INDY:
        for snode_schedule in snode_schedule_all:
            ppline.execute_schedule(snode_schedule, one_by_one=True)
            with indy:
                time.sleep(0.5)
                indy.wait_for_move_finish()

        with indy:
            indy.joint_move_to(np.rad2deg(VIEW_POSE))
            time.sleep(0.5)
            indy.wait_for_move_finish()


    # In[ ]:



