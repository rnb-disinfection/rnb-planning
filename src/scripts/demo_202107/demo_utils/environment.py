import numpy as np
from pkg.geometry.geotype import GEOTYPE
from pkg.planning.constraint.constraint_actor import Gripper2Tool, PlacePlane, SweepFramer, FixtureSlot
from pkg.planning.constraint.constraint_common import MotionConstraint
from pkg.planning.constraint.constraint_subject import AbstractTask, AbstractObject
from pkg.planning.constraint.constraint_subject import SweepLineTask
from pkg.planning.constraint.constraint_subject import SweepFrame
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *
from area_select import *
from kiro_udp_send import *
import time

def add_env(gscene):
    mobile_base = gscene.create_safe(gtype=GEOTYPE.BOX, name="mobile_base", link_name="base_link", 
                       dims=(0.6,0.4,0.439), center=(0,0,-0.439/2), rpy=(0,0,0), 
                       color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=False)
    floor = gscene.create_safe(gtype=GEOTYPE.BOX, name="floor", link_name="base_link", 
                       dims=(10,10,0.01), center=(0,0,-0.439), rpy=(0,0,0), 
                       color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=False)

def add_cam(gscene, tool_link="indy0_tcp"):
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="cam", link_name=tool_link,
                       dims=(0.061, 0.061, 0.026), center=(-0.0785, 0, 0.013), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 0.5), display=True, fixed=True, collision=False)

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="cam_col", link_name=tool_link,
                       dims=(0.081, 0.081, 0.046), center=(-0.0785, 0, 0.013), rpy=(0, 0, 0),
                       color=(0.0, 1, 0, 0.3), display=True, fixed=True, collision=True)

    viewpoint = gscene.create_safe(gtype=GEOTYPE.SPHERE, name="viewpoint", link_name=tool_link,
                                   dims=(0.01, 0.01, 0.01), center=(-0.013, 0, 0), rpy=(0, 0, -np.pi / 2),
                                   color=(1, 0, 0, 0.3), display=True, fixed=True, collision=False, parent="cam")

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="body", link_name=tool_link,
                       dims=(0.067, 0.067, 0.0335), center=(-0.0785, 0, -0.01675), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=False)

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="body_col", link_name=tool_link,
                       dims=(0.087, 0.087, 0.0535), center=(-0.0785, 0, -0.01675), rpy=(0, 0, 0),
                       color=(0.0, 1, 0, 0.3), display=True, fixed=True, collision=True)

    gscene.create_safe(gtype=GEOTYPE.SPHERE, name="backhead", link_name=tool_link,
                       dims=(0.067, 0.067, 0.067), center=(-0.0785, 0, -0.0335), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=False)

    gscene.create_safe(gtype=GEOTYPE.SPHERE, name="backhead_col", link_name=tool_link,
                       dims=(0.087, 0.087, 0.087), center=(-0.0785, 0, -0.0335), rpy=(0, 0, 0),
                       color=(0.0, 1, 0, 0.3), display=True, fixed=True, collision=True)
    return viewpoint

def add_kiro_indytool_down(gscene, zoff=0, tool_link="indy1_tcp", face_name="brush_face"):
    gscene.create_safe(gtype=GEOTYPE.MESH, name="indy_tool_vis", link_name=tool_link,
                       dims=(0.1,0.1,0.1), center=(0,0,0), rpy=(0,0,np.pi),
                       display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       uri="package://my_mesh/meshes/stl/kiro_indytool_down_res.STL")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="hinge_bar_col", link_name=tool_link,
                       center=(0.1, 0, 0.12), dims=(0.05, 0.070, 0.35), rpy=(0, np.pi / 4, 0),
                       display=True, color=(0.0, 0.8, 0.0, 0.5), collision=True, fixed=True)
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name=tool_link,
                       center=(0.32+zoff, 0, 0.236), dims=(0.34, 0.10, 0.01), rpy=(0, -np.pi/2, 0),
                       color=(1.0, 0.0, 0.0, 0.5),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name=tool_link,
                       center=(0.275+zoff, 0, 0.236), dims=(0.34, 0.10, 0.08), rpy=(0, -np.pi/2, 0),
                       color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)
    return brush_face

def add_kiro_indytool_up(gscene, zoff=0, tool_link="indy1_tcp", face_name="brush_face"):
    gscene.create_safe(gtype=GEOTYPE.MESH, name="indy_tool_vis", link_name=tool_link,
                       dims=(0.1,0.1,0.1), center=(0,0,0), rpy=(0,0,np.pi),
                       display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       uri="package://my_mesh/meshes/stl/kiro_indytool_up_res.STL")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="hinge_bar_col", link_name=tool_link,
                       center=(0.1, 0, 0.12), dims=(0.05, 0.070, 0.35), rpy=(0, np.pi / 4, 0),
                       display=True, color=(0.0, 0.8, 0.0, 0.5), collision=True, fixed=True)
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name=tool_link,
                       center=(0.18, 0, 0.345+zoff), dims=(0.34, 0.10, 0.01), rpy=(0, -np.pi, 0),
                       color=(1.0, 0.0, 0.0, 0.5),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name=tool_link,
                       center=(0.18, 0, 0.30+zoff), dims=(0.36, 0.12, 0.08), rpy=(0, -np.pi, 0),
                       color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)
    return brush_face

def add_bed(gscene, bed_center, bed_rpy, COLOR_BED_COL, add_back_wall=True):
    bed_vis = gscene.create_safe(GEOTYPE.MESH, "bed_vis", link_name="base_link",
                                 dims=(0.1,0.1,0.1), center=bed_center, rpy=bed_rpy,
                                 color=(0.8,0.8,0.8,1), display=True, fixed=True, collision=False,
                                 uri="package://my_mesh/meshes/stl/bed_floor_centered_m_scale.stl", scale=(1,1,1))
    bed_mat = gscene.create_safe(GEOTYPE.BOX, "bed_mat", link_name="base_link", 
                                 dims=(1.60,0.91,0.13), center=(-0.025,0,0.6), rpy=(0,0,0),
                                 color=COLOR_BED_COL, fixed=True, collision=True, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "bed_head", link_name="base_link", 
                                 dims=(0.3,1.0,1.30), center=(-1.03,0,0.5), rpy=(0,0,0),
                                 color=COLOR_BED_COL, fixed=True, collision=True, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "bed_foot", link_name="base_link", 
                                 dims=(0.4,1.0,1.30), center=(1.03,0,0.5), rpy=(0,0,0),
                                 color=COLOR_BED_COL, fixed=True, collision=True, parent="bed_vis")

    if add_back_wall:
        gscene.create_safe(GEOTYPE.BOX, "bed_wall", link_name="base_link",
                           dims=(0.1,1.0,1.10), center=(-1.13,0,0.5), rpy=(0,0,0),
                           color=COLOR_BED_COL, fixed=True, collision=True, parent="bed_vis")
    return bed_mat

def move_bed(gscene, bed_center, bed_rpy):
    bed_vis = gscene.NAME_DICT["bed_vis"]
    bed_vis.set_offset_tf(center=bed_center, orientation_mat=Rot_rpy(bed_rpy))
    gscene.update_markers_all()


def add_closet(gscene, closet_center, closet_rpy, COLOR_CLOSET_COL = (0,1,0,0.3)):    
    closet_vis = gscene.create_safe(GEOTYPE.MESH, "closet_vis", link_name="base_link",
                                 dims=(0.1,0.1,0.1), center=closet_center, rpy=closet_rpy,
                                 color=(0.8,0.8,0.8,1), display=True, fixed=True, collision=False,
                                 uri="package://my_mesh/meshes/stl/top_table_centered_m_scale.stl", scale=(1,1,1))
    
    closet_leftup = gscene.create_safe(GEOTYPE.BOX, "closet_leftup", link_name="base_link",
                             dims=(1.3,0.255,0.6), center=(0.0,-0.145,1.52), rpy=(0,np.pi/2,0),
                             color=COLOR_CLOSET_COL, fixed=True, collision=True, parent="closet_vis")

    closet_rightup = gscene.create_safe(GEOTYPE.BOX, "closet_rightup", link_name="base_link", 
                             dims=(0.58,0.22,0.465), center=(-0.065,0.11,1.87), rpy=(0,np.pi/2,0),
                             color=COLOR_CLOSET_COL, fixed=True, collision=True, parent="closet_vis")
    
    closet_down = gscene.create_safe(GEOTYPE.BOX, "closet_down", link_name="base_link",
                                 dims=(0.8,0.495,0.6), center=(0,-0.025,0.45), rpy=(0,np.pi/2,0),
                                 color=COLOR_CLOSET_COL, fixed=True, collision=True, parent="closet_vis")
    closet_shelf = gscene.create_safe(GEOTYPE.BOX, "closet_shelf", link_name="base_link", 
                                 dims=(0.02,0.24,0.465), center=(-0.065,0.105,1.24), rpy=(0,np.pi/2,0),
                                 color=COLOR_CLOSET_COL, fixed=True, collision=True, parent="closet_vis")
    closet_back = gscene.create_safe(GEOTYPE.BOX, "closet_back", link_name="base_link", 
                                 dims=(0.02,0.24,0.73), center=(-0.29,0.105,1.22), rpy=(0,0,0),
                                 color=COLOR_CLOSET_COL, fixed=True, collision=True, parent="closet_vis")
    return closet_leftup, closet_rightup, closet_down

def move_closet(gscene, closet_center, closet_rpy):
    closet_vis = gscene.NAME_DICT["closet_vis"]
    closet_vis.set_offset_tf(center=closet_center, orientation_mat=Rot_rpy(closet_rpy))
    gscene.update_markers_all()

##
# @param Rtw_ref reference orientation matrix for waypoints in track coordinates
def make_work_plane(pscene, track, area_depth, TOOL_DIM, Rtw_ref=None, collision_margin=0.02):
    gscene = pscene.gscene
    track_face_name = track.name
    if Rtw_ref is not None:
        rpy_wps = Rot2rpy(Rtw_ref)
    else:
        rpy_wps = (0,0,0)
    track_face = track
    TRACK_DIM = ((track.dims[0] + area_depth)/2,) + tuple(track.dims[1:])
    TRACK_WIDTH = TOOL_DIM[0] + collision_margin
    TRACK_NUM = np.ceil(np.divide(TRACK_DIM[0] - TOOL_DIM[0], TOOL_DIM[0])).astype(np.int) + 1
    OVERMARGIN = (TRACK_NUM * TOOL_DIM[0] - TRACK_DIM[0])
    OVERMARGIN_1 = OVERMARGIN / (TRACK_NUM - 1)
    TRACK_STEP = TOOL_DIM[0] - OVERMARGIN_1
    WP_REF_B = -np.subtract(TRACK_DIM[:2], TOOL_DIM[:2]) / 2
    WP_REF_A = np.array([WP_REF_B[0], -WP_REF_B[1]])
    TRC_THIC = TRACK_DIM[2]

    track_list = []
    for i_trc in range(TRACK_NUM):
        wp1 = gscene.create_safe(GEOTYPE.BOX, "wp{}a".format(i_trc + 1), "base_link",
                                 (TOOL_DIM[0] / 2, TOOL_DIM[1] / 2, TRC_THIC),
                                 tuple(WP_REF_A + [TRACK_STEP * i_trc, 0]) + (0,), rpy=rpy_wps,
                                 color=(0.8, 0.2, 0.2, 0.2), display=True, fixed=True, collision=False,
                                 parent=track_face_name)
        wp2 = gscene.create_safe(GEOTYPE.BOX, "wp{}b".format(i_trc + 1), "base_link",
                                 (TOOL_DIM[0] / 2, TOOL_DIM[1] / 2, TRC_THIC),
                                 tuple(WP_REF_B + [TRACK_STEP * i_trc, 0]) + (0,), rpy=rpy_wps,
                                 color=(0.8, 0.2, 0.2, 0.2), display=True, fixed=True, collision=False,
                                 parent=track_face_name)
        face = gscene.create_safe(GEOTYPE.BOX, "face{}".format(i_trc + 1), "base_link",
                                  (TRACK_WIDTH, TRACK_DIM[1], TRC_THIC),
                                  center=(WP_REF_A[0] + TRACK_STEP * i_trc, 0, 0), rpy=(0, 0, 0),
                                  color=(0.8, 0.2, 0.2, 0.2), display=True, fixed=True, collision=False,
                                  parent=track_face_name)
        track_list.append((wp1, wp2, face))

    gscene.update_markers_all()

    for sname in pscene.subject_name_list:
        pscene.remove_subject(sname)

    sweep_list = []
    for i_t, track_tem in enumerate(track_list):
        wp1, wp2, face = track_tem
        sweep_ = pscene.create_subject(oname="sweep{}".format(i_t + 1), gname=track_face_name, _type=SweepLineTask,
                                       action_points_dict={
                                           wp1.name: SweepFrame(wp1.name, wp1, [0, 0, 0.005], [0, 0, 0]),
                                           wp2.name: SweepFrame(wp2.name, wp2, [0, 0, 0.005], [0, 0, 0])},
                                       clearance=[face])
        sweep_list.append(sweep_)
    return sweep_list, track_list

def add_track(table, TABLE_HEIGHT, area_depth, area_width, corner_center):
    gscene = table.gscene
    track = gscene.create_safe(GEOTYPE.BOX, "track", "base_link", (abs(area_depth),(area_width),0.01),
                               tuple(corner_center)+(TABLE_HEIGHT-0.005,), rpy=(0,0,0),
                               color=(0.0,0.8,0.8,0.2), display=True, fixed=True, collision=True)
    track_face = gscene.copy_from(track, new_name="track_face", collision=False, color=(0.8,0.8,0.8,0.0))
    TRACK_DIM = np.copy(track_face.dims)
    track_face.set_dims((3, 3, track.dims[2]))
    gscene.update_markers_all()
    return track, track_face

def calc_base_target(table, track, T_ft, T_rot_table):
    T_bf = track.Toff
    T_bt = np.matmul(T_bf, T_ft)
    T_tb = SE3_inv(T_bt)
    ## 6 DoF pose to go
    T_bb2 = np.matmul(table.Toff, np.matmul(T_rot_table, T_tb))
    return T_bt, T_bb2

def base_offet_to_mobile(T_bb2, CONNECT_MOBILE, OFFSET_MOBILE=[0.172, 0, 0.439]):
    T_mb = SE3(Rot_axis(3, 0), OFFSET_MOBILE)
    T_bm = SE3_inv(T_mb)
    T_mm2 = np.matmul(SE3_inv(T_bm), np.matmul(T_bb2, SE3_inv(T_mb)))
    T_bm2 = np.matmul(T_bm, T_mm2)
    return T_mm2, T_bm2

def get_relative_mobile_command(T_mm2, CONNECT_MOBILE, T_approach = SE3(np.identity(3), [-0.5, 0, 0])):
    xy_rel = T_mm2[:2, 3]
    quat_rel = Rotation.from_dcm(T_mm2[:3, :3]).as_quat()
    quat_zw_rel = quat_rel[-2:]
    xyzw_rel = np.concatenate([xy_rel, quat_zw_rel])

    if CONNECT_MOBILE:
        cur_xyzw = get_xyzw_cur()
    else:
        cur_xyzw = (0, 0, 0, 1)

    T_am_cur = T_xyzquat((cur_xyzw[:2] + (0,), (0, 0) + cur_xyzw[2:]))
    T_am_to = np.matmul(T_am_cur, T_mm2)
    T_am_to_ready = np.matmul(T_am_to, T_approach)

    tar_xy = T_am_to[:2, 3].tolist()
    tar_qzw = Rotation.from_dcm(T_am_to[:3, :3]).as_quat()[-2:].tolist()

    tar_xy_rd = T_am_to_ready[:2, 3].tolist()
    tar_qzw_rd = Rotation.from_dcm(T_am_to_ready[:3, :3]).as_quat()[-2:].tolist()

    tar_xyzw = tar_xy + tar_qzw
    tar_xyzw_rd = tar_xy_rd + tar_qzw_rd
    # raise(NotImplementedError("tar_xyzw: relative? absolute?"))
    print("curre  xyzw: {}".format(np.round(cur_xyzw, 2)))
    print("ready  xyzw: {}".format(np.round(tar_xyzw_rd, 2)))
    print("target xyzw: {}".format(np.round(tar_xyzw, 2)))
    return cur_xyzw, tar_xyzw_rd, tar_xyzw

def move_mobile_robot(sock_mobile, cur_xyzw, tar_xyzw_rd, tar_xyzw, MOBILE_IP, CONNECT_MOBILE, move_direct=False):
    delta_xyzw = np.subtract(tar_xyzw, cur_xyzw)
    if move_direct:
        if np.linalg.norm(np.matmul(Rotation.from_quat([0,0] + list(cur_xyzw[-2:])).as_dcm().transpose(),
                                    Rotation.from_quat([0,0] + list(tar_xyzw[-2:])).as_dcm())-np.identity(3))<1e-3:
            delta_xyzw[-2:] = 0
        else:
            move_direct = False

    if move_direct and np.linalg.norm(delta_xyzw) < 2.0:
        move_steps = int(ceil(np.linalg.norm(delta_xyzw) / 0.7))
        xyzw_step = delta_xyzw / move_steps
        for i_stp in range(move_steps):
            if i_stp == move_steps - 1:
                tar_cur = tar_xyzw
            else:
                tar_cur = np.add(cur_xyzw, xyzw_step * (i_stp + 1))
            print("move to: {} ({})".format(np.round(tar_cur, 2), time.time()))
            if CONNECT_MOBILE:
                fix_delay = np.linalg.norm(np.subtract(cur_xyzw, tar_xyzw)) < 1e-1
                cur_xyzw = send_pose_wait(sock_mobile,
                                          tar_cur,
                                          send_ip=MOBILE_IP, recv_delay=1, fix_delay=fix_delay)
    else:
        print("approach to: {} ({})".format(np.round(tar_xyzw_rd, 2), time.time()))
        if CONNECT_MOBILE:
            fix_delay = np.linalg.norm(np.subtract(cur_xyzw, tar_xyzw)) < 1e-1
            cur_xyzw = send_pose_wait(sock_mobile, tar_xyzw_rd, send_ip=MOBILE_IP,
                                      recv_delay=1, fix_delay=fix_delay)
        print("move to: {} ({})".format(np.round(tar_xyzw, 2), time.time()))
        for _ in range(3):
            fix_delay = np.linalg.norm(np.subtract(cur_xyzw, tar_xyzw)) < 1e-1
            if CONNECT_MOBILE:
                cur_xyzw = send_pose_wait(sock_mobile, tar_xyzw, send_ip=MOBILE_IP,
                                          recv_delay=0.5, fix_delay=fix_delay)
        print("stop at: {} ({})".format(np.round(cur_xyzw, 2), time.time()))
    return cur_xyzw
# # Go view loc
# cur_xyzw = send_pose_wait(sock_mobile, [2.77, 1.,   0.86, 0.51], send_ip=MOBILE_IP)


def calc_gaze_pose(cn_cur, mplan, table_front, viewpoint, indy, CONNECT_INDY, GAZE_DIST=0.5,
                   VIEW_POSE=np.deg2rad([  0., -28.,  85.,  -0.,  57., -180])):
    if CONNECT_INDY:
        with indy:
            Qcur = np.deg2rad(indy.get_joint_pos())
    else:
        Qcur = VIEW_POSE

    gscene = table_front.gscene
    T_bt_front = table_front.Toff

    off_corner_gaze = np.divide(table_front.dims, 2) * corner_point_dirs[cn_cur]
    T_tc = SE3(np.identity(3), off_corner_gaze)
    T_bc = np.matmul(T_bt_front, T_tc)

    if cn_cur == Corners.Left:
        T_cam_c_list = [SE3(Rot_rpy((0, -np.pi * 3 / 4, np.pi / 2)), (0, 0, GAZE_DIST))]
    elif cn_cur == Corners.Right:
        T_cam_c_list = [SE3(Rot_rpy((np.pi / 8, -np.pi * 7 / 8, np.pi / 2)), (0, 0, GAZE_DIST))]
    else:
        raise(NotImplementedError("Unexpected Corner"))
    for T_cam_c in T_cam_c_list:
        T_bcam = np.matmul(T_bc, SE3_inv(T_cam_c))
        T_ecam = viewpoint.Toff
        T_be = np.matmul(T_bcam, SE3_inv(T_ecam))
        gscene.add_highlight_axis("cam", "effector", "base_link", T_be[:3, 3], T_be[:3, :3])
        gaze_traj, success = mplan.planner.plan_py("indy0", "indy0_tcp", np.concatenate(T2xyzquat(T_be)), "base_link",
                                                   Qcur)
        if success:
            gaze_pose = gaze_traj[-1]
            break

    print("GAZE: {}".format(success))
    return gaze_traj, success


def move_mobile_update_state(sock_mobile, MOBILE_IP, wayframer, from_Q, to_Q, D_APPROACH=0.3, CONNECT_MOBILE=True):
    gscene = wayframer.geometry.gscene
    Tbw0 = wayframer.get_tf_handle(list2dict(from_Q,
                                             gscene.joint_names))
    Tbw1 = wayframer.get_tf_handle(list2dict(to_Q,
                                             gscene.joint_names))
    print("move to VIEW_MOVED_EXT: {}".format(np.round(to_Q, 3)))
    Tw0w1 = np.matmul(SE3_inv(Tbw0), Tbw1)
    cur_xyzw, tar_xyzw_rd, tar_xyzw = get_relative_mobile_command(
        Tw0w1, CONNECT_MOBILE, T_approach = SE3(np.identity(3), [-D_APPROACH, 0, 0]))
    T_aw0 = T_xyzquat((cur_xyzw[:2] + (0,), (0, 0) + cur_xyzw[2:]))
    cur_xyzw = move_mobile_robot(sock_mobile,
                                 cur_xyzw, tar_xyzw_rd, tar_xyzw,
                                 MOBILE_IP, CONNECT_MOBILE, move_direct=True)
    T_aw1 = T_xyzquat((cur_xyzw[:2] + (0,), (0, 0) + cur_xyzw[2:]))
    T_bw1 = matmul_series(Tbw0, SE3_inv(T_aw0), T_aw1)

    new_Q = np.copy(to_Q)
    new_Q[:2] = T_bw1[:2, 3]
    new_Q[2] = Rot2axis(T_bw1[:3,:3], 3)
    print("ended in VIEW_MOVED_EXT: {}".format(np.round(new_Q, 3)))
    return new_Q
