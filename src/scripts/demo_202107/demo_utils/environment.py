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
                                   dims=(0.01, 0.01, 0.01), center=(0, 0, 0), rpy=(0, 0, -np.pi / 2),
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

def add_indy_tool_kiro(gscene, zoff=0, tool_link="indy0_tcp", face_name="brush_face"):
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="adapter",
                       link_name=tool_link,
                       center=(0, 0, 0.0025), dims=(0.09, 0.09, 0.005), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="adapter_col",
                       link_name=tool_link,
                       center=(0, 0, 0.0025), dims=(0.13, 0.13, 0.005), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)

    gscene.create_safe(gtype=GEOTYPE.BOX, name="hindge0",
                       link_name=tool_link,
                       center=(0, 0, 0.0125), dims=(0.022, 0.036, 0.025), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="hindge0_col",
                       link_name=tool_link,
                       center=(0, 0, 0.0125), dims=(0.062, 0.076, 0.025), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)

    gscene.create_safe(gtype=GEOTYPE.BOX, name="hinge_bar",
                       link_name=tool_link,
                       center=(0.053, 0, 0.068), dims=(0.011, 0.020, 0.15), rpy=(0, 1 * np.pi / 4, 0),
                       color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="hinge_bar_col",
                       link_name=tool_link,
                       center=(0.053, 0, 0.068), dims=(0.051, 0.060, 0.15), rpy=(0, 1 * np.pi / 4, 0),
                       color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)

    gscene.create_safe(gtype=GEOTYPE.BOX, name="hindge1",
                       link_name=tool_link,
                       center=(0.1685+zoff, 0, 0.121), dims=(0.025, 0.036, 0.022), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="hindge1_col",
                       link_name=tool_link,
                       center=(0.1685+zoff, 0, 0.121), dims=(0.025, 0.076, 0.062), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="brushbase",
                       link_name=tool_link,
                       center=(0.1885+zoff, 0, 0.121), dims=(0.08, 0.08, 0.015), rpy=(0, np.pi / 2, 0),
                       color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="brushbase_col",
                       link_name=tool_link,
                       center=(0.1885+zoff, 0, 0.121), dims=(0.12, 0.12, 0.015), rpy=(0, np.pi / 2, 0),
                       color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name=tool_link,
                       center=(0.207+zoff, 0, 0.121), dims=(0.037, 0.10, 0.34), rpy=(np.pi, 0, np.pi),
                       color=(1.0, 1.0, 0.94, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name=tool_link,
                       center=(0.187+zoff, 0, 0.121), dims=(0.057, 0.10, 0.36), rpy=(np.pi, 0, np.pi),
                       color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)
    return brush_face

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
        delta_xyzw[-2:] = 0
    if move_direct and np.linalg.norm(delta_xyzw) < 2.0:
        move_steps = int(ceil(np.linalg.norm(delta_xyzw) / 0.7))
        xyzw_step = delta_xyzw / move_steps
        for i_stp in range(move_steps):
            if i_stp == move_steps - 1:
                tar_cur = tar_xyzw
            else:
                tar_cur = np.add(cur_xyzw, xyzw_step * (i_stp + 1))
            print("move to: {}".format(np.round(tar_cur, 2)))
            if CONNECT_MOBILE:
                cur_xyzw = send_pose_wait(sock_mobile,
                                          tar_cur,
                                          send_ip=MOBILE_IP)
    else:
        print("move to: {}".format(np.round(tar_xyzw_rd, 2)))
        if CONNECT_MOBILE:
            cur_xyzw = send_pose_wait(sock_mobile, tar_xyzw_rd, send_ip=MOBILE_IP)
        print("move to: {}".format(np.round(tar_xyzw, 2)))
        if CONNECT_MOBILE:
            cur_xyzw = send_pose_wait(sock_mobile, tar_xyzw, send_ip=MOBILE_IP)
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