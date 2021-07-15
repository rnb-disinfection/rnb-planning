import numpy as np
from pkg.geometry.geotype import GEOTYPE
from pkg.planning.constraint.constraint_actor import Gripper2Tool, PlacePlane, SweepFramer, FixtureSlot
from pkg.planning.constraint.constraint_common import MotionConstraint
from pkg.planning.constraint.constraint_subject import AbstractTask, AbstractObject
from pkg.planning.constraint.constraint_subject import SweepLineTask
from pkg.planning.constraint.constraint_subject import SweepFrame
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *

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

def make_work_plane(pscene, TRACK_DIM, TOOL_DIM, EE_DEPTH_OFF, depths, width_range, MARGIN=0):
    track_face_binder = pscene.create_binder(bname="track_face", gname="track_face", _type=PlacePlane, point=None)
    track_face = track_face_binder.geometry
    TOOL_WIDTH = TOOL_DIM[1]
    TRACK_WIDTH = TOOL_DIM[0] + 0.02
    TRC_THIC = TRACK_DIM[2]
    track_list = []
    sweep_width = -np.subtract(*width_range)-MARGIN
    gscene = pscene.gscene

    for i_trc, depth_cur in enumerate(depths):
        line_center = np.matmul(
            SE3_inv(track_face.get_tf(pscene.combined_robot.home_dict)),
            (depth_cur - EE_DEPTH_OFF, np.mean(width_range), track_face.center[2], 1))[:3]
        wp1 = gscene.create_safe(GEOTYPE.BOX, "wp{}a".format(i_trc + 1), "base_link",
                                 (TOOL_DIM[0] / 2, TOOL_DIM[1] / 2, TRC_THIC),
                                 np.add(line_center, (0, sweep_width / 2 - TOOL_WIDTH / 2, 0)), rpy=(0, 0, 0),
                                 color=(0.8, 0.2, 0.2, 0.2), display=True, fixed=True, collision=False, parent="track_face")
        wp2 = gscene.create_safe(GEOTYPE.BOX, "wp{}b".format(i_trc + 1), "base_link",
                                 (TOOL_DIM[0] / 2, TOOL_DIM[1] / 2, TRC_THIC),
                                 np.subtract(line_center, (0, sweep_width / 2 - TOOL_WIDTH / 2, 0)), rpy=(0, 0, 0),
                                 color=(0.8, 0.2, 0.2, 0.2), display=True, fixed=True, collision=False, parent="track_face")

        face = gscene.create_safe(GEOTYPE.BOX, "face{}".format(i_trc + 1), "base_link",
                                  (TRACK_WIDTH, TRACK_DIM[1], TRC_THIC),
                                  center=line_center, rpy=(0, 0, 0),
                                  color=(0.8, 0.2, 0.2, 0.2), display=True, fixed=True, collision=False,
                                  parent="track_face")
        track_list.append((wp1, wp2, face))
    gscene.update_markers_all()

    for sname in pscene.subject_name_list:
        pscene.remove_subject(sname)

    sweep_list = []
    for i_t, track_tem in enumerate(track_list):
        wp1, wp2, face = track_tem
        sweep_ = pscene.create_subject(oname="sweep{}".format(i_t + 1), gname="track_face", _type=SweepLineTask,
                                       action_points_dict={wp1.name: SweepFrame(wp1.name, wp1, [0, 0, 0.005], [0, 0, 0]),
                                                           wp2.name: SweepFrame(wp2.name, wp2, [0, 0, 0.005], [0, 0, 0])},
                                       clearance=[face])
        sweep_list.append(sweep_)
    return sweep_list, track_list