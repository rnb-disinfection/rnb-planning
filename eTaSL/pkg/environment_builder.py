from .stereo import *
from .geometry import *
from . constants import *
from threading import Thread

def detect_environment(aruco_map, dictionary, robot_tuples, env_dict, camT_dict={"cam0":np.identity(4)}, ref_name='floor'):
    xyz_rpy_robots = {}
    xyz_rvec_cams = {}
    env_gen_dict = {}
    while True:
        try:
            objectPose_dict, corner_dict, color_image, rs_image, rs_corner_dict = get_object_pose_dict_stereo(aruco_map,
                                                                                                              dictionary)

            for rtuple in robot_tuples:
                rname = rtuple[0]
                Tbr = get_T_rel(ref_name, rname, objectPose_dict)
                xyz_rpy_robots[rname] = T2xyzrpy(Tbr)

            T_bc = SE3_inv(objectPose_dict[ref_name])
            for cname, camT in camT_dict.items():
                xyz_rvec_cams[cname] = T2xyzrvec(np.matmul(T_bc, camT))

            for ename, ginfo in env_dict.items():
                env_gen_dict[ename] = (ginfo, T2xyzrvec(get_T_rel(ref_name, ename, objectPose_dict)))
            break
        except KeyError as e:
            print(e)
            break
        except Exception as e:
            print(e)
            pass
    return xyz_rpy_robots, xyz_rvec_cams, env_gen_dict, objectPose_dict, corner_dict, color_image

def add_objects_gen(graph, obj_gen_dict, color=(0.6,0.6,0.6,1), collision=True, link_name="world"):
    graph.add_geometry_items([ogen[0](*ogen[1], name=oname, link_name=link_name, urdf_content=graph.urdf_content,
             color=color, collision=collision) for oname, ogen in obj_gen_dict.items()], fixed=True)

def add_cam_poles(graph, xyz_rvec_cams, color=(0.6,0.6,0.6,0.3), link_name="world"):
    graph.add_geometry_items([GeoSegment(np.subtract(xyzrvec[0], [0,0,xyzrvec[0][2]/2-0.05]),
                                         (0,0,0), xyzrvec[0][2]+0.1, 0.075,
                                         name="pole_{}".format(cname),
                                         link_name=link_name, urdf_content=graph.urdf_content,
                                         color=color, collision=True)
                              for cname, xyzrvec in xyz_rvec_cams.items()], fixed=True)

def detect_objects(movable_generators, aruco_map, dictionary, stereo=True, kn_config=None):
    aruco_map_mv = {k: v for k, v in aruco_map.items() if k in movable_generators}
    if stereo:
        objectPose_dict_mv, corner_dict_mv, color_image, rs_image, rs_corner_dict = get_object_pose_dict_stereo(
            aruco_map_mv, dictionary)
    else:
        color_image = get_kn_image()
        objectPose_dict_mv, corner_dict_mv = get_object_pose_dict(color_image, aruco_map, dictionary, *kn_config)
    return objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv

def calc_put_point(objectPose_dict_mv, object_generators, object_dict, ref_tuple):
    T_mv_dict = {mname: get_T_rel(ref_tuple[0], mname, objectPose_dict_mv) for mname in object_generators if
                 mname in objectPose_dict_mv}
    xyz_rvec_mv_dict = {mname: T2xyzrvec(Tv) for mname, Tv in T_mv_dict.items()}

    put_point_dict = {}
    up_point_dict = {}
    Rx180 = Rot_axis(1, np.pi)
    for mtem, xyz_rvec in xyz_rvec_mv_dict.items():
        if mtem in object_dict:
            Robj = Rotation.from_rotvec(xyz_rvec[1]).as_dcm()
            put_point_dict[mtem] = get_put_dir(Robj=Robj,
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
            up_point_dict[mtem] = get_put_dir(Robj=np.matmul(Rx180, Robj),
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
    return xyz_rvec_mv_dict, put_point_dict, up_point_dict


class DynamicDetector:
    def __init__(self, dynamic_objects, aruco_map, dictionary, rs_config, T_c12, ref_T):
        self.dynamic_objects, self.aruco_map, self.dictionary, self.rs_config, self.T_c12, self.ref_T = \
            dynamic_objects, aruco_map, dictionary, rs_config, T_c12, ref_T

    def detector_thread_fun(self):
        self.detector_stop = False
        aruco_map_dynamic = {k: v for k, v in self.aruco_map.items() if k in self.dynamic_objects}
        self.dynPos_dict = {}
        while not self.detector_stop:
            color_image = get_rs_image()
            objectPose_dict_mv, corner_dict_mv = get_object_pose_dict(color_image, aruco_map_dynamic, self.dictionary,
                                                                      *self.rs_config)
            for k, v in objectPose_dict_mv.items():
                self.dynPos_dict[k] = np.matmul(SE3_inv(self.ref_T), np.matmul(self.T_c12, v))

    def stop_detector(self):
        self.detector_stop = True

    def get_dynPos_dict(self):
        return self.dynPos_dict

    def __enter__(self):
        self.t_det = Thread(target=self.detector_thread_fun)
        self.t_det.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_detector()

class RvizPublisher:
    def __init__(self, graph, obs_names):
        self.graph, self.obs_names = graph, obs_names
        self.obsPos_dict = None
        self.POS_CUR = None

    def rviz_thread_fun(self):
        self.rviz_stop = False
        while not self.rviz_stop:
            if self.obsPos_dict is None or self.POS_CUR is None:
                time.sleep(0.1)
                continue
            for oname in self.obs_names:
                if oname in self.obsPos_dict:
                    T_bo = self.obsPos_dict[oname]
                    GeometryItem.GLOBAL_GEO_DICT[oname].set_offset_tf(T_bo[:3, 3], T_bo[:3, :3])
            self.graph.show_pose(self.POS_CUR)

    def stop_rviz(self):
        self.rviz_stop = True

    def update(self, obsPos_dict, POS_CUR):
        self.obsPos_dict = obsPos_dict
        self.POS_CUR = POS_CUR

    def __enter__(self):
        self.t_rviz = Thread(target=self.rviz_thread_fun)
        self.t_rviz.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_rviz()

def update_geometries(onames, objectPose_dict_mv):
    for gname in onames:
        gtem = [gtem for gtem in GeometryItem.GLOBAL_GEO_LIST if gtem.name == gname]
        if len(gtem)>0 and gname in objectPose_dict_mv:
            gtem = gtem[0]
            Tg = get_T_rel("floor", gname, objectPose_dict_mv)
            gtem.set_offset_tf(Tg[:3,3], Tg[:3,:3])

from joint_utils import *

def match_point_binder(graph, initial_state, objectPose_dict_mv):
    graph.set_object_state(initial_state)
    Q0 = initial_state.Q
    Q0dict = joint_list2dict(Q0, graph.joint_names)
    binder_T_dict = {}
    binder_dir_dict = {}
    binder_scale_dict = {}
    for k,binder in graph.binder_dict.items():
        binder_T = binder.object.get_tf(Q0dict)
        binder_scale_dict[k] = binder.object.get_scale()
        if binder.point is not None:
            binder_T = np.matmul(binder_T, SE3(np.identity(3), binder.point))
            binder_scale_dict[k] = 0
        binder_T_dict[k] = binder_T
        binder_dir_dict[k] = binder.direction

    kpt_pair_dict = {}
    for kobj, Tobj in objectPose_dict_mv.items():

        min_val = 1e10
        min_point = ""
        if kobj not in graph.object_dict:
            continue
        for kpt, bd in graph.object_dict[kobj].action_points_dict.items():
            Tpt = bd.object.get_tf(Q0dict)
            point_dir = bd.point_dir if hasattr(bd, "point_dir") else bd.point_ori
            point_cur = np.matmul(Tpt, list(point_dir[0])+[1])[:3]
            direction_cur = np.matmul(Tpt[:3,:3], point_dir[1])

            for kbd, Tbd in binder_T_dict.items():
                if kobj == kbd:
                    continue
                point_diff = np.matmul(SE3_inv(Tbd), SE3(np.identity(3), point_cur))[:3,3]
                point_diff_norm = np.linalg.norm(np.maximum(np.abs(point_diff) - binder_scale_dict[kbd],0))
                dif_diff_norm = np.linalg.norm(direction_cur - binder_dir_dict[kbd])
                bd_val_norm = point_diff_norm# + dif_diff_norm
                if bd_val_norm < min_val:
                    min_val = bd_val_norm
                    min_point = kbd
        kpt_pair_dict[kobj] = min_point
#         print("{} - {} ({})".format(kobj, min_point, min_val))
    return kpt_pair_dict