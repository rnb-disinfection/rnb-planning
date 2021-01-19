from .detector.aruco.stereo import *
from . constants import *
from threading import Thread
from .robots_custom import *
import rospy
from .utils.utils import *
from .marker_config import *

__rospy_initialized = False
__roscore = None


def get_T_rel(coord_from, coord_to, objectPose_dict):
    return np.matmul(SE3_inv(objectPose_dict[coord_from]), objectPose_dict[coord_to])


def get_put_dir(Robj, dir_vec_dict, ref_vec=[[0], [0], [-1]]):
    downvec = np.matmul(np.transpose(Robj), ref_vec)
    dir_match_dict = {k: np.dot(v, downvec) for k, v in dir_vec_dict.items()}
    key_match = sorted(dir_match_dict.keys(), key=lambda k: dir_match_dict[k])[-1]
    return key_match


def set_custom_robots(ROBOTS_ON_SCENE, xyz_rpy_robots, custom_limits, node_name='task_planner', start_rviz=True, custom_xacro=None):
    global __rospy_initialized, __roscore
    if not __rospy_initialized:
        __roscore = subprocess.Popen(['roscore'])
        rospy.init_node(node_name, anonymous=True)
        __rospy_initialized = True

    urdf_content = None
    xcustom = XacroCustomizer.instance()
    if custom_xacro is not None:
        xcustom.initialize(ROBOTS_ON_SCENE, xyz_rpy_robots, xacro_path=custom_xacro)
    else:
        xcustom.initialize(ROBOTS_ON_SCENE, xyz_rpy_robots)

    JOINT_NAMES, LINK_NAMES, urdf_content = \
        xcustom.convert_xacro_to_urdf(
            joint_fix_dict={'finger': 'upper'},
            joint_offset_dict={},
            joint_limit_dict=custom_limits)

    if start_rviz:
        xcustom.start_rviz()

    return xcustom, JOINT_NAMES, LINK_NAMES, urdf_content

def detect_robots(aruco_map, dictionary, robot_tuples, kn_config, rs_config, T_c12, ref_name='floor'):
    xyz_rpy_robots = {}
    while True:
        try:
            objectPose_dict, corner_dict, color_image, rs_image, rs_objectPose_dict, rs_corner_dict = \
                get_object_pose_dict_stereo(aruco_map, dictionary, kn_config=kn_config, rs_config=rs_config, T_c12=T_c12)

            for rtuple in robot_tuples:
                rname = rtuple[0]
                Tbr = get_T_rel(ref_name, rname, objectPose_dict)
                xyz_rpy_robots[rname] = T2xyzrpy(Tbr)

            break
        except KeyError as e:
            print(e)
            break
        except Exception as e:
            print(e)
            pass
    T0 = np.identity(4)
    return xyz_rpy_robots, \
           objectPose_dict, corner_dict, color_image, \
           {k:v for k,v in rs_objectPose_dict.items() if np.sum(np.abs(T0-v))>1e-5}, rs_corner_dict, rs_image

def detect_environment(ghnd, aruco_map, dictionary, kn_config, rs_config, T_c12, ref_name='floor'):
    env_dict = {k: CallHolder(ghnd.create_safe,
                              ["center", "rpy"], **v.get_geometry_kwargs()) for k, v in aruco_map.items() if
                v.dlevel == DetectionLevel.ENVIRONMENT}
    env_gen_dict = {}
    while True:
        try:
            objectPose_dict, corner_dict, color_image, rs_image, rs_objectPose_dict, rs_corner_dict = \
                get_object_pose_dict_stereo(aruco_map, dictionary, kn_config=kn_config, rs_config=rs_config, T_c12=T_c12)

            for ename, ginfo in env_dict.items():
                env_gen_dict[ename] = (ginfo, T2xyzrpy(get_T_rel(ref_name, ename, objectPose_dict)))
            break
        except KeyError as e:
            print(e)
            break
        except Exception as e:
            print(e)
            pass
    T0 = np.identity(4)
    return env_gen_dict, \
           objectPose_dict, corner_dict, color_image, \
           {k:v for k,v in rs_objectPose_dict.items() if np.sum(np.abs(T0-v))>1e-5}, rs_corner_dict, rs_image

def add_objects_gen(graph, obj_gen_dict, color=(0.6,0.6,0.6,1), collision=True, link_name="base_link"):
    gtems = []
    for oname, ogen in obj_gen_dict.items():
        gtems.append(ogen[0](*ogen[1], name=oname, link_name=link_name, color=color, collision=collision, fixed=True))
    return gtems

def add_cam_poles(graph, xyz_rpy_cams, color=(0.6,0.6,0.6,0.3), link_name="base_link"):
    gtems = []
    ghnd = graph.ghnd
    for cname, xyzrpy in xyz_rpy_cams.items():
        gtems.append(ghnd.create_safe(name="pole_{}".format(cname), link_name=link_name, gtype=GEOTYPE.CAPSULE,
                                  center= np.subtract(xyzrpy[0], [0,0,xyzrpy[0][2]/2-0.05]),
                                  dims=(0.15, 0.15, xyzrpy[0][2]+0.1), rpy=(0,0,0),
                                  color=color, collision=True, fixed=True)
                     )
    return gtems

def detect_objects(aruco_map, dictionary, stereo=True, kn_config=None):
    aruco_map_mv = {k: v for k, v in aruco_map.items() if v.dlevel in [DetectionLevel.MOVABLE, DetectionLevel.ONLINE]}
    if stereo:
        objectPose_dict_mv, corner_dict_mv, color_image, rs_image, rs_objectPose_dict, rs_corner_dict = \
            get_object_pose_dict_stereo(aruco_map_mv, dictionary)
    else:
        color_image = get_kn_image()
        objectPose_dict_mv, corner_dict_mv = aruco_map.get_object_pose_dict(color_image, *kn_config)
    return objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv

def calc_put_point(ghnd, objectPose_dict_mv, aruco_map, object_dict, ref_tuple):
    object_generators = {k: CallHolder(ghnd.create_safe,
                                       ["center", "rpy"], **v.get_geometry_kwargs()) for k, v in
                         aruco_map.items() if v.dlevel in [DetectionLevel.MOVABLE, DetectionLevel.ONLINE] and k in objectPose_dict_mv}
    T_mv_dict = {mname: np.matmul(SE3_inv(ref_tuple[1]), objectPose_dict_mv[mname]) for mname in object_generators if
                 mname in objectPose_dict_mv}
    xyz_rpy_mv_dict = {mname: T2xyzrpy(Tv) for mname, Tv in T_mv_dict.items()}

    put_point_dict = {}
    up_point_dict = {}
    Rx180 = Rot_axis(1, np.pi)
    for mtem, xyz_rpy in xyz_rpy_mv_dict.items():
        if mtem in object_dict:
            Robj = Rot_rpy(xyz_rpy[1])
            put_point_dict[mtem] = get_put_dir(Robj=Robj,
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
            up_point_dict[mtem] = get_put_dir(Robj=np.matmul(Rx180, Robj),
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
    return xyz_rpy_mv_dict, put_point_dict, up_point_dict


STEREO_CONFIG_DEFAULT = ((np.array([[1.82983423e+03, 0.00000000e+00, 1.91572046e+03],
                                    [0.00000000e+00, 1.82983423e+03, 1.09876086e+03],
                                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
                          np.array([7.09966481e-01, -2.73409390e+00, 1.45804870e-03, -3.24774766e-04,
                                    1.44911301e+00, 5.84310412e-01, -2.56374550e+00, 1.38472950e+00])),
                         (np.array([[1.39560388e+03, 0.00000000e+00, 9.62751587e+02],
                                    [0.00000000e+00, 1.39531934e+03, 5.47687012e+02],
                                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
                          np.array([0., 0., 0., 0., 0.])),
                         np.array([[0.8316497, -0.03031947, -0.55447227, 0.6465144],
                                   [0.17082036, 0.964059, 0.20349611, -0.13999878],
                                   [0.5283741, -0.26395264, 0.8069386, 0.06811327],
                                   [0., 0., 0., 1.]],
                                  dtype=np.float32)
                         )

CAM_XYZ_RPY_DEFAULT = {'cam0': ([0.04992591589689255, -0.5109567046165466, 0.5428988933563232],
                                [-2.2611520414184096, 0.03879040388027996, -0.02012913871801042]),
                       'cam1': ([0.7000014781951904, -0.39214593172073364, 0.6081380844116211],
                                [-2.4248302293421484, -0.1538402791253786, 0.7300117151331913])}
REF_POSE_DEFAULT = np.array([[0.9993871, -0.0349786, 0.00139052, -0.06847536],
                             [-0.02122436, -0.6370408, -0.77053785, 0.09429896],
                             [0.02783815, 0.77003604, -0.63739276, 0.7379357],
                             [0., 0., 0., 1.]],
                            dtype=np.float32)


class StereoCamera(Singleton):
    CAM0_NAME = "cam0"
    CAM1_NAME = "cam1"
    REF_NAME = 'floor'

    def __init__(self):
        init_stereo()
        self.aruco_map, self.dictionary = get_aruco_map()
        self.kn_config, self.rs_config, self.T_c12 = STEREO_CONFIG_DEFAULT
        self.xyz_rpy_cams = CAM_XYZ_RPY_DEFAULT
        self.ref_tuple = (self.REF_NAME, REF_POSE_DEFAULT)

    def set_aruco_map(self, aruco_map):
        self.aruco_map = aruco_map

    def calibrate(self, ghnd):
        self.kn_config, self.rs_config, self.T_c12 = calibrate_stereo(self.aruco_map, self.dictionary)
        self.env_gen_dict, objectPose_dict, corner_dict, color_image, rs_objectPose_dict, rs_corner_dict, rs_image = \
            detect_environment(ghnd, {k: v for k, v in self.aruco_map.items() if k == self.REF_NAME}, self.dictionary,
                               kn_config=self.kn_config, rs_config=self.rs_config, T_c12=self.T_c12,
                               ref_name=self.REF_NAME)
        self.ref_tuple = (self.REF_NAME, objectPose_dict[self.REF_NAME])
        self.update_cam_coords()

    def update_cam_coords(self):
        T_bc = SE3_inv(self.ref_tuple[1])
        self.xyz_rpy_cams = {}
        for cname, camT in self.get_camT_dict().items():
            self.xyz_rpy_cams[cname] = T2xyzrpy(np.matmul(T_bc, camT))

    def get_camT_dict(self):
        return {self.CAM0_NAME: np.identity(4), self.CAM1_NAME: self.T_c12}

    def detect_robots(self, robots_on_scene):
        self.xyz_rpy_robots, objectPose_dict, corner_dict, color_image, rs_objectPose_dict, rs_corner_dict, rs_image = \
            detect_robots(self.aruco_map, self.dictionary, robots_on_scene,
                          kn_config=self.kn_config, rs_config=self.rs_config, T_c12=self.T_c12, ref_name=self.REF_NAME)
        self.robots_on_scene = robots_on_scene
        return self.xyz_rpy_robots

    def detect_environment(self, ghnd):
        self.env_gen_dict, objectPose_dict, corner_dict, color_image, rs_objectPose_dict, rs_corner_dict, rs_image = \
            detect_environment(ghnd, self.aruco_map, self.dictionary,
                               kn_config=self.kn_config, rs_config=self.rs_config, T_c12=self.T_c12,
                               ref_name=self.REF_NAME)
        return self.env_gen_dict, objectPose_dict, corner_dict, color_image, rs_objectPose_dict, rs_corner_dict, rs_image

    def __del__(self):
        disconnect_stereo()


class DynamicDetector:
    def __init__(self, dynamic_objects, aruco_map, dictionary, rs_config, T_c12, ref_T):
        self.dynamic_objects, self.aruco_map, self.dictionary, self.rs_config, self.T_c12, self.ref_T = \
            dynamic_objects, aruco_map, dictionary, rs_config, T_c12, ref_T

    def detector_thread_fun(self):
        self.detector_stop = False
        self.dynPos_dict = {}
        while not self.detector_stop:
            color_image = get_rs_image()
            objectPose_dict_mv, corner_dict_mv = self.aruco_map.get_object_pose_dict(color_image, *self.rs_config,
                                                                      name_mask=self.dynamic_objects)
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
        self.ghnd = graph.ghnd

    def rviz_thread_fun(self):
        self.rviz_stop = False
        while not self.rviz_stop:
            if self.obsPos_dict is None or self.POS_CUR is None:
                time.sleep(0.1)
                continue
            for oname in self.obs_names:
                if oname in self.obsPos_dict:
                    T_bo = self.obsPos_dict[oname]
                    self.ghnd.NAME_DICT[oname].set_offset_tf(T_bo[:3, 3], T_bo[:3, :3])
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

def update_geometries(ghnd, onames, objectPose_dict_mv, refFrame):
    refFrameinv = SE3_inv(refFrame)
    for gname in onames:
        gtem = [gtem for gtem in ghnd if gtem.name == gname]
        if len(gtem)>0 and gname in objectPose_dict_mv:
            gtem = gtem[0]
            Tg = np.matmul(refFrameinv, objectPose_dict_mv[gname])
            gtem.set_offset_tf(Tg[:3,3], Tg[:3,:3])

from .utils.utils import list2dict

def match_point_binder(graph, initial_state, objectPose_dict_mv):
    graph.set_object_state(initial_state)
    Q0 = initial_state.Q
    Q0dict = list2dict(Q0, graph.joint_names)
    binder_T_dict = {}
    binder_scale_dict = {}
    for k,binder in graph.binder_dict.items():
        binder_T = binder.get_tf_handle(Q0dict)
        binder_scale_dict[k] = binder.object.dims
        if binder.point is not None:
            binder_scale_dict[k] = 0
        binder_T_dict[k] = binder_T

    kpt_pair_dict = {}
    for kobj, Tobj in objectPose_dict_mv.items():

        min_val = 1e10
        min_point = ""
        if kobj not in graph.object_dict:
            continue
        for kpt, bd in graph.object_dict[kobj].action_points_dict.items():
            handle_T = bd.get_tf_handle(Q0dict)
            point_cur = handle_T[:3,3]
            direction_cur = handle_T[:3,2]

            for kbd, Tbd in binder_T_dict.items():
                if kobj == kbd or kobj == graph.binder_dict[kbd].object.name:
                    continue
                point_diff = Tbd[:3,3]-point_cur
                point_diff_norm = np.linalg.norm(np.maximum(np.abs(point_diff) - binder_scale_dict[kbd],0))
                dif_diff_norm = np.linalg.norm(direction_cur - Tbd[:3,2])
                bd_val_norm = point_diff_norm# + dif_diff_norm
                if bd_val_norm < min_val:
                    min_val = bd_val_norm
                    min_point = kbd
        kpt_pair_dict[kobj] = min_point
#         print("{} - {} ({})".format(kobj, min_point, min_val))
    return kpt_pair_dict

def register_hexahedral_binder(graph, object_name, _type):
    dims = graph.ghnd.NAME_DICT[object_name].dims
    for k in DIR_RPY_DICT.keys():
        rpy = DIR_RPY_DICT[k]
        point = tuple(-np.multiply(DIR_VEC_DICT[k], dims)/2)
        graph.register_binder(name="{}_{}".format(object_name, k), object_name=object_name, _type=_type,
                              point=point, rpy=rpy)



