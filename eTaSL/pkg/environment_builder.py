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

def add_cam_poles(graph, xyz_rvec_cams, color=(0.6,0.6,0.6,1), link_name="world"):
    graph.add_geometry_items([GeoBox(np.subtract(xyzrvec[0], [0,0,xyzrvec[0][2]/2-0.1]),
                                     (0.15,0.15,xyzrvec[0][2]),
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

def register_objects(graph, objectPose_dict_mv, object_generators, binder_dict, object_dict, ref_tuple, link_name="world"):
    objectPose_dict_mv.update({ref_tuple[0]: ref_tuple[1]})
    xyz_rvec_mv_dict, put_point_dict = calc_put_point(objectPose_dict_mv, object_generators, object_dict, ref_tuple)

    for mname, mgen in object_generators.items():
        if mname in xyz_rvec_mv_dict:
            xyz_rvec = xyz_rvec_mv_dict[mname]
            graph.add_geometry_items([mgen(*xyz_rvec, name=mname,
                                           link_name=link_name, urdf_content=graph.urdf_content, color=(0.3, 0.3, 0.8, 1),
                                           collision=True)],
                                     fixed=False)

    for bname, bkwargs in binder_dict.items():
        graph.register_binder(name=bname, **bkwargs)

    for mtem, xyz_rvec in xyz_rvec_mv_dict.items():
        if mtem in put_point_dict:
            graph.register_object(mtem, binding=(put_point_dict[mtem], "floor"), **object_dict[mtem])

    return put_point_dict

def calc_put_point(objectPose_dict_mv, object_generators, object_dict, ref_tuple):
    T_mv_dict = {mname: get_T_rel(ref_tuple[0], mname, objectPose_dict_mv) for mname in object_generators if
                 mname in objectPose_dict_mv}
    xyz_rvec_mv_dict = {mname: T2xyzrvec(Tv) for mname, Tv in T_mv_dict.items()}

    put_point_dict = {}
    for mtem, xyz_rvec in xyz_rvec_mv_dict.items():
        if mtem in object_dict:
            put_point_dict[mtem] = get_put_dir(Robj=Rotation.from_rotvec(xyz_rvec[1]).as_dcm(),
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
    return xyz_rvec_mv_dict, put_point_dict


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