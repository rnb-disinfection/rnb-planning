from ...detector.aruco.stereo import *
from ...constants import *
from threading import Thread
from .xacro_customizer import *
import rospy
from ...utils.utils import *
from ...marker_config import *


##
# @brief    select putting direction
# @param    Robj    object orientation
# @param    dir_vec_dict    direction vector dictionary {name: vec}
# @param    ref_vec     reference vector
# @return   key_match   matching vector name
def select_put_dir(Robj, dir_vec_dict, ref_vec=[[0], [0], [-1]]):
    downvec = np.matmul(np.transpose(Robj), ref_vec)
    dir_match_dict = {k: np.dot(v, downvec) for k, v in dir_vec_dict.items()}
    key_match = sorted(dir_match_dict.keys(), key=lambda k: dir_match_dict[k])[-1]
    return key_match

##
# @class    SceneBuilder
# @brief    Geometric scene builder
# @remark   Build geometric scene using a detector. All coordinates are converted relative to a reference coordinate.
#           call reset_reference_coord -> reset_ghnd
class SceneBuilder(Singleton):
    __rospy_initialized = False
    __roscore = None

    ##
    # @param detector    detector to use when building the scene
    # @param base_link   name of base link in urdf content
    def __init__(self, detector, base_link):
        self.detector = detector
        self.base_link = base_link

    ##
    # @brief re-detect reference coordinate - in case the camera has moved
    # @param ref_name   name of reference geometric item. this coordinate is synchronized with base link.
    def reset_reference_coord(self, ref_name):
        objectPose_dict = self.detector.detect(name_mask=[ref_name])
        self.ref_name = ref_name
        self.ref_coord = objectPose_dict[ref_name]
        self.ref_coord_inv = SE3_inv(self.ref_coord)

    ##
    # @brief reset geometry handle with new robot configuration
    # @param combined_robot     rnb-planning.src.pkg.controller.combined_robot.CombinedRobot
    # @param node_name          ros node name, by default "task_planner"
    # @param start_rviz         whether to start rviz or not default=True
    # @return joint_names       joint names
    # @return link_names        link names
    # @return urdf_content      urdf content
    def reset_ghnd(self, combined_robot, node_name='task_planner', start_rviz=True):
        robots_on_scene = combined_robot.robots_on_scene
        custom_limits   = combined_robot.custom_limits

        if not SceneBuilder.__rospy_initialized:
            SceneBuilder.__roscore = subprocess.Popen(['roscore'])
            rospy.init_node(node_name, anonymous=True)
            SceneBuilder.__rospy_initialized = True

        xcustom = XacroCustomizer.instance()
        xcustom.initialize(robots_on_scene)

        self.joint_names, self.link_names, self.urdf_content = xcustom.convert_xacro_to_urdf(
            joint_fix_dict={'finger': 'upper'}, joint_limit_dict=custom_limits)

        if start_rviz:
            xcustom.start_rviz()

        self.ghnd = GeometryHandle(self.urdf_content, self.joint_names, self.link_names, rviz=start_rviz)

        return self.ghnd, self.urdf_content, self.joint_names, self.link_names

    ##
    # @brief detect robots
    # @param item_names     List of string name for items
    # @param level_mask     List of rnb-planning.src.pkg.detector.detector_interface.DetectionLevel
    # @param as_matrix      flag to get transform matrix as-is
    # @return xyz_rpy_dict  Dictionary of detected item coordinates in xyz(m), rpy(rad)
    def detect_items(self, item_names=None, level_mask=None, as_matrix=False):
        xyz_rpy_dict = {}
        while True:
            try:
                objectPose_dict = self.detector.detect(name_mask=item_names, level_mask=level_mask)
                for okey in objectPose_dict.keys():
                    Tbr = np.matmul(self.ref_coord_inv, objectPose_dict[okey])
                    xyz_rpy_dict[okey] = Tbr if as_matrix else T2xyzrpy(Tbr)
                break
            except KeyError as e:
                print(e)
                break
            except Exception as e:
                print(e)
                pass
        return xyz_rpy_dict

    ##
    # @brief detect geometric items and register them in the geometry handle
    # @param item_names     List of string name for items
    # @param level_mask     List of rnb-planning.src.pkg.detector.detector_interface.DetectionLevel
    # @param ghnd   rnb-planning.src.pkg.geometry.geometry.GeometryHandle to add detected environment geometry
    # @return gtem_dict dictionary of detected geometry items
    def detect_and_register(self, item_names=None, level_mask=None, color=(0.6,0.6,0.6,1), collision=True, ghnd=None):
        if ghnd is None:
            ghnd = self.ghnd
        xyz_rpy_dict = self.detect_items(item_names=item_names, level_mask=level_mask)
        gtem_dict = {}
        for ename, xyzrpy in xyz_rpy_dict.items():
            kwargs = dict(name=ename, center=xyzrpy[0], rpy=xyzrpy[1], color=color,
                          link_name=self.base_link, collision=collision)
            kwargs.update(self.detector.get_geometry_kwargs(ename))
            gtem_dict[ename] = ghnd.create_safe(**kwargs)
        return gtem_dict

    ##
    # @brief add pole geometries to the scene
    # @param xyz_pole_top_dict  dictionary of pole top locations: {name: xyz}
    # @param thickness          thickness of the poles (m)
    # @param ghnd               rnb-planning.src.pkg.geometry.geometry.GeometryHandle to add poles
    def add_poles(self, xyz_pole_top_dict, thickness=0.15, color=(0.6,0.6,0.6,0.3), ghnd=None):
        if ghnd is None:
            ghnd = self.ghnd
        gtems = []
        for cname, xyz in xyz_pole_top_dict.items():
            gtems.append(ghnd.create_safe(name="pole_{}".format(cname), link_name=self.base_link,
                                          gtype=GEOTYPE.CAPSULE,
                                          center= np.subtract(xyz, [0,0,xyz[2]/2-thickness/2]),
                                          dims=(thickness, thickness, xyz[2]+thickness/2), rpy=(0,0,0),
                                          color=color, collision=True, fixed=True)
                         )
        return gtems

    ##
    # @brief add collision geometries for robot body
    def add_robot_geometries(self, color=None, display=True, collision=True, exclude_link=None):
        if color is None:
            color = (0, 1, 0, 0.5)
        if exclude_link is None:
            exclude_link = []
        urdf_content = self.urdf_content
        ghnd = self.ghnd
        geometry_items = []
        id_dict = defaultdict(lambda: -1)
        geometry_dir = "./geometry_tmp"
        try:
            os.mkdir(geometry_dir)
        except:
            pass
        for link in urdf_content.links:
            skip = False
            for ex_link in exclude_link:
                if ex_link in link.name:
                    skip = True
            if skip:
                continue
            for col_item in link.collisions:
                geometry = col_item.geometry
                geotype = geometry.__class__.__name__
                #             print("{}-{}".format(link.name, geotype))
                if col_item.origin is None:
                    xyz = [0, 0, 0]
                    rpy = [0, 0, 0]
                else:
                    xyz = col_item.origin.xyz
                    rpy = col_item.origin.rpy

                id_dict[link.name] += 1
                if geotype == 'Cylinder':
                    gname = "{}_{}_{}".format(link.name, geotype, id_dict[link.name])
                    geometry_items.append(
                        ghnd.create_safe(
                            name=gname, link_name=link.name, gtype=GEOTYPE.CAPSULE,
                            center=xyz, dims=(geometry.radius * 2, geometry.radius * 2, geometry.length), rpy=rpy,
                            color=color, display=display, collision=collision, fixed=True)
                    )
                elif geotype == 'Box':
                    gname = "{}_{}_{}".format(link.name, geotype, id_dict[link.name])
                    geometry_items.append(
                        ghnd.create_safe(
                            name=gname, link_name=link.name, gtype=GEOTYPE.BOX,
                            center=xyz, dims=geometry.size, rpy=rpy,
                            color=color, display=display, collision=collision, fixed=True)
                    )
                elif geotype == 'Sphere':
                    gname = "{}_{}_{}".format(link.name, geotype, id_dict[link.name])
                    geometry_items.append(
                        ghnd.create_safe(
                            name=gname, link_name=link.name, gtype=GEOTYPE.SPHERE,
                            center=xyz, dims=[geometry.radius * 2] * 3, rpy=rpy,
                            color=color, display=display, collision=collision, fixed=True)
                    )
                elif geotype == 'Mesh':
                    raise (NotImplementedError("Mesh collision boundary is not supported"))
                else:
                    raise (NotImplementedError("collision geometry {} is not implemented".format(geotype)))
        return geometry_items

##
# @class DynamicDetector
# @brief Dynamic detector wrapper of SceneBuilder
class DynamicDetector:
    def __init__(self, scene_builder, object_names):
        self.scene_builder, self.object_names = scene_builder, object_names

    def detector_thread_fun(self):
        self.detector_stop = False
        self.dynPos_dict = {}
        while not self.detector_stop:
            T_dict = self.scene_builder.detect_items(item_names=self.object_names, as_matrix=True)
            for k, T in T_dict.items():
                self.dynPos_dict[k] = np.matmul(SE3_inv(self.ref_T), np.matmul(self.T_c12, T))

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

##
# @class RvizPublisher
# @brief rviz publisher for DynamicDetector
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


##
# @ get current put point --> move to planning scene function
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
            put_point_dict[mtem] = select_put_dir(Robj=Robj,
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
            up_point_dict[mtem] = select_put_dir(Robj=np.matmul(Rx180, Robj),
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
    return xyz_rpy_mv_dict, put_point_dict, up_point_dict

def update_geometries(ghnd, onames, objectPose_dict_mv, refFrame):
    refFrameinv = SE3_inv(refFrame)
    for gname in onames:
        gtem = [gtem for gtem in ghnd if gtem.name == gname]
        if len(gtem)>0 and gname in objectPose_dict_mv:
            gtem = gtem[0]
            Tg = np.matmul(refFrameinv, objectPose_dict_mv[gname])
            gtem.set_offset_tf(Tg[:3,3], Tg[:3,:3])

from ...utils.utils import list2dict

def match_point_binder(graph, initial_state, objectPose_dict_mv):
    graph.set_object_state(initial_state)
    Q0 = initial_state.Q
    Q0dict = list2dict(Q0, graph.joint_names)
    binder_T_dict = {}
    binder_scale_dict = {}
    for k,binder in graph.binder_dict.items():
        binder_T = binder.get_tf_handle(Q0dict)
        binder_scale_dict[k] = binder.geometry.dims
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
                if kobj == kbd or kobj == graph.binder_dict[kbd].geometry.name:
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



