from ...detector.aruco.stereo import *
from ...constants import *
from threading import Thread
from .xacro_customizer import *
import rospy
from ...utils.utils import *


##
# @brief    wait until rviz is started
def wait_rviz_on():
    while True:
        time.sleep(0.1)
        topic_names = [tp[0] for tp in rospy.get_published_topics()]
        if '/initialpose' in topic_names:
            break


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
#           call reset_reference_coord -> create_gscene
class SceneBuilder(Singleton):
    __rospy_initialized = False
    __roscore = None

    ##
    # @param detector    detector to use when building the scene
    # @param base_link   name of base link in urdf content
    def __init__(self, detector, base_link="base_link"):
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
    # @brief create and reset geometry handle with new robot configuration
    # @param combined_robot     rnb-planning.src.pkg.controller.combined_robot.CombinedRobot
    # @param node_name          ros node name, by default "task_planner"
    # @param start_rviz         whether to start rviz or not default=True
    # @return joint_names       joint names
    # @return link_names        link names
    # @return urdf_content      urdf content
    def create_gscene(self, combined_robot, node_name='task_planner', start_rviz=True, gscene_from=None):
        self.combined_robot = combined_robot
        robots_on_scene = combined_robot.robots_on_scene
        custom_limits   = combined_robot.custom_limits

        if not SceneBuilder.__rospy_initialized:
            SceneBuilder.__roscore = subprocess.Popen(['roscore'])
            rospy.init_node(node_name, anonymous=True)
            SceneBuilder.__rospy_initialized = True

        self.xcustom = XacroCustomizer.instance()
        self.xcustom.initialize(robots_on_scene)

        joint_names, link_names, urdf_content, urdf_path = self.xcustom.convert_xacro_to_urdf(
            joint_fix_dict={'finger': 'upper'}, joint_limit_dict=custom_limits)

        if start_rviz:
            self.xcustom.start_rviz()
            wait_rviz_on()

        if gscene_from:
            GeometryScene.__init__(gscene_from, urdf_content, urdf_path, joint_names, link_names, rviz=start_rviz)
            self.gscene = gscene_from
        else:
            self.gscene = GeometryScene(urdf_content, urdf_path, joint_names, link_names, rviz=start_rviz)

        return self.gscene

    ##
    # @brief detect robots
    # @param item_names     List of string name for items
    # @param level_mask     List of rnb-planning.src.pkg.detector.detector_interface.DetectionLevel
    # @param as_matrix      flag to get transform matrix as-is
    # @return xyz_rpy_dict  Dictionary of detected item coordinates in xyz(m), rpy(rad)
    def detect_items(self, item_names=None, level_mask=None, as_matrix=False, visualize=False):
        xyz_rpy_dict = {}
        while True:
            try:
                objectPose_dict = self.detector.detect(name_mask=item_names, level_mask=level_mask, visualize=visualize)
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
    # @param gscene   rnb-planning.src.pkg.geometry.geometry.GeometryScene to add detected environment geometry
    # @return gtem_dict dictionary of detected geometry items
    def detect_and_register(self, item_names=None, level_mask=None, color=(0.6,0.6,0.6,1), collision=True,
                            gscene=None, visualize=False):
        if gscene is None:
            gscene = self.gscene
        xyz_rpy_dict = self.detect_items(item_names=item_names, level_mask=level_mask, visualize=visualize)
        gtem_dict = {}
        for ename, xyzrpy in xyz_rpy_dict.items():
            kwargs = dict(name=ename, center=xyzrpy[0], rpy=xyzrpy[1], color=color,
                          link_name=self.base_link, collision=collision)
            kwargs.update(self.detector.get_geometry_kwargs(ename))
            gtem_dict[ename] = gscene.create_safe(**kwargs)
        return gtem_dict

    ##
    # @brief shift geometry item slightly to make sure there is a clearance from the floor
    # @param floor GeometryItem for floor
    # @param gtem_list list of GeometryItem to shift
    # @param clearance the amount of clearance to give, in m units
    def give_clearance(self, floor, gtem_list, clearance=1e-3):
        Ttrack = floor.get_tf(self.combined_robot.home_dict)
        Ttrack_inv = SE3_inv(Ttrack)
        for gtem in gtem_list:
            verts, _ = gtem.get_vertice_radius_from(self.combined_robot.home_dict, from_link="base_link")
            verts_loc = np.matmul(Ttrack_inv[:3, :3], verts.transpose()) + Ttrack_inv[:3, 3:]
            off = floor.dims[2] / 2 - np.min(verts_loc[2, :]) + clearance
            center_loc = np.matmul(Ttrack_inv[:3, :3], gtem.center) + Ttrack_inv[:3, 3]
            center_loc_new = center_loc + [0, 0, off]
            center_new = np.matmul(Ttrack[:3, :3], center_loc_new) + Ttrack[:3, 3]
            gtem.set_offset_tf(center=center_new)

    ##
    # @brief add pole geometries to the scene
    # @param xyz_pole_top_dict  dictionary of pole top locations: {name: xyz}
    # @param thickness          thickness of the poles (m)
    # @param gscene               rnb-planning.src.pkg.geometry.geometry.GeometryScene to add poles
    def add_poles(self, xyz_pole_top_dict, thickness=0.15, color=(0.6,0.6,0.6,0.3), gscene=None):
        if gscene is None:
            gscene = self.gscene
        gtems = []
        for cname, xyz in xyz_pole_top_dict.items():
            gtems.append(gscene.create_safe(name="pole_{}".format(cname), link_name=self.base_link,
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
        gscene = self.gscene
        geometry_items = []
        id_dict = defaultdict(lambda: -1)
        geometry_dir = "./geometry_tmp"
        try:
            os.mkdir(geometry_dir)
        except:
            pass
        for link in gscene.urdf_content.links:
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
                        gscene.create_safe(
                            name=gname, link_name=link.name, gtype=GEOTYPE.CYLINDER,
                            center=xyz, dims=(geometry.radius * 2, geometry.radius * 2, geometry.length), rpy=rpy,
                            color=color, display=display, collision=collision, fixed=True)
                    )
                elif geotype == 'Box':
                    gname = "{}_{}_{}".format(link.name, geotype, id_dict[link.name])
                    geometry_items.append(
                        gscene.create_safe(
                            name=gname, link_name=link.name, gtype=GEOTYPE.BOX,
                            center=xyz, dims=geometry.size, rpy=rpy,
                            color=color, display=display, collision=collision, fixed=True)
                    )
                elif geotype == 'Sphere':
                    gname = "{}_{}_{}".format(link.name, geotype, id_dict[link.name])
                    geometry_items.append(
                        gscene.create_safe(
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
    def __init__(self, gscene, obs_names=[]):
        self.gscene, self.obs_names = gscene, obs_names
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
                    self.gscene.NAME_DICT[oname].set_offset_tf(T_bo[:3, 3], T_bo[:3, :3])
            self.gscene.show_pose(self.POS_CUR)

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