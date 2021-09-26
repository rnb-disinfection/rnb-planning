import numpy as np
from .filter_interface import MotionFilterInterface, save_scene
from ...utils.joint_utils import *
from ...utils.utils import *
from ...utils.gjk import get_point_list, get_gjk_distance

C_SVM_DEFAULT = 1000
GAMMA_SVM_DEFAULT = 'scale'

DEBUG_REACH_FILT_LOG = False

if DEBUG_REACH_FILT_LOG:
    TextColors.RED.println("===== WARNING: reach_filter in DEBUG MODE====")

def T2features(T, shoulder_height):
    xyz = T[:3, 3]
    R_mat = T[:3,:3]
    radius, theta, height = cart2cyl(*xyz)
    azimuth_loc, zenith = mat2hori(R_mat, theta=theta)
    R_mat_ref = hori2mat(theta, azimuth_loc, zenith)
    R_z = np.matmul(R_mat_ref.transpose(), R_mat)
    rot_z = Rot2axis(R_z, 3)
    ee_dist = np.linalg.norm([radius, height-shoulder_height])
    return radius,theta, height, azimuth_loc, zenith, ee_dist, rot_z

def xyzquat2features(xyzquat, shoulder_height):
    xyz = xyzquat[:3]
    quat = xyzquat[3:]
    R_mat = Rotation.from_quat(quat).as_dcm()
    return T2features(SE3(R_mat, xyz), shoulder_height)

def features2T(radius,theta, height, azimuth_loc, zenith, ee_dist, rot_z):
    xyz = cyl2cart(radius,theta, height)
    R_mat_ref = hori2mat(theta, azimuth_loc, zenith)
    R_mat = np.matmul(R_mat_ref, Rot_axis(3, rot_z))
    xyz = cyl2cart(radius, theta, height)
    return SE3(R_mat, xyz)

def features2xyzquat(radius,theta, height, azimuth_loc, zenith, ee_dist, rot_z):
    T = features2T(radius,theta, height, azimuth_loc, zenith, ee_dist, rot_z)
    xyzquat = T2xyzquat(T)
    goal_pose = xyzquat[0]+xyzquat[1]
    return goal_pose

def to_featurevec(radius,theta, height, azimuth_loc, zenith, ee_dist, rot_z):
    return (radius, theta, height, azimuth_loc, zenith, radius**2, ee_dist, ee_dist**2, rot_z)

##
# @class    ReachChecker
# @brief    check reach regarding kinematic chain
# @remark   You need to train ReachTrainer tu use ReachChecker. See src/scripts/training/ReachSVM.ipynb
class ReachChecker(MotionFilterInterface):
    BEFORE_IK = True

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene
    def __init__(self, pscene, feature_fn=to_featurevec):
        self.pscene = pscene
        self.feature_fn = feature_fn
        self.combined_robot = pscene.combined_robot
        self.robot_names = pscene.combined_robot.robot_names
        chain_dict = pscene.robot_chain_dict
        binder_links = [chain_dict[rname]['tip_link'] for rname in self.robot_names]
        self.binder_link_robot_dict = {blink: rname for blink, rname in zip(binder_links, self.robot_names)}

        self.model_dict = {}
        for rconfig in self.combined_robot.robots_on_scene:
            self.model_dict[rconfig.get_indexed_name()] = ReachTrainer(None).load_model(rconfig.type)
        self.base_dict = self.combined_robot.get_robot_base_dict()
        self.shoulder_link_dict = {rname: pscene.gscene.urdf_content.joint_map[rchain['joint_names'][1]].child
                                   for rname, rchain in chain_dict.items()}
        self.shoulder_height_dict = {rname: get_tf(shoulder_link, self.combined_robot.home_dict,
                                                   pscene.gscene.urdf_content, from_link=self.base_dict[rname])[2,3]
                                     for rname, shoulder_link in self.shoulder_link_dict.items()}
        self.shoulder_reach_dict = {
            rname: RobotSpecs.get_shoulder_reach(self.combined_robot.get_robot_config_dict()[rname].type)
            for rname in self.shoulder_link_dict.keys()}

    ##
    # @param btf    BindingTransorm instance
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    # @param ignore         GeometryItems to ignore
    def check(self, btf, Q_dict, interpolate=False, **kwargs):
        obj, handle, actor = btf.get_instance_chain(self.pscene)
        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name
        T_loal = btf.T_loal

        group_name_handle = self.binder_link_robot_dict[object_link] if object_link in self.binder_link_robot_dict else None
        group_name_actor = self.binder_link_robot_dict[actor_link] if actor_link in self.binder_link_robot_dict else None

        if group_name_actor and not group_name_handle:
            group_name = group_name_actor
            T_handle_link = get_tf(object_link, Q_dict, self.pscene.gscene.urdf_content,
                                   from_link=self.base_dict[group_name])
            T_tar = np.matmul(T_handle_link, T_loal)
        elif group_name_handle and not group_name_actor:
            group_name = group_name_handle
            T_actor_link = get_tf(actor_link, Q_dict, self.pscene.gscene.urdf_content,
                                  from_link=self.base_dict[group_name])
            T_tar = np.matmul(T_actor_link, SE3_inv(T_loal))
        else:
            # dual motion not predictable
            return True

        shoulder_height = self.shoulder_height_dict[group_name]
        features = T2features(T_tar, shoulder_height)
        radius, theta, height, azimuth_loc, zenith, ee_dist, rot_z = features
        if ee_dist > self.shoulder_reach_dict[group_name]:
            res = False
        else:
            featurevec = self.feature_fn(*features)
            res = self.model_dict[group_name].predict([featurevec])[0]

        if DEBUG_REACH_FILT_LOG:
            save_scene(self.__class__.__name__, self.pscene, btf, Q_dict,
                       error_state=False, result=res, **kwargs)
        return res


from ...utils.utils import *
from ...controller.combined_robot import CombinedRobot, RobotConfig
from ...controller.robot_config import RobotSpecs
from ...planning.scene import PlanningScene
from ...planning.motion.moveit.moveit_planner import MoveitPlanner
from ...planning.constraint.constraint_actor import Gripper2Tool
from ...geometry.geometry import GEOTYPE
import random
import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm

DATA_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data")
try_mkdir(DATA_PATH)
MODEL_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "model")
try_mkdir(MODEL_PATH)


##
# @class    ReachChecker
# @brief    check reach regarding kinematic chain
# @remark   to train ReachChecker, see src/scripts/training/ReachSVM.ipynb
#           used features are: (radius,theta, height, azimuth_loc, zenith, radius**2, ee_dist, ee_dist**2)
class ReachTrainer:
    ##
    # @param scene_builder  scene builder is required to make data scene (rnb-planning.src.pkg.geometry.builder.scene_builder.SceneBuilder)
    #                       not needed if not training the algorithm
    def __init__(self, scene_builder=None):
        self.algorithm_name = 'reach_svm'
        self.scene_builder = scene_builder ##< rnb-planning.src.pkg.geometry.builder.scene_builder.SceneBuilder

        self.data_path = os.path.join(DATA_PATH, self.algorithm_name)
        try_mkdir(self.data_path)
        self.model_path = os.path.join(MODEL_PATH, self.algorithm_name)
        try_mkdir(self.model_path)

    ##
    # @brief collect and learn
    def collect_and_learn(self, ROBOT_TYPE, END_LINK, TRAIN_COUNT=10000, TEST_COUNT=10000,
                          save_data=True, save_model=True, C_svm=C_SVM_DEFAULT, 
                          gamma=GAMMA_SVM_DEFAULT, timeout=1, try_num=1, feature_fn=to_featurevec):
        self.samplevec_list_train, self.success_list_train = self.collect_reaching_data(ROBOT_TYPE, END_LINK, TRAIN_COUNT, timeout=timeout, try_num=try_num)
        self.samplevec_list_test, self.success_list_test = self.collect_reaching_data(ROBOT_TYPE, END_LINK, TEST_COUNT, timeout=timeout, try_num=try_num)

        if save_data:
            self.save_data("train", self.samplevec_list_train, self.success_list_train)
            self.save_data("test", self.samplevec_list_test, self.success_list_test)

        self.featurevec_list_train = map(lambda x: feature_fn(*xyzquat2features(x, self.shoulder_height)), self.samplevec_list_train)
        self.featurevec_list_test = map(lambda x: feature_fn(*xyzquat2features(x, self.shoulder_height)), self.samplevec_list_test)

        feature_mat_train = np.array(self.featurevec_list_train)
        self.clf = svm.SVC(kernel='rbf', C=C_svm, gamma=gamma)
        self.clf.fit(feature_mat_train, self.success_list_train)
        if save_model:
            self.save_model()


        feature_mat_train = np.array(self.featurevec_list_train)
        feature_mat_test = np.array(self.featurevec_list_test)

        gtimer = GlobalTimer.instance()
        gtimer.reset()
        with gtimer.block("trainset"):
            train_res = np.equal(self.clf.predict(feature_mat_train), self.success_list_train)
        with gtimer.block("testset"):
            test_res = np.equal(self.clf.predict(feature_mat_test), self.success_list_test)
        print("=" * 80)
        print(gtimer)

        print("=" * 80)
        print("trainning accuracy = {} %".format(round(np.mean(train_res) * 100, 2)))
        print("test accuracy = {} %".format(round(np.mean(test_res) * 100, 2)))
        print("=" * 80)
        print("trainning success accuracy = {} %".format(
            round(np.mean(train_res[np.where(self.success_list_train)]) * 100, 2)))
        print("trainning failure accuracy = {} %".format(
            round(np.mean(train_res[np.where(np.logical_not(self.success_list_train))]) * 100, 2)))
        print("=" * 80)
        print("test success accuracy = {} %".format(round(np.mean(test_res[np.where(self.success_list_test)]) * 100, 2)))
        print("test failure accuracy = {} %".format(
            round(np.mean(test_res[np.where(np.logical_not(self.success_list_test))]) * 100, 2)))
        print("=" * 80)

    ##
    # @brief load and learn
    def load_and_learn(self, ROBOT_TYPE, C_svm=C_SVM_DEFAULT, gamma=GAMMA_SVM_DEFAULT, save_model=True, feature_fn=to_featurevec):
        self.set_shoulder_height(ROBOT_TYPE)

        self.samplevec_list_train, self.success_list_train = self.load_data(ROBOT_TYPE, "train")
        self.samplevec_list_test, self.success_list_test = self.load_data(ROBOT_TYPE, "test")

        self.featurevec_list_train = map(lambda x: feature_fn(*xyzquat2features(x, self.shoulder_height)), self.samplevec_list_train)
        self.featurevec_list_test = map(lambda x: feature_fn(*xyzquat2features(x, self.shoulder_height)), self.samplevec_list_test)

        feature_mat_train = np.array(self.featurevec_list_train)
        feature_mat_test = np.array(self.featurevec_list_test)

        self.clf = svm.SVC(kernel='rbf', C=C_svm, gamma=gamma)
        self.clf.fit(feature_mat_train, self.success_list_train)
        if save_model:
            self.save_model()

        gtimer = GlobalTimer.instance()
        gtimer.reset()
        with gtimer.block("trainset"):
            train_res = np.equal(self.clf.predict(feature_mat_train), self.success_list_train)
        with gtimer.block("testset"):
            test_res = np.equal(self.clf.predict(feature_mat_test), self.success_list_test)
        print("=" * 80)
        print(gtimer)

        print("=" * 80)
        print("trainning accuracy = {} %".format(round(np.mean(train_res) * 100, 2)))
        print("test accuracy = {} %".format(round(np.mean(test_res) * 100, 2)))
        print("=" * 80)
        print("trainning success accuracy = {} %".format(
            round(np.mean(train_res[np.where(self.success_list_train)]) * 100, 2)))
        print("trainning failure accuracy = {} %".format(
            round(np.mean(train_res[np.where(np.logical_not(self.success_list_train))]) * 100, 2)))
        print("=" * 80)
        print("test success accuracy = {} %".format(round(np.mean(test_res[np.where(self.success_list_test)]) * 100, 2)))
        print("test failure accuracy = {} %".format(
            round(np.mean(test_res[np.where(np.logical_not(self.success_list_test))]) * 100, 2)))
        print("=" * 80)
        return round(np.mean(test_res) * 100, 2)

    def set_shoulder_height(self, ROBOT_TYPE):
        crob = CombinedRobot(robots_on_scene=[
            RobotConfig(0, ROBOT_TYPE, ((0, 0, 0), (0, 0, 0)),
                        None)]
            , connection_list=[False])

        gscene = self.scene_builder.create_gscene(crob, start_rviz=False)

        shoulder_link = gscene.urdf_content.joint_map[gscene.joint_names[1]].child
        self.shoulder_height = get_tf(shoulder_link, crob.home_dict, gscene.urdf_content)[2, 3]

    ##
    # @brief load and test
    def load_and_test(self, ROBOT_TYPE, feature_fn=to_featurevec):
        self.samplevec_list_train, self.success_list_train = self.load_data(ROBOT_TYPE, "train")
        self.samplevec_list_test, self.success_list_test = self.load_data(ROBOT_TYPE, "test")

        self.featurevec_list_train = map(lambda x: feature_fn(*xyzquat2features(x, self.shoulder_height)), self.samplevec_list_train)
        self.featurevec_list_test = map(lambda x: feature_fn(*xyzquat2features(x, self.shoulder_height)), self.samplevec_list_test)

        feature_mat_train = np.array(self.featurevec_list_train)
        feature_mat_test = np.array(self.featurevec_list_test)

        self.load_model(ROBOT_TYPE)

        gtimer = GlobalTimer.instance()
        gtimer.reset()
        with gtimer.block("trainset"):
            train_res = np.equal(self.clf.predict(feature_mat_train), self.success_list_train)
        with gtimer.block("testset"):
            test_res = np.equal(self.clf.predict(feature_mat_test), self.success_list_test)
        print("=" * 80)
        print(gtimer)

        print("=" * 80)
        print("trainning accuracy = {} %".format(round(np.mean(train_res) * 100, 2)))
        print("test accuracy = {} %".format(round(np.mean(test_res) * 100, 2)))
        print("=" * 80)
        print("trainning success accuracy = {} %".format(
            round(np.mean(train_res[np.where(self.success_list_train)]) * 100, 2)))
        print("trainning failure accuracy = {} %".format(
            round(np.mean(train_res[np.where(np.logical_not(self.success_list_train))]) * 100, 2)))
        print("=" * 80)
        print("test success accuracy = {} % ({}/{})".format(
            round(np.mean(test_res[np.where(self.success_list_test)]) * 100, 2),
            np.sum(test_res[np.where(self.success_list_test)]), len(np.where(self.success_list_test)[0])
        )
        )
        print("test failure accuracy = {} % ({}/{})".format(
            round(np.mean(test_res[np.where(np.logical_not(self.success_list_test))]) * 100, 2),
            np.sum(test_res[np.where(np.logical_not(self.success_list_test))]),
            len(np.where(np.logical_not(self.success_list_test))[0])
        )
        )
        print("=" * 80)
        return round(np.mean(test_res) * 100, 2)

    ##
    # @brief sample reaching plan results
    # @remark accuracy decreases when very small radius is included in the dataset. radius lowerbound 0.2 is recommended
    def sample_reaching(self, robot_name, tool_link, home_pose, base_link="base_link", timeout=1,
                        radius_min=0.2, radius_max=1.5, theta_min=-np.pi, theta_max=np.pi,
                        height_min=-0.7, height_max=1.5, zenith_min=0, zenith_max=np.pi,
                        azimuth_min=-np.pi, azimuth_max=np.pi, rot_z_min=-np.pi,rot_z_max=np.pi, try_num=1):
        while True: # sample in cartesian space with rejection, for uniform sampling
            xyz = np.random.uniform(-radius_max, radius_max, size=3) + [0,0,self.shoulder_height]
            radius, theta, height = cart2cyl(*xyz)
            if radius_min == radius_max:
                radius = radius_min
            if theta_min == theta_max:
                theta = theta_min
            if height_min == height_max:
                height = height_min
            if not radius_min<=radius<=radius_max:
                continue
            if not theta_min<=theta<=theta_max:
                continue
            if not height_min<=height<=height_max:
                continue
            ee_dist = np.linalg.norm([radius, height-self.shoulder_height])
            if ee_dist>radius_max and not radius_min == radius_max:
                continue
            break
        azimuth_loc = np.random.uniform(azimuth_min, azimuth_max)
        zenith = np.arccos(np.random.uniform(-np.cos(zenith_min), -np.cos(zenith_max)))
        rot_z = np.random.uniform(rot_z_min, rot_z_max)
        R_mat_ref = hori2mat(theta, azimuth_loc, zenith)
        R_mat = np.matmul(R_mat_ref, Rot_axis(3, rot_z))

        xyz = cyl2cart(radius, theta, height)
        quat = tuple(Rotation.from_dcm(R_mat).as_quat())
        goal_pose = xyz+quat
        GlobalTimer.instance().tic("plan_py")
        for _ in range(try_num):
            trajectory, success = self.planner.planner.plan_py(
                robot_name, tool_link, goal_pose, base_link, tuple(home_pose), timeout=timeout)
            if success:
                break
        self.time_plan.append(GlobalTimer.instance().toc("plan_py"))

        features_recalc = xyzquat2features(goal_pose, self.shoulder_height)
        goal_pose_recalc = features2xyzquat(*features_recalc)
        assert np.sum(np.abs(
            np.subtract(features_recalc,
                        (radius,theta, height, azimuth_loc, zenith, ee_dist, rot_z)))
        )<1e-5, "feature transformation mismatch"
        assert np.sum(np.abs(
            np.subtract(goal_pose_recalc, goal_pose))
        )<1e-5, "goal pose transformation mismatch"

        return goal_pose, success, trajectory

    def collect_reaching_data(self, robot_type, TIP_LINK, N_s, timeout=1, try_num=1):
        self.robot_type = robot_type
        # set robot
        crob = CombinedRobot(robots_on_scene=[
            RobotConfig(0, robot_type, None,"")], connection_list=[False])
        ROBOT_NAME = crob.robot_names[0]
        xyz_rpy_robots = {ROBOT_NAME: ((0,0,0), (0,0,0))}
        crob.update_robot_pos_dict(xyz_rpy_robots=xyz_rpy_robots)

        # create scene
        gscene = self.scene_builder.create_gscene(crob, start_rviz=False)
        self.scene_builder.add_robot_geometries(color=(0, 1, 0, 0.5), display=True, collision=True)
        print("added robot collision boundaries")
        pscene = PlanningScene(gscene, combined_robot=crob)

        self.shoulder_link = gscene.urdf_content.joint_map[gscene.joint_names[1]].child
        self.shoulder_height = get_tf(self.shoulder_link, crob.home_dict, gscene.urdf_content)[2,3]

        # make dummy binders
        gscene.create_safe(gtype=GEOTYPE.SPHERE, name="grip0", link_name=TIP_LINK,
                           dims=(0.01,)*3, center=(0,0,0.0), rpy=(-np.pi/2,0,0), color=(1,0,0,1), display=True, collision=False, fixed=True)
        pscene.create_binder(bname="grip0", gname="grip0", _type=Gripper2Tool, point=(0,0,0), rpy=(0,0,0))

        self.planner = MoveitPlanner(pscene)
        self.planner.update_gscene()

        self.time_plan = []
        gtimer = GlobalTimer.instance()
        gtimer.reset()
        gtimer.tic("full_loop")
        samplevec_list = []
        success_list = []
        self.time_list = []
        for i_s in range(N_s):
            gtimer.tic("sample_reaching")
            samplevec, success, trajectory = self.sample_reaching(ROBOT_NAME, TIP_LINK, home_pose=crob.home_pose, timeout=timeout,
                                                                   radius_max=RobotSpecs.get_shoulder_reach(robot_type), try_num=try_num)
            self.time_list.append(gtimer.toc("sample_reaching"))
            # xyz = cyl2cart(*featurevec[:3])
            # orientation_mat = hori2mat(featurevec[1], *featurevec[-2:])
    #         gscene.add_highlight_axis("hl", "toolvec", "base_link", xyz, orientation_mat)
            samplevec_list.append(samplevec)
            success_list.append(success)
            if i_s % 100 == 0 :
                t_cur = gtimer.toc("full_loop")
                print("{} / {} ({} / {} s): current success ratio = {}".format(
                    i_s, N_s, int(t_cur/1000), int(float(N_s)/float(i_s+1e-3)*t_cur/1000), np.mean(success_list)))

        return samplevec_list, success_list

    def update_label(self, robot_type, tip_link, data_div, update_labels, timeout=1):
        self.robot_type = robot_type
        # set robot
        crob = CombinedRobot(robots_on_scene=[
            RobotConfig(0, robot_type, None,"")], connection_list=[False])
        ROBOT_NAME = crob.robot_names[0]
        xyz_rpy_robots = {ROBOT_NAME: ((0,0,0), (0,0,0))}
        crob.update_robot_pos_dict(xyz_rpy_robots=xyz_rpy_robots)

        # create scene
        gscene = self.scene_builder.create_gscene(crob, start_rviz=False)
        self.scene_builder.add_robot_geometries(color=(0, 1, 0, 0.5), display=True, collision=True)
        print("added robot collision boundaries")
        pscene = PlanningScene(gscene, combined_robot=crob)

        self.shoulder_link = gscene.urdf_content.joint_map[gscene.joint_names[1]].child
        self.shoulder_height = get_tf(self.shoulder_link, crob.home_dict, gscene.urdf_content)[2,3]

        # make dummy binders
        gscene.create_safe(gtype=GEOTYPE.SPHERE, name="grip0", link_name=tip_link,
                           dims=(0.01,)*3, center=(0,0,0.0), rpy=(-np.pi/2,0,0), color=(1,0,0,1), display=True, collision=False, fixed=True)
        pscene.create_binder(bname="grip0", gname="grip0", _type=Gripper2Tool, point=(0,0,0), rpy=(0,0,0))

        self.planner = MoveitPlanner(pscene)
        self.planner.update_gscene()

        featurevec_list, success_list = self.load_data(robot_type, data_div)
        N_s = len(featurevec_list)

        self.time_plan = []
        gtimer = GlobalTimer.instance()
        gtimer.reset()
        gtimer.tic("full_loop")
        self.time_list = []
        for i_s, (featurevec, succ) in enumerate(zip(featurevec_list, success_list)):
            if succ not in update_labels:
                continue
            radius, theta, height, azimuth_loc, zenith, _, _, _ = featurevec
            gtimer.tic("sample_reaching")
            featurevec, success, trajectory = self.sample_reaching(ROBOT_NAME, tip_link, home_pose=crob.home_pose, timeout=timeout,
                                                                   radius_min=radius, radius_max=radius, theta_min=theta, theta_max=theta,
                                                                   height_min=height, height_max=height, zenith_min=zenith, zenith_max=zenith,
                                                                   azimuth_min=azimuth_loc, azimuth_max=azimuth_loc)
            if success:
                self.time_list.append(gtimer.toc("sample_reaching"))
            assert np.linalg.norm(np.subtract(featurevec_list[i_s], featurevec))<1e-5, "featurevec changed"
            success_list[i_s] = success
            if i_s % 100 == 0 :
                t_cur = gtimer.toc("full_loop")
                print("{} / {} ({} / {} s): current success ratio = {}".format(
                    i_s, N_s, int(t_cur/1000), int(float(N_s)/float(i_s+1e-3)*t_cur/1000), np.mean(success_list)))
        self.save_data(data_div, featurevec_list, success_list)

    def load_and_visualize(self, robot_type, tip_link, data_div, timeout=1):
        self.robot_type = robot_type
        # set robot
        crob = CombinedRobot(robots_on_scene=[
            RobotConfig(0, robot_type, None,"")], connection_list=[False])
        ROBOT_NAME = crob.robot_names[0]
        xyz_rpy_robots = {ROBOT_NAME: ((0,0,0), (0,0,0))}
        crob.update_robot_pos_dict(xyz_rpy_robots=xyz_rpy_robots)

        # create scene
        gscene = self.scene_builder.create_gscene(crob, start_rviz=True)
        self.scene_builder.add_robot_geometries(color=(0, 1, 0, 0.5), display=True, collision=True)
        print("added robot collision boundaries")
        pscene = PlanningScene(gscene, combined_robot=crob)

        self.shoulder_link = gscene.urdf_content.joint_map[gscene.joint_names[1]].child
        self.shoulder_height = get_tf(self.shoulder_link, crob.home_dict, gscene.urdf_content)[2,3]

        # make dummy binders
        gscene.create_safe(gtype=GEOTYPE.SPHERE, name="grip0", link_name=tip_link,
                           dims=(0.01,)*3, center=(0,0,0.0), rpy=(-np.pi/2,0,0), color=(1,0,0,1), display=True, collision=False, fixed=True)
        pscene.create_binder(bname="grip0", gname="grip0", _type=Gripper2Tool, point=(0,0,0), rpy=(0,0,0))

        self.planner = MoveitPlanner(pscene)
        self.planner.update_gscene()

        featurevec_list, success_list = self.load_data(robot_type, data_div)
        N_s = len(featurevec_list)

        self.time_plan = []
        gtimer = GlobalTimer.instance()
        gtimer.reset()
        gtimer.tic("full_loop")
        self.time_list = []
        for i_s, (featurevec, succ) in enumerate(zip(featurevec_list, success_list)):
            radius, theta, height, azimuth_loc, zenith, _, _, _ = featurevec
            xyz = cyl2cart(radius, theta, height)
            orientation_mat = hori2mat(theta, azimuth_loc, zenith)
            if not succ:
                gscene.add_highlight_axis("hl", "toolvec_{}".format(i_s), "base_link", xyz, orientation_mat,
                                          color=(1,0,0,1), axis="z", dims=(0.10, 0.02, 0.02))
                continue
            gtimer.tic("sample_reaching")
            featurevec, success, trajectory = self.sample_reaching(ROBOT_NAME, tip_link, home_pose=crob.home_pose, timeout=timeout,
                                                                   radius_min=radius, radius_max=radius, theta_min=theta, theta_max=theta,
                                                                   height_min=height, height_max=height, zenith_min=zenith, zenith_max=zenith,
                                                                   azimuth_min=azimuth_loc, azimuth_max=azimuth_loc)
            gscene.add_highlight_axis("hl", "toolvec_{}".format(i_s), "base_link", xyz, orientation_mat,
                                      color=(0,0,1,1) if success else (1,0,0,1), axis="z", dims=(0.10, 0.02, 0.02))
            if success:
                self.time_list.append(gtimer.toc("sample_reaching"))
                gscene.show_motion(trajectory[::int(0.01/0.001)], period=0.01)
            assert np.linalg.norm(np.subtract(featurevec_list[i_s], featurevec))<1e-5, "featurevec changed"
            success_list[i_s] = success
            if i_s % 100 == 0 :
                t_cur = gtimer.toc("full_loop")
                print("{} / {} ({} / {} s): current success ratio = {}".format(
                    i_s, N_s, int(t_cur/1000), int(float(N_s)/float(i_s+1e-3)*t_cur/1000), np.mean(success_list)))

    def save_data(self, div, featurevec_list, success_list):
        robot_path = os.path.join(self.data_path, self.robot_type.name)
        try_mkdir(robot_path)
        div_path = os.path.join(self.data_path, self.robot_type.name, div)
        try_mkdir(div_path)
        save_json(os.path.join(div_path, "featurevec_list.json"), featurevec_list)
        save_json(os.path.join(div_path, "success_list.json"), success_list)

    def load_data(self, robot_type, div):
        self.robot_type = robot_type
        data_path = os.path.join(self.data_path, self.robot_type.name, div)
        return (load_json(os.path.join(data_path, "featurevec_list.json")),
                load_json(os.path.join(data_path, "success_list.json")))

    def save_model(self):
        try_mkdir(self.model_path)
        save_pickle(os.path.join(self.model_path, "{}.pkl".format(self.robot_type.name)), self.clf)

    def load_model(self, robot_type):
        self.robot_type = robot_type
        self.clf = load_pickle(os.path.join(self.model_path, "{}.pkl".format(self.robot_type.name)))
        return self.clf
