import numpy as np
from .filter_interface import MotionFilterInterface
from ..constraint.constraint_common import calc_redundancy
from ...utils.joint_utils import *
from ...utils.gjk import get_point_list, get_gjk_distance


##
# @class    ReachChecker
# @brief    check reach regarding kinematic chain
# @remark   You need to train ReachTrainer tu use ReachChecker. See src/scripts/training/ReachSVM.ipynb
class ReachChecker(MotionFilterInterface):
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene
    def __init__(self, pscene):
        self.pscene = pscene
        self.combined_robot = pscene.combined_robot
        self.model_dict = {}
        self.robot_names = pscene.combined_robot.robot_names
        chain_dict = pscene.robot_chain_dict
        binder_links = [chain_dict[rname]['tip_link'] for rname in self.robot_names]
        self.binder_link_robot_dict = {blink: rname for blink, rname in zip(binder_links, self.robot_names)}
        for rconfig in self.combined_robot.robots_on_scene:
            self.model_dict[rconfig.get_indexed_name()] = ReachTrainer(None).load_model(rconfig.type)
        self.base_dict = self.combined_robot.get_robot_base_dict()
        self.shoulder_link_dict = {rname: pscene.gscene.urdf_content.joint_map[rchain['joint_names'][1]].child
                                   for rname, rchain in chain_dict.items()}
        self.shoulder_height_dict = {rname: get_tf(shoulder_link, self.combined_robot.home_dict,
                                                   pscene.gscene.urdf_content, from_link=self.base_dict[rname])[2,3]
                                     for rname, shoulder_link in self.shoulder_link_dict.items()}

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    def check(self, actor, obj, handle, redundancy_values, Q_dict, interpolate=False, **kwargs):

        point_add_handle, rpy_add_handle = redundancy_values[(obj.oname, handle.name)]
        point_add_actor, rpy_add_actor = redundancy_values[(obj.oname, actor.name)]
        T_handle_lh = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_link_handle_actor_link = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))
        return self.check_T_loal(group_name, Ttar)

    ##
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param T_loal     transformation matrix from object-side link to actor-side link
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    # @param ignore         GeometryItems to ignore
    def check_T_loal(self, actor, obj, T_loal, Q_dict, interpolate=False, obj_only=False, ignore=[],
              **kwargs):

        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name

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
        radius, theta, height = cart2cyl(*T_tar[:3,3])
        azimuth_loc, zenith = mat2hori(T_tar[:3,:3], theta)
        shoulder_height = self.shoulder_height_dict[group_name]
        ee_dist = np.linalg.norm([radius, height-shoulder_height])
        featurevec = (radius, theta, height, azimuth_loc, zenith, radius**2, ee_dist, ee_dist**2)
        return self.model_dict[group_name].predict([featurevec])[0]


from ...utils.utils import *
from ...controller.combined_robot import CombinedRobot, RobotConfig
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
                          save_data=True, save_model=True, C_svm=500, timeout=1):
        self.featurevec_list_train, self.success_list_train = self.collect_reaching_data(ROBOT_TYPE, END_LINK, TRAIN_COUNT, timeout=timeout)

        self.featurevec_list_test, self.success_list_test = self.collect_reaching_data(ROBOT_TYPE, END_LINK, TEST_COUNT, timeout=timeout)
        if save_data:
            self.save_data("train", self.featurevec_list_train, self.success_list_train)
            self.save_data("test", self.featurevec_list_test, self.success_list_test)

        feature_mat_train = np.array(self.featurevec_list_train)
        self.clf = svm.SVC(kernel='rbf', C=C_svm)
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
    def load_and_learn(self, ROBOT_TYPE, C_svm=500, save_model=True):
        self.featurevec_list_train, self.success_list_train = self.load_data(ROBOT_TYPE, "train")
        self.featurevec_list_test, self.success_list_test = self.load_data(ROBOT_TYPE, "test")

        feature_mat_train = np.array(self.featurevec_list_train)
        feature_mat_test = np.array(self.featurevec_list_test)

        self.clf = svm.SVC(kernel='rbf', C=C_svm)
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

    ##
    # @brief load and test
    def load_and_test(self, ROBOT_TYPE):
        self.featurevec_list_train, self.success_list_train = self.load_data(ROBOT_TYPE, "train")
        self.featurevec_list_test, self.success_list_test = self.load_data(ROBOT_TYPE, "test")
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

    ##
    # @brief sample reaching plan results
    # @remark accuracy decreases when very small radius is included in the dataset. radius lowerbound 0.2 is recommended
    def sample_reaching(self, robot_name, tool_link, home_pose, base_link="base_link", timeout=1,
                        radius_min=0.2, radius_max=1.5, theta_min=-np.pi, theta_max=np.pi,
                        height_min=-0.7, height_max=1.5, zenith_min=0, zenith_max=np.pi,
                        azimuth_min=-np.pi, azimuth_max=np.pi):
        radius = random.uniform(radius_min, radius_max)
        theta = random.uniform(theta_min, theta_max)
        height = random.uniform(height_min, height_max)
        azimuth_loc = random.uniform(azimuth_min, azimuth_max)
        zenith = random.uniform(zenith_min, zenith_max)

        xyz = cyl2cart(radius, theta, height)
        quat = tuple(Rotation.from_dcm(hori2mat(theta, azimuth_loc, zenith)).as_quat())
        goal_pose = xyz+quat
        GlobalTimer.instance().tic("plan_py")
        trajectory, success = self.planner.planner.plan_py(
            robot_name, tool_link, goal_pose, base_link, tuple(home_pose), timeout=timeout)
        self.time_plan.append(GlobalTimer.instance().toc("plan_py"))

        ee_dist = np.linalg.norm([radius, height-self.shoulder_height])
        return (radius, theta, height, azimuth_loc, zenith, radius**2, ee_dist, ee_dist**2), success, trajectory

    def collect_reaching_data(self, robot_type, TIP_LINK, N_s, timeout=1):
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
        pscene.create_binder(bname="grip0", gname="grip0", rname=ROBOT_NAME, _type=Gripper2Tool, point=(0,0,0), rpy=(0,0,0))

        self.planner = MoveitPlanner(pscene)
        self.planner.update_gscene()

        self.time_plan = []
        gtimer = GlobalTimer.instance()
        gtimer.reset()
        gtimer.tic("full_loop")
        featurevec_list = []
        success_list = []
        self.time_list = []
        for i_s in range(N_s):
            gtimer.tic("sample_reaching")
            featurevec, success, trajectory = self.sample_reaching(ROBOT_NAME, TIP_LINK, home_pose=crob.home_pose, timeout=timeout)
            self.time_list.append(gtimer.toc("sample_reaching"))
            xyz = cyl2cart(*featurevec[:3])
            orientation_mat = hori2mat(featurevec[1], *featurevec[-2:])
    #         gscene.add_highlight_axis("hl", "toolvec", "base_link", xyz, orientation_mat)
            featurevec_list.append(featurevec)
            success_list.append(success)
            if i_s % 100 == 0 :
                t_cur = gtimer.toc("full_loop")
                print("{} / {} ({} / {} s): current success ratio = {}".format(
                    i_s, N_s, int(t_cur/1000), int(float(N_s)/float(i_s+1e-3)*t_cur/1000), np.mean(success_list)))

        return featurevec_list, success_list

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
        save_pickle(os.path.join(self.model_path, "{}.json".format(self.robot_type.name)), self.clf)

    def load_model(self, robot_type):
        self.robot_type = robot_type
        self.clf = load_pickle(os.path.join(self.model_path, "{}.json".format(self.robot_type.name)))
        return self.clf
