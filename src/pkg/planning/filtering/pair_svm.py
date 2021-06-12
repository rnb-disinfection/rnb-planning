import numpy as np
from .filter_interface import MotionFilterInterface
from ..constraint.constraint_common import calc_redundancy
from ...utils.joint_utils import *
from ...controller.combined_robot import *
from ...utils.utils import get_now, try_mkdir

MODEL_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "model")
try_mkdir(MODEL_PATH)

GF_MODEL_PATH = os.path.join(MODEL_PATH, "gf3d")
try_mkdir(GF_MODEL_PATH)

##
# @brief get geometric feature
def get_pairwise_feature(T_obj, dims_obj, obstacle_geo, plane_height):
    xvec = [1, 0, 0]
    xvec_obj = np.matmul(T_obj[:3, :3], xvec)
    theta_obj = np.arctan2(xvec_obj[1], xvec_obj[0])
    theta_obj_loc = (theta_obj + np.pi / 4) % (np.pi / 2) - np.pi / 4
    theta_obj_step = theta_obj - theta_obj_loc
    xvec_obs = np.matmul(obstacle_geo.orientation_mat, xvec)
    theta_obs = np.arctan2(xvec_obs[1], xvec_obs[0])
    theta_obs_loc = (theta_obs + np.pi / 4) % (np.pi / 2) - np.pi / 4
    theta_obs_step = theta_obs - theta_obs_loc

    delta_cylcoord = cart2cyl(*np.subtract(obstacle_geo.center, T_obj[:3, 3]))[:2]
    delta_cylcoord = list(delta_cylcoord) + [delta_cylcoord[1] ** 2]
    return list(np.abs(np.matmul(Rot_axis(3, theta_obj_step), dims_obj))) + \
           list(np.abs(np.matmul(Rot_axis(3, theta_obs_step), obstacle_geo.dims))) + \
           list(cart2cyl(*T_obj[:3, 3])[:2]) + list(cart2cyl(*obstacle_geo.center)[:2]) + delta_cylcoord + \
           [plane_height, theta_obj_loc, theta_obs_loc]


##
# @param Tbg transformation matrix from robot base to gripper coordinate
def get_step_dirYZ(Tbg):
    rotY, rotZ = Tbg[:3,1], Tbg[:3,2]
    idxY, idxZ = np.argmax(np.abs(rotY)), np.argmax(np.abs(rotZ))
    sgnY, sgnZ = np.sign(rotY[idxY]), np.sign(rotZ[idxZ])
    dirY, dirZ = np.zeros(3,dtype=np.int), np.zeros(3,dtype=np.int)
    dirY[idxY], dirZ[idxZ] = sgnY, sgnZ
    return (tuple(dirY), tuple(dirZ))


##
# @class    PairSVM
# @brief    feature-based pair-wise feasibility checker, following A. M. Wells paper:
#           (Wells, Andrew M., et al. "Learning feasibility for task and motion planning in tabletop environments." IEEE robotics and automation letters 4.2 (2019): 1255-1262.)
class PairSVM(MotionFilterInterface):

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param put_banned GeometryItem list to indicate area where not to put object
    def __init__(self, pscene):
        self.pscene = pscene
        self.combined_robot = pscene.combined_robot
        self.robot_names = pscene.combined_robot.robot_names
        chain_dict = pscene.robot_chain_dict
        binder_links = [chain_dict[rname]['tip_link'] for rname in self.robot_names]
        self.binder_link_robot_dict = {blink: rname for blink, rname in zip(binder_links, self.robot_names)}
        self.base_dict = self.combined_robot.get_robot_base_dict()

        self.model_dict = {}
        for rconfig in self.combined_robot.robots_on_scene:
            self.model_dict[rconfig.get_indexed_name()] = self.load_model(rconfig.type)

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    # @param ignore         GeometryItems to ignore
    def check(self, actor, obj, handle, redundancy_values, Q_dict, interpolate=False, obj_only=False, ignore=[],
              **kwargs):
        # gtimer = GlobalTimer.instance()
        # gtimer.tic("get_grasping_vert_infos")
        point_add_handle, rpy_add_handle = redundancy_values[(obj.oname, handle.name)]
        point_add_actor, rpy_add_actor = redundancy_values[(obj.oname, actor.name)]
        T_handle_lh = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_loal = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))

        return self.check_T_loal(actor, obj, T_loal, Q_dict, interpolate=interpolate, obj_only=obj_only, ignore=ignore,
                                 **kwargs)

    ##
    # @brief check end-effector collision in grasping
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

        group_name_handle = self.binder_link_robot_dict[
            object_link] if object_link in self.binder_link_robot_dict else None
        group_name_actor = self.binder_link_robot_dict[
            actor_link] if actor_link in self.binder_link_robot_dict else None

        if group_name_actor and not group_name_handle:
            link_tar = object_link
            group_name = group_name_actor
            gripper = actor
            base_link = self.base_dict[group_name]
            T_handle_link = get_tf(object_link, Q_dict, self.pscene.gscene.urdf_content,
                                   from_link=base_link)
            T_lbrl = np.matmul(T_handle_link, T_loal)  # T from robot base to object to handle to gripper to robot link
            Tobj = obj.geometry.get_tf(Q_dict, from_link=base_link)
        elif group_name_handle and not group_name_actor:
            link_tar = actor_link
            group_name = group_name_handle
            gripper = self.pscene.actor_dict[self.pscene.robot_actor_dict[group_name]]
            base_link = self.base_dict[group_name]
            T_actor_link = get_tf(actor_link, Q_dict, self.pscene.gscene.urdf_content,
                                  from_link=base_link)
            T_lbrl = np.matmul(T_actor_link,
                               SE3_inv(T_loal))  # T from robot base to placement actor to attached handle to robot link
            T_rlo = obj.geometry.Toff
            Tobj = np.matmul(T_lbrl, T_rlo)
        else:
            print("group_name fail: {} - {}".format(group_name_actor, group_name_handle))
            print("group_name fail: {} - {}".format(actor.geometry.name, obj.geometry.name))
            # dual motion not predictable
            return True

        tool_T = self.get_tool_T(group_name, gripper)
        Trobot = np.matmul(T_lbrl, tool_T)
        clf = self.get_svm(group_name, Trobot)

        plane_height = Tobj[2, 3] - obj.geometry.dims[2] / 2
        res = True
        for gtem in self.pscene.gscene:
            if gtem.collision and gtem.link_name == link_tar and gtem != obj.geometry:
                T_gtem = gtem.get_tf(Q_dict, from_link=base_link)
                H_gtem = T_gtem[2, 3]
                B_gtem = H_gtem - gtem.dims[2] / 2
                if H_gtem > plane_height and B_gtem < plane_height + 1e-2:  ## higher than plane, but bottom near plane
                    obstacle_geo = gtem
                    featurevec = get_pairwise_feature(Tobj, obj.geometry.dims, obstacle_geo, plane_height)
#                     print("CHECKER FEA {}: {}".format(gtem.name, np.round(featurevec, 2)))
#                     print("Tobj: \n {}".format(Tobj))
                    if not clf.predict([featurevec])[0]:
                        res = False
                        break
#                 else:
#                     print("H {}: {} / {}".format(gtem.name, H_gtem, plane_height))
#                     print("B {}: {} / {}".format(gtem.name, B_gtem, plane_height))
        return res

    def get_tool_T(self, rname, gripper):
        model = self.model_dict[rname]
        tool_T = model["tool_T"]
        assert gripper.geometry.link_name == model["tool_link"], "gripper is not linked in the same way as trained"
        assert np.sum(np.abs(gripper.Toff_lh - tool_T)) < 1e5, "gripper offset if different from the trained model"
        return tool_T

    ##
    # @brief get geometric feature
    def get_svm(self, rname, Trobot):
        model = self.model_dict[rname]
        clf_dict = model["clf_dict"]
        found = None
        maxval = 0
        dirkey = get_step_dirYZ(Trobot)
#         print("CHECKER DIR: {}".format(dirkey))
        assert dirkey in clf_dict, "svm corresponding to given direction not found"
        return clf_dict[dirkey]

    def load_model(self, robot_type):
        model = load_pickle(os.path.join(GF_MODEL_PATH, "{}.pkl".format(robot_type.name)))
        return model