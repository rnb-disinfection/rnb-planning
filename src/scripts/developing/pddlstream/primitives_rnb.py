import os
import sys
sys.path.insert(1, os.environ["PDDL_STREAM_DIR"])
sys.path.insert(1, os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
from pkg.utils.rotation_utils import *
from pkg.utils.utils import *
from primitives_pybullet import *

TIMEOUT_MOTION = 1

def plan_motion(mplan, body_subject_map, conf1, conf2, grasp, fluents, tool_link, base_link="base_link"):
    pscene = mplan.pscene
    for fluent in fluents:
        subject = body_subject_map[fluent[1]]
        Tbase = T_xyzquat(fluent[2].value)
        subject.geometry.set_link(base_link)
        subject.geometry.set_offset_tf(center=Tbase[:3,3], orientation_mat=Tbase[:3,:3])
    if grasp is not None:
        graspped = body_subject_map[grasp.body]
        Tgrasp = T_xyzquat(grasp.grasp_pose)
        graspped.geometry.set_link(tool_link)
        graspped.geometry.set_offset_tf(center=Tgrasp[:3,3], orientation_mat=Tgrasp[:3,:3])
    Qcur, Qto = conf1.values, conf2.values
    from_state = pscene.initialize_state(np.array(Qcur))
    to_state = from_state.copy(pscene)
    to_state.Q = np.array(Qto)
    Traj, LastQ, error, success, binding_list = mplan.plan_transition(from_state, to_state, {}, timeout=TIMEOUT_MOTION)
    return Traj, success

def get_free_motion_gen_rnb(mplan, body_subject_map, robot, tool_link, base_link="base_link"):
    def fn(conf1, conf2, fluents=[]):
        with GlobalTimer.instance().block("free_motion_gen"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            conf1.assign()
            path, succ = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                     conf1=conf1, conf2=conf2, grasp=None, fluents=fluents, tool_link=tool_link, base_link=base_link)
            if not succ:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
            command = Command([BodyPath(robot, path, joints=conf2.joints)])
            return (command,)
    return fn


def get_holding_motion_gen_rnb(mplan, body_subject_map, robot, tool_link, base_link="base_link"):
    def fn(conf1, conf2, body, grasp, fluents=[]):
        with GlobalTimer.instance().block("holding_motion_gen"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            conf1.assign()
            path, succ = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                     conf1=conf1, conf2=conf2, grasp=grasp, fluents=fluents, tool_link=tool_link, base_link=base_link)
            if not succ:
                if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
                return None
            command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
            return (command,)
    return fn


def get_free_motion_gen_ori(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        with GlobalTimer.instance().block("free_motion_gen_ori"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            if teleport:
                path = [conf1.configuration, conf2.configuration]
            else:
                conf1.assign()
                obstacles = fixed + assign_fluent_state(fluents)
                path = plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=obstacles, self_collisions=self_collisions)
                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                    return None
            command = Command([BodyPath(robot, path, joints=conf2.joints)])
            return (command,)
    return fn


def get_holding_motion_gen_ori(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, body, grasp, fluents=[]):
        with GlobalTimer.instance().block("holding_motion_gen_ori"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            if teleport:
                path = [conf1.configuration, conf2.configuration]
            else:
                conf1.assign()
                obstacles = fixed + assign_fluent_state(fluents)
                path = plan_joint_motion(robot, conf2.joints, conf2.configuration,
                                         obstacles=obstacles, attachments=[grasp.attachment()], self_collisions=self_collisions)
                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
                    return None
            command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
            return (command,)
    return fn


def check_feas(pscene, body_subject_map, actor, checkers, home_dict, body, pose, grasp, base_link="base_link"):
    assert body == grasp.body
    gtimer = GlobalTimer.instance()
    with gtimer.block('check_feas'):
        subject = body_subject_map[body]
        Tbo = T_xyzquat(pose.value)
        subject.geometry.set_link(base_link)
        subject.geometry.set_offset_tf(center=Tbo[:3,3], orientation_mat=Tbo[:3,:3])

        Tlao = T_xyzquat(grasp.grasp_pose)
        Tboal = np.matmul(Tbo, SE3_inv(Tlao))
        Tboal_ap = np.matmul(Tboal, T_xyzquat(grasp.approach_pose))
        # print("check_feas Tboal: {}".format(T2xyzquat(Tboal)))
        # print("check_feas Tboal: {}".format(T2xyzquat(Tboal_ap)))

        ignore = []
        for k,v in body_subject_map.items():
            if k != body:
                ignore += [pscene.gscene.NAME_DICT[gname] for gname in v.geometry.get_family()]

        for checker in checkers:
            with gtimer.block(checker.__class__.__name__):
                if not checker.check_T_loal(actor, subject, Tboal, home_dict, ignore=ignore):
                    return False
                if not checker.check_T_loal(actor, subject, Tboal_ap, home_dict, ignore=ignore):
                    return False
    return True

def get_ik_fn_rnb(pscene, body_subject_map, actor, checkers, home_dict, base_link="base_link",
                  disabled_collisions=set(), robot=0L, fixed=[], teleport=False, num_attempts=10):
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    def fn(body, pose, grasp):
        with GlobalTimer.instance().block("ik_fn"):
            if not check_feas(pscene, body_subject_map, actor, checkers,
                              home_dict, body, pose, grasp, base_link=base_link):
                return None
            obstacles = [body] + fixed
            gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
            approach_pose = approach_from_grasp_tool_side(grasp.approach_pose, gripper_pose)
            # print("gripper_pose: {}".format(gripper_pose))
            # print("approach_pose: {}".format(approach_pose))
            for _ in range(num_attempts):
                set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
                # TODO: multiple attempts?
                q_approach = inverse_kinematics(robot, grasp.link, approach_pose)
                print("q_approach: {}".format(q_approach))
                if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                    print("obstacles: {}".format(obstacles))
                    continue
                print("go on")
                conf = BodyConf(robot, q_approach)
                q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)
                print("q_grasp: {}".format(q_grasp))
                if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                    print("obstacles: {}".format(obstacles))
                    continue
                if teleport:
                    path = [q_approach, q_grasp]
                else:
                    conf.assign()
                    #direction, _ = grasp.approach_pose
                    #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                    #                                   quat_from_pose(approach_pose))
                    path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles,
                                                    disabled_collisions=disabled_collisions)
                    if path is None:
                        print("Approach motion failed")
                        if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
                        continue
                command = Command([BodyPath(robot, path),
                                   Attach(body, robot, grasp.link),
                                   BodyPath(robot, path[::-1], attachments=[grasp])])
                return (conf, command)
                # TODO: holding collisions
            return None
    return fn