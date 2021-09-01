import os
import sys
sys.path.insert(1, os.environ["PDDL_STREAM_DIR"])
sys.path.insert(1, os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
from ...utils.rotation_utils import *
from ...utils.joint_utils import *
from ...utils.utils import *
from ..constraint.constraint_common import *
from primitives_pybullet import *

TIMEOUT_MOTION_DEFAULT = 1

gtimer= GlobalTimer.instance()

def get_matching_object(pscene, binder, Q_dict, approach_vec):
    margin_max = -1e10
    max_point = ""
    max_binder = ""
    binder_T = binder.get_tf_handle(Q_dict)
    if approach_vec is not None:
        binder_link_T = np.matmul(binder_T, SE3_inv(binder.Toff_lh))
        binder_link_T_approached = np.matmul(binder_link_T, SE3(np.identity(3), np.negative(approach_vec)))
        binder_T = np.matmul(binder_link_T_approached, binder.Toff_lh)


    for sname, subject in pscene.subject_dict.items():
        self_family = subject.geometry.get_family()
        ## find best binding between object and binders
        for kpt, handle in subject.action_points_dict.items():
            handle_T = handle.get_tf_handle(Q_dict)

            if binder.check_available(Q_dict):
                if binder.geometry.name in self_family or not binder.check_type(handle):
                    continue
                binder_redundancy = binder.get_redundancy()
                handle_redundancy = handle.get_redundancy()
                margins = get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy,
                                              rot_scale=1e-2)
                margin_min = np.min(margins) # get max. deviation (negative)
                if margin_min > margin_max: # pick smallest max. deviation (negative)
                    margin_max = margin_min
                    max_point = handle.name
                    max_subject = subject
    if margin_max < -1e-2:
        return None
    return max_subject

def get_matching_binder(pscene, subject, Q_dict, excludes=[]):
    margin_max = -1e10
    max_point = ""
    max_binder = ""
    actor_dict = {bname: binder for bname, binder in pscene.actor_dict.items()
                   if bname not in excludes and binder not in excludes}
    binder_T_dict = {bname: binder.get_tf_handle(Q_dict) for bname, binder in actor_dict.items()}
    self_family = subject.geometry.get_family()
    ## find best binding between object and binders
    for kpt, handle in subject.action_points_dict.items():
        handle_T = handle.get_tf_handle(Q_dict)

        for bname, binder in actor_dict.items():
            if binder.check_available(Q_dict):
                binder_T = binder_T_dict[bname]
                if binder.geometry.name in self_family or not binder.check_type(handle):
                    continue
                binder_redundancy = binder.get_redundancy()
                handle_redundancy = handle.get_redundancy()
                margins = get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy,
                                              rot_scale=1e-2)
                margin_min = np.min(margins)
                if margin_min > margin_max:
                    margin_max = margin_min
                    max_point = handle.name
                    max_binder = binder
    return max_binder


def plan_motion(mplan, body_subject_map, conf1, conf2, grasp, fluents, tool, tool_link, approach_vec=None,
                timeout=TIMEOUT_MOTION_DEFAULT, base_link="base_link", show_state=False):
    pscene = mplan.pscene
    for fluent in fluents:
        subject = body_subject_map[fluent[1]]
        Tbase = T_xyzquat(fluent[2].value)
        subject.set_state(binding=(subject.oname, None, None, None),
                          state_param=(base_link, Tbase))

    if grasp is not None:
        graspped = body_subject_map[grasp.body]
        Tgrasp = T_xyzquat(grasp.grasp_pose)
        graspped.set_state(binding=(graspped.oname, None, tool.geometry.name, tool.name),
                          state_param=(tool_link, Tgrasp))
    Qcur, Qto = conf1.values, conf2.values
    from_state = pscene.initialize_state(np.array(Qcur))
    if show_state:
        pscene.gscene.clear_highlight()
        pscene.gscene.update_markers_all()
    to_state = from_state.copy(pscene)
    to_state.Q = np.array(Qto)

    Qto_dict = list2dict(Qto, pscene.gscene.joint_names)

    if grasp is None: # free motion, go to grasp - actor:tool
        subject = get_matching_object(pscene,tool, Qto_dict, approach_vec)
        actor = tool
    else: # holding motion, go to release - actor:plane
        subject = graspped
        actor = get_matching_binder(pscene, subject, Qto_dict, excludes=[tool])

    # GlobalLogger.instance()["from_state"] = from_state
    # GlobalLogger.instance()["to_state"] = to_state
    # GlobalLogger.instance()["subject"] = subject.oname if hasattr(subject, "oname") else subject
    # GlobalLogger.instance()["actor"] = actor.name if hasattr(actor, "name") else actor
    ## Check motion_filters outside, as plan_transition below will do joint motion - filters will be skipped
    res = True
    if subject is not None:
        Tloal_list = [
            get_tf(to_link=actor.geometry.link_name, from_link=subject.geometry.link_name,
                   joint_dict=Qto_dict, urdf_content=pscene.gscene.urdf_content)]
        res = run_checkers(mplan.motion_filters, actor, subject, Tloal_list,
                     Q_dict=list2dict(Qcur, pscene.gscene.joint_names), show_state=show_state, mplan=mplan)
    if mplan.flag_log:
        mplan.result_log["pre_motion_checks"].append(res)
    if res:
        Traj, LastQ, error, success, binding_list = mplan.plan_transition(
            from_state, to_state, {}, timeout=timeout, show_state=show_state)
    else:
        Traj, success = [], False
    if show_state:
        pscene.gscene.show_motion(Traj)
    return Traj, success

def get_free_motion_gen_rnb(mplan, body_subject_map, robot, tool, tool_link, approach_vec, timeout=TIMEOUT_MOTION_DEFAULT,
                            base_link="base_link", show_state=False):
    def fn(conf1, conf2, fluents=[]):
        with GlobalTimer.instance().block("free_motion_gen"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            conf1.assign()
            path, succ = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                     conf1=conf1, conf2=conf2, grasp=None, fluents=fluents, tool=tool,
                                     tool_link=tool_link, base_link=base_link, timeout=timeout,
                                     show_state=show_state, approach_vec=approach_vec)
            if not succ:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
            command = Command([BodyPath(robot, path, joints=conf2.joints)])
            return (command,)
    return fn


def get_holding_motion_gen_rnb(mplan, body_subject_map, robot, tool, tool_link, approach_vec, timeout=TIMEOUT_MOTION_DEFAULT,
                               base_link="base_link", show_state=False):
    def fn(conf1, conf2, body, grasp, fluents=[]):
        with GlobalTimer.instance().block("holding_motion_gen"):
            assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
            conf1.assign()
            path, succ = plan_motion(mplan=mplan, body_subject_map=body_subject_map,
                                     conf1=conf1, conf2=conf2, grasp=grasp, fluents=fluents, tool=tool,
                                     tool_link=tool_link, base_link=base_link, timeout=timeout,
                                     show_state=show_state, approach_vec=approach_vec)
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
                attachment = grasp.attachment()
                attachment.assign()
                path = plan_joint_motion(robot, conf2.joints, conf2.configuration,
                                         obstacles=obstacles, attachments=[attachment], self_collisions=self_collisions)
                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
                    return None
            command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
            return (command,)
    return fn


def check_feas(pscene, body_subject_map, actor, checkers, home_dict, body, pose, grasp,
               base_link="base_link", show_state=False):
    assert body == grasp.body
    gtimer = GlobalTimer.instance()
    if show_state:
        pscene.gscene.clear_highlight()
    with gtimer.block('check_feas'):
        subject = body_subject_map[body]
        Tbo = T_xyzquat(pose.value)
        subject.set_state(binding=(subject.oname, None, None, None),
                          state_param=(base_link, Tbo))

        Tlao = T_xyzquat(grasp.grasp_pose)
        Tboal = np.matmul(Tbo, SE3_inv(Tlao))
        Tboal_ap = np.matmul(Tboal, T_xyzquat(grasp.approach_pose))
        # print("check_feas Tboal: {}".format(T2xyzquat(Tboal)))
        # print("check_feas Tboal: {}".format(T2xyzquat(Tboal_ap)))

        ignore = []
        for k,v in body_subject_map.items():
            if k != body:
                ignore += [pscene.gscene.NAME_DICT[gname] for gname in v.geometry.get_family()]
        if show_state:
            display_bak = []
            for ig_tem in ignore:
                display_bak.append(ig_tem.display)
                ig_tem.display = False
                pscene.gscene.update_marker(ig_tem)

        if show_state:
            pscene.gscene.update_markers_all()
            pscene.gscene.add_highlight_axis("feas", "obj", "base_link",
                                             center=Tbo[:3,3], orientation_mat=Tbo[:3,:3])

        res = run_checkers(checkers, actor, subject, [Tboal, Tboal_ap], home_dict, ignore, show_state=show_state)

        if show_state:
            for ig_tem, disp in zip(ignore, display_bak):
                ig_tem.display = disp
                pscene.gscene.update_marker(ig_tem)
    return res

def run_checkers(checkers, actor, subject, Tloal_list, Q_dict, ignore=[], show_state=False, mplan=None):
    res = True
    for i_c, checker in enumerate(checkers):
        fname = checker.__class__.__name__
        flag_log = mplan is not None and mplan.flag_log
        if flag_log:
            mplan.gtimer.tic(fname)
        for Tloal in Tloal_list:
            if not checker.check_T_loal(actor, subject, Tloal, Q_dict, ignore=ignore):
                res = False
                break
        if flag_log:
            mplan.gtimer.toc(fname)
            mplan.result_log[fname].append(res)
        if not res:
            break
    if show_state:
        gscene = subject.geometry.gscene
        if subject.geometry.link_name == "base_link":
            Tbal_list = [Tloal for Tloal in Tloal_list]
        elif actor.geometry.link_name == "base_link":
            Tbal_list = [SE3_inv(Tloal) for Tloal in Tloal_list]
        else:
            raise(NotImplementedError("undefined binding case - possibly dual arm manipulation"))

        for i_t, Tbal in enumerate(Tbal_list):
            gscene.add_highlight_axis("feas", "tlink_{}".format(i_t), "base_link",
                                             center=Tbal[:3,3], orientation_mat=Tbal[:3,:3])

        if not res:
            print("Check Feas Fail: {}".format(checker.__class__.__name__))
            vis_bak = gscene.highlight_robot(color=gscene.COLOR_LIST[i_c])
        time.sleep(0.5)
        if not res:
            gscene.recover_robot(vis_bak)
    return res

def get_ik_fn_rnb(pscene, body_subject_map, actor, checkers, home_dict, base_link="base_link", show_state=False,
                  disabled_collisions=set(), robot=0L, fixed=[], teleport=False, num_attempts=10):
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    eye4 = np.identity(4)
    def fn(body, pose, grasp):
        with GlobalTimer.instance().block("ik_fn"):
            if show_state:
                pscene.gscene.show_pose(dict2list(home_dict, pscene.gscene.joint_names))
            if not check_feas(pscene, body_subject_map, actor, checkers,
                              home_dict, body, pose, grasp, base_link=base_link, show_state=show_state):
                fn.checkout_count += 1
                return None
            obstacles = [body] + fixed
            set_pose(body, pose.pose)
            gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
            approach_pose = approach_from_grasp_tool_side(grasp.approach_pose, gripper_pose)
            # print("gripper_pose: {}".format(gripper_pose))
            # print("approach_pose: {}".format(approach_pose))
            for i_ in range(num_attempts):
                if show_state:
                    pscene.gscene.show_pose(dict2list(home_dict, pscene.gscene.joint_names))
                set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
                # TODO: multiple attempts?
                q_approach = inverse_kinematics(robot, grasp.link, approach_pose)
                if show_state and q_approach is not None:
                    print("inverse_kinematics fail to approach")
                    pscene.gscene.show_pose(q_approach)
                # print("q_approach: {}".format(q_approach))
                if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                    # print("obstacles: {}".format(obstacles))
                    if show_state:
                        print("IK-col approach fail")
                        if q_approach is None:
                            color = pscene.gscene.COLOR_LIST[2]
                        else:
                            color = pscene.gscene.COLOR_LIST[0]
                        vis_bak = pscene.gscene.highlight_robot(color)
                        time.sleep(0.5)
                        pscene.gscene.recover_robot(vis_bak)
                    continue
                # print("go on")
                conf = BodyConf(robot, q_approach)
                if np.sum(np.abs(eye4 - T_xyzquat(grasp.approach_pose))) < 1e-6:
                    q_grasp = deepcopy(q_approach)
                else:
                    q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)
                    if show_state and q_grasp is not None:
                        print("inverse_kinematics fail to grasp")
                        pscene.gscene.show_pose(q_grasp)
                    if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                        # print("obstacles: {}".format(obstacles))
                        if show_state:
                            print("IK-col grasp fail")
                            if q_grasp is None:
                                color = pscene.gscene.COLOR_LIST[2]
                            else:
                                color = pscene.gscene.COLOR_LIST[0]
                            vis_bak = pscene.gscene.highlight_robot(color)
                            time.sleep(0.5)
                            pscene.gscene.recover_robot(vis_bak)
                        continue
                if show_state:
                    time.sleep(0.5)
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
                        # print("Approach motion failed")
                        if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
                        continue
                command = Command([BodyPath(robot, path),
                                   Attach(body, robot, grasp.link),
                                   BodyPath(robot, path[::-1], attachments=[grasp])])
                fn.pass_count += 1
                return (conf, command)
                # TODO: holding collisions
            fn.fail_count += 1
            return None
    return fn